# Copyright (c) 2013, Shadow Robot Company, SynTouch LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
import rospkg
import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtGui import QIcon, QColor, QPainter, QFont
from QtWidgets import QMessageBox, QWidget
from QtCore import QRectF, QTimer
from pr2_gripper_sensor_msgs.msg import PR2GripperSensorRawData


class Pr2GripperSensorGui(Plugin):
    _nb_electrodes_biotac = 19
    _nb_electrodes_biotac_sp = 24

    def define_electrodes(self):
        self.electrode_pos_x = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_x_locations",
                [0,100,200,0,100,200,0,100,200,0,100,200,0,100,200,
                -100, -100, 300, 300,
                0, 150, 0])
        self.electrode_pos_y = \
            rospy.get_param(
                "sr_gui_biotac/sensing_electrodes_y_locations",
                [0,0,0,100,100,100,200,200,200,300,300,300,400,400,400,
                0, 250, 0, 250,
                -100, -100, -200])
        self.sensing_electrodes_w = \
                [90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,
                90, 90, 90, 90,
                140, 140, 290]
        self.sensing_electrodes_h = \
                [90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,
                240, 240, 240, 240,
                90, 90, 90]

    def assign_electrodes(self, nb_electrodes):
        if nb_electrodes == self._nb_electrodes_biotac:
            self.sensing_electrodes_x = self.electrode_pos_x
            self.sensing_electrodes_y = self.electrode_pos_y
        else:
            rospy.logerr("Number of electrodes %d not matching known biotac models. expected: %d or %d",
                         nb_electrodes, self._nb_electrodes_biotac, self._nb_electrodes_biotac_sp)
            return

        for n in range(len(self.sensing_electrodes_x)):
            self.sensing_electrodes_x[n] = (
                self.sensing_electrodes_x[n] +
                self.x_display_offset[0])
            self.sensing_electrodes_y[n] = (
                self.sensing_electrodes_y[n] +
                self.y_display_offset[0])

    def tactile_cb(self, msg):
        #if len(msg.tactiles[0].electrodes) != self._nb_electrodes:
        #    self._nb_electrodes = len(msg.tactiles[0].electrodes)
        #    self.assign_electrodes(self._nb_electrodes)
        self.latest_data = msg

    def get_electrode_colour_from_value(self, value):
        r = 0.0
        g = 0.0
        b = 255.0

        value = float(value)

        b = 255 * value/5000 
        return QColor(r, g, b)

        threshold = (0.0, 1000.0, 2000.0, 3000.0, 4095.0)

        if value <= threshold[0]:
            pass

        elif value < threshold[1]:

            r = 255
            g = 255 * ((value - threshold[0]) / (threshold[1] - threshold[0]))
            b = 0

        elif value < threshold[2]:

            r = 255 * ((threshold[2] - value) / (threshold[2] - threshold[1]))
            g = 255
            b = 0

        elif value < threshold[3]:

            r = 0
            g = 255
            b = 255 * ((value - threshold[2]) / (threshold[3] - threshold[2]))

        elif value < threshold[4]:

            r = 0
            g = 255 * ((threshold[4] - value) / (threshold[4] - threshold[3]))
            b = 255

        return QColor(r, g, b)

    def draw_electrode(self, painter, rect_x, rect_y, rect_w, rect_h, text_x, text_y,
                       colour, text):

        rect = QRectF(rect_x, rect_y, rect_w, rect_h)

        painter.setBrush(colour)
        painter.drawRect(rect)

        rect.setX(text_x)
        rect.setY(text_y)

        painter.drawText(rect, text)

    def paintEvent(self, paintEvent):
        painter = QPainter(self._widget.scrollAreaWidgetContents)
        which_tactile = 0 

        painter.setFont(QFont("Arial", self.label_font_size[0]))

        for n in range(len(self.sensing_electrodes_x)):
            value = self.latest_data.left_finger_pad_forces[n]
            eval("self._widget.lcdE%02d.display(%d)" % (n + 1, value))
            colour = self.get_electrode_colour_from_value(value)

            rect_x = self.sensing_electrodes_x[n]
            rect_y = self.sensing_electrodes_y[n]
            rect_w = self.sensing_electrodes_w[n]
            rect_h = self.sensing_electrodes_h[n]

            if n < 9:
                text_x = rect_x + self.x_display_offset[1]
                text_y = rect_y + self.y_display_offset[1]

            else:
                text_x = rect_x + self.x_display_offset[2]
                text_y = rect_y + self.y_display_offset[2]

            self.draw_electrode(painter, rect_x, rect_y, rect_w, rect_h, text_x, text_y,
                                colour, str(n + 1))

        painter.setFont(QFont("Arial", self.label_font_size[1]))

        self._widget.update()

    def subscribe_to_topic(self, prefix):
        if prefix:
            rospy.Subscriber(prefix, PR2GripperSensorRawData, self.tactile_cb)

    def load_params(self):
        self.RECTANGLE_WIDTH = rospy.get_param(
            "sr_gui_biotac/electrode_display_width",
            90)  # Display sizes for electrodes in pixels
        self.RECTANGLE_HEIGHT = rospy.get_param(
            "sr_gui_biotac/electrode_display_height", 90)

        # location on the sensor in mm to display location in pixels
        self.x_display_offset = rospy.get_param(
            "sr_gui_biotac/x_display_offset", [100, 12.5, 4.5,
                                               3.5])  # Pixel offsets for
        # displaying electrodes. offset[0] is applied to each electrode.
        # 1,2 and 3 are the label offsets for displaying electrode number.
        self.y_display_offset = rospy.get_param(
            "sr_gui_biotac/y_display_offset", [200, 4.0, 4.0, 4.0])
        self.label_font_size = rospy.get_param(
            "sr_gui_biotac/electrode_label_font_sizes", [24, 22])  # Font sizes
        self.default_topic = "/l_gripper_sensor_controller/raw_data"

    def __init__(self, context):

        super(Pr2GripperSensorGui, self).__init__(context)
        self.setObjectName('Pr2GripperSensorGui')
        self.load_params()

        self._publisher = None
        self._widget = QWidget()

        self.latest_data = PR2GripperSensorRawData()

        self.define_electrodes()
        self._nb_electrodes = self._nb_electrodes_biotac
        self.assign_electrodes(self._nb_electrodes)

        ui_file = os.path.join(rospkg.RosPack().get_path('pr2_gripper_sensor_gui'),
                               'uis', 'Pr2GripperSensorGui.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Pr2GripperSensorUi')

        self.timer = QTimer(self._widget)
        # self._widget.connect(self.timer, SIGNAL("timeout()"),
        #                     self._widget.scrollAreaWidgetContents.update)
        self.timer.timeout.connect(self._widget.scrollAreaWidgetContents.update)
        self._widget.scrollAreaWidgetContents.paintEvent = self.paintEvent

        self.subscribe_to_topic(self.default_topic)

        '''
        self._widget.connect(self._widget.select_prefix,
                             SIGNAL("activated(QString)"),
                             self.subscribe_to_topic)
        '''
        self._widget.select_prefix.activated['QString'].connect(self.subscribe_to_topic)

        self.timer.start(50)

        context.add_widget(self._widget)
