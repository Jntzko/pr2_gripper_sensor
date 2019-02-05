cmake_minimum_required(VERSION 2.8.3)
project(pr2_gripper_sensor_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
  pr2_gripper_sensor_msgs
)

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs rospy rqt_gui rqt_gui_py pr2_gripper_sensor_msgs

)

install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
install( FILES pr2_gripper_sensor_gui_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
