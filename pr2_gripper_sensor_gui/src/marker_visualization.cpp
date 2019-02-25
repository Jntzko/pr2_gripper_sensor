#include <ros/ros.h>

#include <pr2_gripper_sensor_msgs/PR2GripperSensorRawData.h>
#include <visualization_msgs/MarkerArray.h>

visualization_msgs::MarkerArray marker_array_;
ros::Publisher marker_publisher_;


// Position and scale of the taxels in mm
const float taxel_pos_x[22] = {27, 8.25, 21.5, 31, 31, 8.25, 21.5, 25, 25, 25, 19, 19, 19, 13, 13, 13, 7, 7, 7, 1, 1, 1};
const float taxel_pos_y[22] = {-5, -10.5, -10.5, -10.5, -10.5, -10.5, -10.5, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8, -15.8};
const float taxel_pos_z[22] = {0, 11.5, 11.5, 3, -3, -11.5, -11.5, 6, 0, -6, 6, 0, -6, 6, 0, -6, 6, 0, -6, 6, 0, -6};

const float taxel_scale_x[22] = {3, 20, 5.5, 0.5, 0.5, 20, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5};
const float taxel_scale_y[22] = {0.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
const float taxel_scale_z[22] = {13, 0.5, 0.5, 5.5, 5.5, 0.5, 0.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5};

void init() {
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.pose.orientation.w = 1.0;
  marker.action = visualization_msgs::Marker::ADD;

  /* Left side */
  marker.header.frame_id = "l_gripper_l_finger_tip_link"; 

  for (int i=0; i<22;i++) {
    marker.id = i;
    marker.pose.position.x = taxel_pos_x[i] / 1000;
    marker.pose.position.y = taxel_pos_y[i] / 1000;
    marker.pose.position.z = taxel_pos_z[i] / 1000;
    marker.scale.x = taxel_scale_x[i] / 1000;
    marker.scale.y = taxel_scale_y[i] / 1000;
    marker.scale.z = taxel_scale_z[i] / 1000;

    marker_array_.markers.push_back(marker);
  }

  /* Right side */

  marker.header.frame_id = "l_gripper_r_finger_tip_link"; 

  for (int i=0; i<22; i++) {
    marker.id = i + 22;
    marker.pose.position.x = taxel_pos_x[i] / 1000;
    marker.pose.position.y = taxel_pos_y[i] / -1000;
    marker.pose.position.z = taxel_pos_z[i] / 1000;
    marker.scale.x = taxel_scale_x[i] / 1000;
    marker.scale.y = taxel_scale_y[i] / 1000;
    marker.scale.z = taxel_scale_z[i] / 1000;

    marker_array_.markers.push_back(marker);
  }
}

void callback(const pr2_gripper_sensor_msgs::PR2GripperSensorRawData::ConstPtr &msg) {
  float r, g, b;
  float threshold_1 = 2000.0;
  float threshold_2 = 8000.0;
  float value;

  for (int i=0; i<22; i++) {
    if (i<22)
      value = msg->left_finger_pad_forces[i];
    else
      value = msg->right_finger_pad_forces[i-22];

    if (value <= 0) {
      r = g = b = 1;
    } else if (value < threshold_1) {
      r = 1;
      g = 1;
      b = 1 - 1 * (value / threshold_1);
    } else {
      r = 1;
      g = 1 * ((threshold_2 - value) / (threshold_2 - threshold_1));
      b = 0;
    } 
    marker_array_.markers[i].color.r = r;
    marker_array_.markers[i].color.g = g;
    marker_array_.markers[i].color.b = b;
  }

  marker_publisher_.publish(marker_array_);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "PR2GripperSensorVisualization");
  ros::NodeHandle node_handle;

  marker_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>( "l_gripper_sensor_marker", 1);
  ros::Subscriber subscriber = node_handle.subscribe("/l_gripper_sensor_controller/raw_data", 1, callback);

  init();

  ros::spin();
  return 0;
}
