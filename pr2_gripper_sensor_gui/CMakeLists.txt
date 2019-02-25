cmake_minimum_required(VERSION 2.8.3)
project(pr2_gripper_sensor_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
  pr2_gripper_sensor_msgs
  visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs rospy rqt_gui rqt_gui_py pr2_gripper_sensor_msgs

)

add_executable(marker_visualization src/marker_visualization.cpp)
add_dependencies(marker_visualization ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_visualization ${catkin_LIBRARIES})


install( DIRECTORY uis DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
install( FILES pr2_gripper_sensor_gui_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
