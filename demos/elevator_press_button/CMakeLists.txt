cmake_minimum_required(VERSION 2.8.3)
project(elevator_press_button)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  genmsg
  cv_bridge
  image_transport
  message_generation
  actionlib
  pcl_ros
  pcl_conversions
  tf_conversions
  moveit_utils
  moveit_ros_planning_interface
  moveit_msgs
  segbot_arm_manipulation
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 2 REQUIRED)

 add_service_files(
   FILES
   color_perception.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elevator_press_button
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## get links and definitons for pcl
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_panel_filter src/pcl_panel_filter.cpp)
add_executable(press_button src/press_button.cpp)
add_executable(see_button_opencv src/see_button_opencv.cpp)

add_dependencies(pcl_panel_filter elevator_press_button_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(press_button elevator_press_button_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(see_button_opencv elevator_press_button_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS${OpenCV_LIBRARIES}})

target_link_libraries(pcl_panel_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(press_button ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(see_button_opencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
