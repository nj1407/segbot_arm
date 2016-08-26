cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(door_manipulation_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
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

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   door_perception.srv
)


## Generate actions in the 'action' folder
add_action_files(
  FILES
  PushDoor.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mimic_motion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib

)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

## get links and definitons for pcl
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(door_handle_detection src/door_handle_detection.cpp)
add_executable(door_open src/door_open.cpp)
add_executable(door_open_as src/door_open_as.cpp)
add_executable(door_open_client src/door_open_client.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(door_handle_detection door_demo_manipulation_gencpp)
add_dependencies(door_open door_demo_manipulation_gencpp)
add_dependencies(door_open_as jaco_msgs_gencpp segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(door_open_client jaco_msgs_gencpp segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(door_handle_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(door_open ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(door_open_as ${catkin_LIBRARIES})
target_link_libraries(door_open_client ${catkin_LIBRARIES})

