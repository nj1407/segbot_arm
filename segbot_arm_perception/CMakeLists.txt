cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_perception)

find_package(catkin REQUIRED
	pcl_ros
	pcl_conversions
	sensor_msgs
	geometry_msgs
	std_msgs
	std_srvs
	roscpp
	rospy
 	message_generation
 	actionlib_msgs
)

add_service_files(
   FILES
   PlanarSegmentation.srv
   TableDetectionObjectExtraction.srv
   FeatureExtraction.srv
   TabletopPerception.srv
   ButtonDetection.srv
   TabletopReorder.srv
   SetObstacles.srv
   GetCloud.srv
)

<<<<<<< HEAD
generate_messages(
=======
# add_action_files(
#   DIRECTORY action
#   FILES LogPerception.action
#)

 generate_messages(
>>>>>>> e2a5e20ae9970b5ba4061d3c3e49fec332b51de5
   DEPENDENCIES
   std_msgs
   std_srvs
   sensor_msgs
   geometry_msgs
   actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS actionlib_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_executable(obstacle_cloud_manager src/obstacle_cloud_manager.cpp)
add_dependencies(obstacle_cloud_manager segbot_arm_perception_gencpp)
target_link_libraries(obstacle_cloud_manager ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(table_object_detection_node src/table_object_detection_node.cpp)
add_dependencies(table_object_detection_node segbot_arm_perception_gencpp)
target_link_libraries(table_object_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(table_change_detection_node src/table_change_detection_node.cpp)
add_dependencies(table_change_detection_node segbot_arm_perception_gencpp)
target_link_libraries(table_change_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_feature_server src/pointcloud_feature_server.cpp)
add_dependencies(pointcloud_feature_server segbot_arm_perception_gencpp)
target_link_libraries(pointcloud_feature_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(button_detection_srv_node src/button_detection_srv_node.cpp)
target_link_libraries(button_detection_srv_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(button_detection_srv_node segbot_arm_perception_gencpp)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
