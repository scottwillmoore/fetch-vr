cmake_minimum_required(VERSION 3.0)
project(fetch_vr)

set(
	MESSAGE_DEPENDENCIES
		std_msgs
)

find_package(catkin
	REQUIRED COMPONENTS
		${MESSAGE_DEPENDENCIES}
		message_generation
		rospy
		roscpp
		sensor_msgs
		pcl_ros
		message_filters
)

#add_message_files(
#	FILES
#		ExampleMessage.msg
#)

#add_service_files(
#	FILES
#		ExampleService.srv
#)

#add_action_files(
#	FILES
#		Example.action
#)

#generate_messages(
#	DEPENDENCIES
#		${MESSAGE_DEPENDENCIES}
#)

catkin_package(
	CATKIN_DEPENDS
		${MESSAGE_DEPENDENCIES}
		message_runtime
		rospy
)

catkin_install_python(
	PROGRAMS
		scripts/marker_transform_publisher.py
		scripts/planning_scene.py
		scripts/setup_planning_scene.py
		scripts/segment_point_cloud_publisher.py
		scripts/transform_point_cloud_publisher.py
		scripts/camera_transform_publisher.py
		scripts/assemble_clouds.py
	DESTINATION
		${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	FILES
		launch/fetch_vr.launch
		launch/external_camera_publisher.launch
	DESTINATION
		${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS} ${PCL_LIBRARIES})

add_executable(concatenate_clouds src/concatenate_clouds.cpp)
target_link_libraries(concatenate_clouds ${catkin_LIBRARIES} ${PCL_LIBRARIES})
