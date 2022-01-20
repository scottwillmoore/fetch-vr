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
)

add_message_files(
	FILES
		ExampleMessage.msg
)

add_service_files(
	FILES
		ExampleService.srv
)

#add_action_files(
#	FILES
#		Example.action
#)

generate_messages(
	DEPENDENCIES
		${MESSAGE_DEPENDENCIES}
)

catkin_package(
	CATKIN_DEPENDS
		${MESSAGE_DEPENDENCIES}
		message_runtime
		rospy
)

catkin_install_python(
	PROGRAMS
		scripts/example.py
		scripts/segmentation.py
		scripts/lib_cloud_conversion_between_Open3D_and_ROS.py
	DESTINATION
		${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	FILES
		launch/test.launch
	DESTINATION
		${CATKIN_PACKAGE_SHARE_DESTINATION}
)
