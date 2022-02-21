cmake_minimum_required(VERSION 3.0)
project(calibrate_modify_contrast)

find_package(catkin
	REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        OpenCV
        image_transport
)
find_package(OpenCV 3 REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp image_transport cv_bridge
    DEPENDS OpenCV
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(contrast_augmenter src/contrast_augmenter.cpp)
target_link_libraries(contrast_augmenter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS contrast_augmenter
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
