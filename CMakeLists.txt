cmake_minimum_required( VERSION 3.10 )

project( openpose_ros )

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(OpenPose_DIR "/usr/local/lib/OpenPose")
set(openpose_ros_VERSION 1.0)

include(GenerateExportHeader)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	sensor_msgs
	image_transport
	cv_bridge
)

find_package( OpenPose REQUIRED )
find_package( gflags REQUIRED )
find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)
include_directories(
    ${OpenPose_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIR}
)

add_library(openpose_ros SHARED src/openpose_ros.cpp )
add_executable(openpose_ros_demo src/openpose_ros_demo.cpp )

target_link_libraries( openpose_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenPose_LIBS} stdc++fs gflags )
target_link_libraries( openpose_ros_demo ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenPose_LIBS} openpose_ros gflags)

install(TARGETS openpose_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


add_compile_options(${GTK3_CFLAGS_OTHER})
