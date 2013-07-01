cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(camera src/camera_node.cpp src/CameraNode.cpp src/Camera.cpp)
target_link_libraries(camera ueye_api )

rosbuild_add_executable(stereo src/stereo_node.cpp src/StereoNode.cpp src/CameraNode.cpp src/Camera.cpp)
target_link_libraries(stereo ueye_api )

rosbuild_add_executable(framerate src/framerate_node.cpp src/FramerateNode.cpp)

rosbuild_add_library(ueye_nodelets
		src/camera_nodelet.cpp src/CameraNode.cpp src/Camera.cpp
		src/stereo_nodelet.cpp src/StereoNode.cpp
		src/framerate_nodelet.cpp src/FramerateNode.cpp)
target_link_libraries(ueye_nodelets ueye_api )
