# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ueye2)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure nodelet roscpp image_transport camera_calibration_parsers)
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_find_ros_package(dynamic_reconfigure)

# CATKIN_MIGRATION: removed during catkin migration
# include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
generate_dynamic_reconfigure_options(
  cfg/UEye.cfg
)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES 
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS dynamic_reconfigure nodelet roscpp image_transport camera_calibration_parsers
    INCLUDE_DIRS include# TODO include
    LIBRARIES # TODO
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
add_executable(${PROJECT_NAME} src/node.cpp src/UEyeCamera.cpp src/UEyeNode.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ueye_api)

add_library(${PROJECT_NAME}_nodelet src/nodelet.cpp src/UEyeCamera.cpp src/UEyeNode.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_nodelet ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_nodelet ueye_api)
