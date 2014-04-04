# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(object_finder)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp message_filters actionlib message_runtime std_msgs message_generation geometry_msgs uf_common visualization_msgs tf actionlib_msgs)
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
set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_find_ros_package(actionlib_msgs)

# CATKIN_MIGRATION: removed during catkin migration
# include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
add_action_files(
  FILES
  Find.action
)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory

# CATKIN_MIGRATION: removed during catkin migration
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  FILES
  TargetDesc.msg
  TargetRes.msg
  Mesh.msg
  Component.msg
  Triangle.msg
  Color.msg
)
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES geometry_msgs actionlib_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS Eigen # TODO
    CATKIN_DEPENDS roscpp message_filters actionlib message_runtime std_msgs message_generation geometry_msgs uf_common visualization_msgs tf actionlib_msgs
    INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} include# TODO include
    LIBRARIES # TODO
)

include_directories(${EIGEN_INCLUDE_DIRS} include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
add_executable(object_finder src/node.cpp src/obj_finding.cpp src/sphere_finding.cpp src/fft.cpp)
target_link_libraries(object_finder ${catkin_LIBRARIES})
add_dependencies(object_finder ${catkin_EXPORTED_TARGETS})
add_dependencies(object_finder ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(object_finder fftw3)
set_target_properties(object_finder PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall -Wno-unused-local-typedefs")

add_executable(test_obj_finding src/test_obj_finding.cpp src/obj_finding.cpp src/sphere_finding.cpp)
target_link_libraries(test_obj_finding ${catkin_LIBRARIES})
add_dependencies(test_obj_finding ${catkin_EXPORTED_TARGETS})
add_dependencies(test_obj_finding ${PROJECT_NAME}_generate_messages_cpp)
set_target_properties(test_obj_finding PROPERTIES COMPILE_FLAGS -std=c++11)

add_executable(test_fft src/test_fft.cpp src/fft.cpp)
target_link_libraries(test_fft fftw3)
set_target_properties(test_fft PROPERTIES COMPILE_FLAGS -std=c++11)

find_package(Boost REQUIRED COMPONENTS signals)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#target_link_libraries(example ${PROJECT_NAME})
