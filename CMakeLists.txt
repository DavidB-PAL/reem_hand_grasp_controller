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

# Hand Posture Controller
rosbuild_add_executable(reem_hand_grasp_controller src/reem_hand_grasp_controller.cpp)

# grasp/release tests
rosbuild_add_executable(test/grasp_left   src/test/grasp_left.cpp)
rosbuild_add_executable(test/release_left src/test/release_left.cpp)
rosbuild_add_executable(test/grasp_right   src/test/grasp_right.cpp)
rosbuild_add_executable(test/release_right src/test/release_right.cpp)

