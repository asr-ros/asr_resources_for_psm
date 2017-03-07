cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(asr_resources_for_psm)

find_package(catkin REQUIRED COMPONENTS
  asr_msgs
  asr_relation_graph_generator
)

find_package(cmake_modules REQUIRED)

# include Eigen library
find_package(Eigen REQUIRED)
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES resources_for_psm_lib
  CATKIN_DEPENDS asr_msgs asr_relation_graph_generator
  DEPENDS Eigen
)

############################################################
# ROS Nodes that are using probabilistic scene recognition.
############################################################

# Build a library containing the scene model.
add_library(resources_for_psm_lib
			src/Pose.cpp)

# dependency to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this package
add_dependencies(resources_for_psm_lib asr_msgs_generate_messages_cpp)

target_link_libraries(resources_for_psm_lib
  ${Eigen_LIBRARIES}
  ${catkin_LIBRARIES}
)
