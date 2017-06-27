cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(asr_resources_for_psm)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(catkin REQUIRED COMPONENTS
  asr_msgs
  asr_relation_graph_generator
)

find_package(cmake_modules REQUIRED)

#find Eigen (depends on ros distro):
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES}) 
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES resources_for_psm_lib
  CATKIN_DEPENDS asr_msgs asr_relation_graph_generator
  DEPENDS EIGEN3
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
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)
