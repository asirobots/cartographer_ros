# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros)

<<<<<<< HEAD
find_package(ament_cmake REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -ggdb3")
endif()
=======
set(PACKAGE_DEPENDENCIES
  cartographer_ros_msgs
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_cartographer_project()
google_enable_testing()
>>>>>>> master

find_package(rclcpp REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(asiframework_msgs REQUIRED)
find_package(localization_msgs REQUIRED)

include(FindPkgConfig)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()

find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
# Fix for PCL linking issue in Zesty, as in
# https://github.com/ros-perception/perception_pcl/blob/b53b5d5c3abc7c3afde8a77177674e6d74c59522/pcl_ros/CMakeLists.txt#L12
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # FIXME: this causes duplicates and not found error in ubuntu:zesty
  list(REMOVE_ITEM PCL_LIBRARIES "/usr/lib/libmpi.so")
endif()
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(console_bridge REQUIRED)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
<<<<<<< HEAD
  include
  "."
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
=======
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    # TODO(damonkohler): This should be here but causes Catkin to abort because
    # protobuf specifies a library '-lpthread' instead of just 'pthread'.
    # CARTOGRAPHER
    PCL
    EIGEN3
    Boost
    urdfdom_headers
  INCLUDE_DIRS "."
  LIBRARIES ${PROJECT_NAME}
>>>>>>> master
)

set(ALL_SRCS
  "cartographer_ros/assets_writer.cc"
  "cartographer_ros/map_builder_bridge.cc"
  "cartographer_ros/map_writer.cc"
  "cartographer_ros/msg_conversion.cc"
  "cartographer_ros/occupancy_grid.cc"
  "cartographer_ros/node.cc"
  "cartographer_ros/node_options.cc"
  "cartographer_ros/ros_log_sink.cc"
  "cartographer_ros/sensor_bridge.cc"
  "cartographer_ros/tf_bridge.cc"
  "cartographer_ros/time_conversion.cc"
  "cartographer_ros/trajectory_options.cc"
  )
add_library(${PROJECT_NAME} ${ALL_SRCS})

# Cartographer
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

<<<<<<< HEAD
# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")

=======
>>>>>>> master
# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# YAML
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAMLCPP_LIBRARIES})

# URDFDOM
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

add_subdirectory(cartographer_ros)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY configuration_files
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/
)

if(AMENT_ENABLE_TESTING OR BUILD_TESTING)
  google_enable_testing()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # not sure how to put timeout back on these:
  google_test(configuration_files_test "cartographer_ros/configuration_files_test.cc" )
  google_test(time_conversion_test "cartographer_ros/time_conversion_test.cc")
endif()


ament_package()
