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

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -ggdb3")
endif()

find_package(ament_cmake REQUIRED)
find_package(asiframework REQUIRED)
find_package(asiframework_msgs REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)


include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
# Fix for PCL linking issue in Zesty, as in
# https://github.com/ros-perception/perception_pcl/blob/b53b5d5c3abc7c3afde8a77177674e6d74c59522/pcl_ros/CMakeLists.txt#L12
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # FIXME: this causes duplicates and not found error in ubuntu:zesty
  list(REMOVE_ITEM PCL_LIBRARIES "/usr/lib/libmpi.so")
endif()

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  include
  "."
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

set(ALL_SRCS
  "cartographer_ros/asi_node.cc"
  "cartographer_ros/map_builder_bridge.cc"
  "cartographer_ros/msg_conversion.cc"
  "cartographer_ros/node.cc"
  "cartographer_ros/node_constants.cc"
  "cartographer_ros/node_options.cc"
  "cartographer_ros/ros_log_sink.cc"
  "cartographer_ros/sensor_bridge.cc"
  "cartographer_ros/submap.cc"
  "cartographer_ros/tf_bridge.cc"
  "cartographer_ros/time_conversion.cc"
  "cartographer_ros/trajectory_options.cc"
  "cartographer_ros/urdf_reader.cc"
)
add_library(${PROJECT_NAME} ${ALL_SRCS})

# Cartographer
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

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
