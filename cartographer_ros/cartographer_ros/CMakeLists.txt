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

set(PACKAGE_LIB_DESTINATION lib)
set(PACKAGE_BIN_DESTINATION bin)

set(PACKAGE_DEPS
        "asiframework_msgs"
        "cartographer"
        "cartographer_ros_msgs"
        "Eigen3"
        "geometry_msgs"
        "gMock"
        "gLog"
        "localization_msgs"
        "LUA"
        "nav_msgs"
        "PCL"
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "tf2"
        "tf2_msgs"
        "tf2_ros"
        "urdf"
        "visualization_msgs"
        "YAMLCPP"
        )


#google_binary(cartographer_assets_writer
#  SRCS
#    assets_writer_main.cc
#)

#install(TARGETS cartographer_assets_writer
#  ARCHIVE DESTINATION ${PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${PACKAGE_BIN_DESTINATION}
#)

add_executable(cartographer_node node_main.cc)
target_include_directories(cartographer_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_node ${PACKAGE_DEPS})

install(TARGETS cartographer_node
  ARCHIVE DESTINATION ${PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${PACKAGE_BIN_DESTINATION}
)

#google_binary(cartographer_offline_node
#  SRCS
#    offline_node_main.cc
#)

#install(TARGETS cartographer_offline_node
#  ARCHIVE DESTINATION ${PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${PACKAGE_BIN_DESTINATION}
#)

add_executable(cartographer_start_trajectory start_trajectory_main.cc)
target_include_directories(cartographer_start_trajectory SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_start_trajectory ${PROJECT_NAME})
ament_target_dependencies(cartographer_start_trajectory ${PACKAGE_DEPS})

install(TARGETS cartographer_start_trajectory
  ARCHIVE DESTINATION ${PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${PACKAGE_BIN_DESTINATION}
)

add_executable(cartographer_occupancy_grid_node occupancy_grid_node_main.cc)
target_include_directories(cartographer_occupancy_grid_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_occupancy_grid_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_occupancy_grid_node ${PACKAGE_DEPS})

install(TARGETS cartographer_occupancy_grid_node
  ARCHIVE DESTINATION ${PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${PACKAGE_BIN_DESTINATION}
)
