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

set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_BIN_DESTINATION bin)

#add_executable(cartographer_node node_main.cc)
#target_include_directories(cartographer_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
#target_link_libraries(cartographer_node ${PROJECT_NAME})
#ament_target_dependencies(cartographer_node "rclcpp" "sensor_msgs" "nav_msgs" "tf2_ros" "cartographer_ros_msgs" "tf2" "tf2_msgs" "console_bridge" "asiframework_msgs" "localization_msgs")


google_binary(cartographer_assets_writer
  SRCS
    assets_writer_main.cc
)

install(TARGETS cartographer_assets_writer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_node
  SRCS
    node_main.cc
)

install(TARGETS cartographer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_offline_node
  SRCS
    offline_node_main.cc
)

install(TARGETS cartographer_offline_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_start_trajectory
  SRCS
    start_trajectory_main.cc
)

install(TARGETS cartographer_start_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

google_binary(cartographer_occupancy_grid_node
  SRCS
    occupancy_grid_node_main.cc
)

install(TARGETS cartographer_occupancy_grid_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
