/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <string>
#include <vector>
#include <thread>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");

DEFINE_bool(durable, false, "Enable the durability setting 'transient local' for the publisher.");

namespace cartographer_ros {
namespace {

using ::cartographer::mapping::SubmapId;

constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

struct SubmapState {
  SubmapState()
      : surface(::cartographer::io::MakeUniqueCairoSurfacePtr(nullptr)) {}

  // Texture data.
  int width;
  int height;
  int version;
  double resolution;
  ::cartographer::transform::Rigid3d slice_pose;
  ::cartographer::io::UniqueCairoSurfacePtr surface;
  // Pixel data used by 'surface'. Must outlive 'surface'.
  std::vector<uint32_t> cairo_data;

  // Metadata.
  ::cartographer::transform::Rigid3d pose;
  int metadata_version = -1;
};

void CairoDrawEachSubmap(
    const double scale, std::map<SubmapId, SubmapState>* submaps, cairo_t* cr,
    std::function<void(const SubmapState&)> draw_callback) {
  cairo_scale(cr, scale, scale);

  for (auto& pair : *submaps) {
    auto& submap_state = pair.second;
    if (submap_state.surface == nullptr) {
      return;
    }
    const Eigen::Matrix4d homo =
        ToEigen(submap_state.pose * submap_state.slice_pose).matrix();

    cairo_save(cr);
    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_state.resolution;
    cairo_scale(cr, submap_resolution, submap_resolution);
    draw_callback(submap_state);
    cairo_restore(cr);
  }
}

class Node {
 public:
  explicit Node(double resolution, ::rclcpp::Node::SharedPtr node_handle);
  ~Node() { wants_exit_ = true; condition_.notify_all(); background_worker_.join(); }

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(cartographer_ros_msgs::msg::SubmapList::SharedPtr msg);
  void DrawAndPublish(const string& frame_id, builtin_interfaces::msg::Time time);
  void PublishOccupancyGrid(const string& frame_id, builtin_interfaces::msg::Time time,
                            const Eigen::Array2f& origin,
                            const Eigen::Array2i& size,
                            cairo_surface_t* surface);
  void HandleSubmapListWorker();

  const double resolution_;
  ::rclcpp::Node::SharedPtr node_handle_;
  bool wants_exit_ = false, has_new_msg_ = false;

  std::mutex mutex_;
  std::condition_variable condition_;
  rclcpp::Client<::cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_;
  rclcpp::SubscriptionBase::SharedPtr submap_list_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  std::map<SubmapId, SubmapState> submaps_;
  cartographer_ros_msgs::msg::SubmapList::SharedPtr last_msg_ = nullptr;
  std::thread background_worker_;
};

Node::Node(const double resolution, ::rclcpp::Node::SharedPtr node_handle)
    : resolution_(resolution), node_handle_(node_handle) {
  client_ = node_handle_->create_client<::cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName);

  auto qos = rmw_qos_profile_default;
  qos.depth = kLatestOnlyPublisherQueueSize;
  submap_list_subscriber_ = node_handle_->create_subscription<cartographer_ros_msgs::msg::SubmapList>(
      kSubmapListTopic,
      [this](cartographer_ros_msgs::msg::SubmapList::SharedPtr msg) {
        HandleSubmapList(msg);
      }, qos);

  if (FLAGS_durable)
    qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  occupancy_grid_publisher_ = node_handle_->create_publisher<::nav_msgs::msg::OccupancyGrid>(
      kOccupancyGridTopic, qos);

  background_worker_ = std::thread([this]() {
    HandleSubmapListWorker();
  });
}

void Node::HandleSubmapListWorker() {
  while (!wants_exit_) {
    cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg;
    {
      std::unique_lock<std::mutex> locker(mutex_);
      condition_.wait(locker, [this]{ return wants_exit_ || has_new_msg_; });
      if (wants_exit_) return;
      has_new_msg_ = false;
      msg = last_msg_;
    }
    // We do not do any work if nobody listens.
    if (msg == nullptr || node_handle_->count_subscribers(occupancy_grid_publisher_->get_topic_name()) == 0) {
      continue;
    }
    for (const auto& submap_msg : msg->submap) {
      const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
      SubmapState& submap_state = submaps_[id];
      submap_state.pose = ToRigid3d(submap_msg.pose);
      submap_state.metadata_version = submap_msg.submap_version;
      if (submap_state.surface != nullptr &&
          submap_state.version == submap_msg.submap_version) {
        continue;
      }

      auto fetched_textures = ::cartographer_ros::FetchSubmapTextures(id, client_);
      if (fetched_textures == nullptr) {
        continue;
      }

      CHECK(!fetched_textures->textures.empty());
      submap_state.version = fetched_textures->version;

      // TODO(gaschler): Handle more textures than just the first one.
      const auto fetched_texture = fetched_textures->textures.begin();
      submap_state.width = fetched_texture->width;
      submap_state.height = fetched_texture->height;
      submap_state.slice_pose = fetched_texture->slice_pose;
      submap_state.resolution = fetched_texture->resolution;

      // Properly dealing with a non-common stride would make this code much more
      // complicated. Let's check that it is not needed.
      const int expected_stride = 4 * submap_state.width;
      CHECK_EQ(expected_stride,
               cairo_format_stride_for_width(kCairoFormat, submap_state.width));
      submap_state.cairo_data.clear();
      for (size_t i = 0; i < fetched_texture->intensity.size(); ++i) {
        // We use the red channel to track intensity information. The green
        // channel we use to track if a cell was ever observed.
        const uint8_t intensity = fetched_texture->intensity.at(i);
        const uint8_t alpha = fetched_texture->alpha.at(i);
        const uint8_t observed = (intensity == 0 && alpha == 0) ? 0 : 255;
        submap_state.cairo_data.push_back((alpha << 24) | (intensity << 16) |
                                          (observed << 8) | 0);
      }

      submap_state.surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
          cairo_image_surface_create_for_data(
              reinterpret_cast<unsigned char*>(submap_state.cairo_data.data()),
              kCairoFormat, submap_state.width, submap_state.height,
              expected_stride));
      CHECK_EQ(cairo_surface_status(submap_state.surface.get()),
               CAIRO_STATUS_SUCCESS)
        << cairo_status_to_string(
            cairo_surface_status(submap_state.surface.get()));
    }
    DrawAndPublish(msg->header.frame_id, msg->header.stamp);
  }
}

void Node::HandleSubmapList(
    cartographer_ros_msgs::msg::SubmapList::SharedPtr msg) {
  {
    std::unique_lock<std::mutex> locker(mutex_);
    last_msg_ = msg;
    has_new_msg_ = true;
  }
  // we can't call FetchSubmapTexture on this thread
  condition_.notify_one();
}

void Node::DrawAndPublish(const string& frame_id, builtin_interfaces::msg::Time time) {
  if (submaps_.empty()) {
    return;
  }

  Eigen::AlignedBox2f bounding_box;
  {
    auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(kCairoFormat, 1, 1));
    auto cr =
        ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    CairoDrawEachSubmap(
        1. / resolution_, &submaps_, cr.get(),
        [&update_bounding_box, &bounding_box](const SubmapState& submap_state) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_state.width, 0);
          update_bounding_box(0, submap_state.height);
          update_bounding_box(submap_state.width, submap_state.height);
        });
  }

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  {
    auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(kCairoFormat, size.x(), size.y()));
    auto cr =
        ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    cairo_paint(cr.get());
    cairo_translate(cr.get(), origin.x(), origin.y());
    CairoDrawEachSubmap(1. / resolution_, &submaps_, cr.get(),
                        [&cr](const SubmapState& submap_state) {
                          cairo_set_source_surface(
                              cr.get(), submap_state.surface.get(), 0., 0.);
                          cairo_paint(cr.get());
                        });
    cairo_surface_flush(surface.get());
    PublishOccupancyGrid(frame_id, time, origin, size, surface.get());
  }
}

void Node::PublishOccupancyGrid(const string& frame_id, builtin_interfaces::msg::Time time,
                                const Eigen::Array2f& origin,
                                const Eigen::Array2i& size,
                                cairo_surface_t* surface) {
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = time;
  occupancy_grid.header.frame_id = frame_id;
  occupancy_grid.info.map_load_time = time;
  occupancy_grid.info.resolution = resolution_;
  occupancy_grid.info.width = size.x();
  occupancy_grid.info.height = size.y();
  occupancy_grid.info.origin.position.x = -origin.x() * resolution_;
  occupancy_grid.info.origin.position.y =
      (-size.y() + origin.y()) * resolution_;
  occupancy_grid.info.origin.position.z = 0.;
  occupancy_grid.info.origin.orientation.w = 1.;
  occupancy_grid.info.origin.orientation.x = 0.;
  occupancy_grid.info.origin.orientation.y = 0.;
  occupancy_grid.info.origin.orientation.z = 0.;

  const uint32* pixel_data =
      reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface));
  occupancy_grid.data.reserve(size.x() * size.y());
  for (int y = size.y() - 1; y >= 0; --y) {
    for (int x = 0; x < size.x(); ++x) {
      const uint32_t packed = pixel_data[y * size.x() + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int8_t value =
          observed == 0
              ? int8_t(-1)
              : int8_t(::cartographer::common::RoundToInt((1. - color / 255.) * 100.));
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid.data.push_back(value);
    }
  }
  occupancy_grid_publisher_->publish(occupancy_grid);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("occupancy_grid_node");
  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node wrapper(FLAGS_resolution, node);

  ::rclcpp::spin(node);
  ::rclcpp::shutdown();
}
