// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef LASER__OBSTACLEDETECTORNODE_HPP_
#define LASER__OBSTACLEDETECTORNODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/buffer_core.h"

namespace laser
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  void generate_tf();
  void publish_tf();

  geometry_msgs::msg::TransformStamped transform_;
  geometry_msgs::msg::Vector3 v_att;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  float min_distance_ {1.5f};
};

}  // namespace laser

#endif  // LASER__OBSTACLEDETECTORNODE_HPP_
