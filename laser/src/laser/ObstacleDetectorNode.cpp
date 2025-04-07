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

#include <memory>

#include "laser/ObstacleDetectorNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

namespace laser
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "ObstacleDetectorNode set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetectorNode::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle", 100);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  // Establece la distancia mínima de detección
  float min_distance = std::numeric_limits<float>::infinity();
  size_t min_index = 0;

  // Parte trasera: primeros 1/8 (~45° desde -π)
  for (size_t i = 0; i < scan->ranges.size() / 8; ++i) {
    if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < min_distance) {
      min_distance = scan->ranges[i];
      min_index = i;
      RCLCPP_INFO(get_logger(), "Parte trasera (izquierda) [%zu]: %f", i, scan->ranges[i]);
    }
  }

  // Parte trasera: últimos 1/8 (~45° desde +π)
  for (size_t i = 7 * scan->ranges.size() / 8; i < scan->ranges.size(); ++i) {
    if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < min_distance) {
      min_distance = scan->ranges[i];
      min_index = i;
      RCLCPP_INFO(get_logger(), "Parte trasera (derecha) [%zu]: %f", i, scan->ranges[i]);
    }
  }

  // Imprime la distancia mínima encontrada
  RCLCPP_INFO(get_logger(), "Distancia mínima detectada: %f", min_distance);

  // Publica el mensaje de detección de obstáculo si la distancia mínima es menor que el umbral
  auto obstacle_msg = std_msgs::msg::Bool();
  obstacle_msg.data = (min_distance < min_distance_);  // Usa el parámetro de ROS

  if (obstacle_msg.data) {
    RCLCPP_INFO(get_logger(), "¡Obstáculo detectado! Distancia: %f", min_distance);
    float angle = scan->angle_min + min_index * scan->angle_increment;
    v_att.x = min_distance * cos(angle);
    v_att.y = min_distance * sin(angle);
    v_att.z = 0.0;

    publish_tf();
  }

  obstacle_pub_->publish(obstacle_msg);
}

void
ObstacleDetectorNode::generate_tf()
{
  RCLCPP_INFO(get_logger(), "----------------Creando tf laser---------\n");
  transform_.header.frame_id = "base_footprint";
  transform_.child_frame_id = "obstacle";

  transform_.transform.translation.x = v_att.x;
  transform_.transform.translation.y = v_att.y;
  transform_.transform.translation.z = v_att.z;
}

void
ObstacleDetectorNode::publish_tf()
{
  generate_tf();
  transform_.header.stamp = now();
  tf_broadcaster_->sendTransform(transform_);
  RCLCPP_INFO(get_logger(), "------------Tf laser publicada------------------\n");
}

}  // namespace laser
