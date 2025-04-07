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

#ifndef p6__TFPUBLISHER_HPP_
#define p6__TFPUBLISHER_HPP_

#include <memory>
#include <vector>

#include "p6/PIDController.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"  
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"



namespace p6
{

class TfPublisher : public rclcpp::Node
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TfPublisher)

  TfPublisher();
  void callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

private:
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_detection_time_;
  rclcpp::TimerBase::SharedPtr timer_publish_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  static const int PERCIBIDO = 0;
  static const int NO_PERCIBIDO = 1;
  int state_;

  void control_cycle();
  void go_state(int new_state);

  vision_msgs::msg::Detection3DArray last_detection_;
  geometry_msgs::msg::Vector3 v_att;   
  geometry_msgs::msg::Vector3 get_atractive_vector(const vision_msgs::msg::Detection3DArray& detection);

  geometry_msgs::msg::TransformStamped transform_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  

  void generate_tf();
  void publish_tf();
  void Camera_tf();
  void Laser_tf();
  void Result_tf();
  bool check_no_detection();

  PIDController vlin_pid_, vrot_pid_;



};

}  //  namespace p6

#endif  // p6__TFPUBLISHER_HPP_
