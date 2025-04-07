
#include <memory>
#include <utility>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <chrono>


#include "p6/TfPublisher.hpp"
#include "p6/PIDController.hpp"



using std::placeholders::_1;


namespace p6
{

using namespace std::chrono_literals;

TfPublisher::TfPublisher()
: Node("subscriber_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  last_detection_time_(this->now()),  // Guardamos el tiempo inicial
  state_(NO_PERCIBIDO),
 vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
 {
 RCLCPP_INFO(get_logger(), "Nodo TfPublisher creado");
 detections_sub = create_subscription<vision_msgs::msg::Detection3DArray>(
   "detection_3d", 10,
   std::bind(&TfPublisher::callback, this, _1));
   timer_ = create_wall_timer(
    500ms, std::bind(&TfPublisher::control_cycle, this));
 
 
 tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
 vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

}


void
TfPublisher::control_cycle()
{
  switch (state_) {
    case PERCIBIDO:
      RCLCPP_INFO(get_logger(), "-------------PERCIBIDO---------");
      Result_tf();  
      Camera_tf(); 
         

      if (check_no_detection()) {
        go_state(NO_PERCIBIDO);
      } 
      break;
    case NO_PERCIBIDO:
      RCLCPP_INFO(get_logger(), "-------------NO PERCIBIDO---------");
      break;      
    }
}


void
TfPublisher::callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
 RCLCPP_INFO(get_logger(), "Detection_recibida");
 go_state(PERCIBIDO);


 last_detection_time_ = this->now();
 last_detection_ = *msg;
  v_att = get_atractive_vector(last_detection_);

 RCLCPP_INFO(get_logger(), "Vector atractivo a: [%f, %f, %f]", v_att.x, v_att.y, v_att.z);
 publish_tf();
}


geometry_msgs::msg::Vector3 TfPublisher::get_atractive_vector(const vision_msgs::msg::Detection3DArray& detection) {
 geometry_msgs::msg::Vector3 vector;
  if (!last_detection_.detections.empty()) {
 // Tomamos la posición del primer objeto detectado como vector atractivo
 vector.x = detection.detections[0].bbox.center.position.x;
 vector.y = detection.detections[0].bbox.center.position.y;
 vector.z = detection.detections[0].bbox.center.position.z;
 }
 //RCLCPP_INFO(get_logger(), "Vector atractivo calculado: [%f, %f, %f]", vector.x, vector.y, vector.z);
 return vector;
}

bool TfPublisher::check_no_detection() {
  RCLCPP_INFO(get_logger(), "Entrando en check no detection");
  rclcpp::Time now = this->now();
  rclcpp::Duration time_since_last_detection = now - last_detection_time_;
  RCLCPP_INFO(get_logger(), "tiempo transcurrido: %f", time_since_last_detection.seconds());
  return (time_since_last_detection.seconds() > 1.0);
}
void
TfPublisher::go_state(int new_state)
{
  state_ = new_state;
}


void
TfPublisher::generate_tf()
{
 RCLCPP_INFO(get_logger(), "----------------Creando transformada---------\n");
 transform_.header.stamp = now();
 transform_.header.frame_id = "base_footprint";
 transform_.child_frame_id = "target";


 transform_.transform.translation.x = v_att.z;
 transform_.transform.translation.y = -v_att.x;
 transform_.transform.translation.z = v_att.y;
}

void
TfPublisher::publish_tf()
{
  generate_tf();
  transform_.header.stamp = now();
  tf_broadcaster_->sendTransform(transform_);
  RCLCPP_INFO(get_logger(), "------------Transformada publicada------------------\n");

}

void TfPublisher::Camera_tf()
{
  tf2::Stamped<tf2::Transform> bf2result;
  std::string error;

  while (!tf_buffer_.canTransform("base_footprint", "result", tf2::TimePointZero) && rclcpp::ok()) {
    RCLCPP_WARN(get_logger(), "Esperando por la transformación base_footprint -> result...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto bf2result_msg = tf_buffer_.lookupTransform("base_footprint", "result", tf2::TimePointZero);
  tf2::fromMsg(bf2result_msg, bf2result);

  double x = bf2result.getOrigin().x();
  double y = bf2result.getOrigin().y();

  // Calcular el ángulo con respecto a la orientación del robot
  double angle_to_target = atan2(y, x);
  double dist = sqrt(x * x + y * y);

  RCLCPP_INFO(get_logger(), "Distancia al vector resultante: %f", dist);

  geometry_msgs::msg::Twist twist;
  //double vel_rot = std::clamp(angle_to_target * 0.5, -1.0, 1.0);  // Ganancia ajustable para la rotación
  //double vel_lin = std::clamp(dist * 0.2, -0.5, 0.5);  // Ganancia ajustable para la velocidad lineal

  double vel_rot = std::clamp(vrot_pid_.get_output(angle_to_target), -1.0, 1.0);
  double vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -3.0, 3.0);

  twist.linear.x = vel_lin;
  twist.angular.z = vel_rot;

  RCLCPP_INFO(get_logger(), "Moviéndose hacia el vector resultante.");

  vel_publisher_->publish(twist);
}





void
TfPublisher::Laser_tf()
{
  tf2::Stamped<tf2::Transform> bf2target;
  std::string error;

  if (tf_buffer_.canTransform("base_footprint", "obstacle", tf2::TimePointZero, &error)) {

    auto bf2target_msg = tf_buffer_.lookupTransform(
      "base_footprint", "obstacle", tf2::TimePointZero);

    tf2::fromMsg(bf2target_msg, bf2target);

    double x = bf2target.getOrigin().x();
    double y = bf2target.getOrigin().y();

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

      RCLCPP_INFO(get_logger(), "Laser detectado\n");
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF base_footprint -> obstacle [<< " << error << "]");
  }

}

void TfPublisher::Result_tf()
{
  try {
    auto cam_tf = tf_buffer_.lookupTransform("base_footprint", "target", tf2::TimePointZero);
    auto obs_tf = tf_buffer_.lookupTransform("base_footprint", "obstacle", tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped combined_tf;
    combined_tf.header.stamp = now();
    combined_tf.header.frame_id = "base_footprint";
    combined_tf.child_frame_id = "result";

    combined_tf.transform.translation.x = (cam_tf.transform.translation.x); //+ obs_tf.transform.translation.x) / 2.0;
    combined_tf.transform.translation.y = (cam_tf.transform.translation.y); //+ obs_tf.transform.translation.y) / 2.0;
    combined_tf.transform.translation.z = (cam_tf.transform.translation.z); //+ obs_tf.transform.translation.z) / 2.0;

    // Puedes interpolar orientación con tf2 si quieres. Por ahora, tomamos la de la cámara.
    combined_tf.transform.rotation = cam_tf.transform.rotation;

    tf_broadcaster_->sendTransform(combined_tf);
    RCLCPP_INFO(get_logger(), "Transformada combinada publicada.");
    
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "No se pudo obtener una TF: %s", ex.what());
  }
}


}