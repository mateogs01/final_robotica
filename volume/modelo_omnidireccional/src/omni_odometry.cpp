#include "omni_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace robmovil;

#define WHEEL_BASELINE 0.175
#define WHEEL_RADIUS 0.050
#define ENCODER_TICKS 500.0

OmniOdometry::OmniOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", rclcpp::QoS(10), std::bind(&OmniOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_front_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_front_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&OmniOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OmniOdometry::on_velocity_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr twist)
{
  double linearVel = twist->twist.linear.x;
  double transvVel = twist->twist.linear.y;
  double angularVel = twist->twist.angular.z;
     
  double vLeftFront =  1 / WHEEL_RADIUS * (linearVel - transvVel - angularVel * (WHEEL_BASELINE*2));
  double vRightFront = 1 / WHEEL_RADIUS * (linearVel + transvVel + angularVel * (WHEEL_BASELINE*2));
  double vLeftRear =   1 / WHEEL_RADIUS * (linearVel + transvVel - angularVel * (WHEEL_BASELINE*2));
  double vRightRear =  1 / WHEEL_RADIUS * (linearVel - transvVel + angularVel * (WHEEL_BASELINE*2));

  {
    std_msgs::msg::Float64 msg;
    msg.data = vLeftFront;

    vel_pub_front_left_->publish(msg);
  }

  {
    std_msgs::msg::Float64 msg;
    msg.data = vRightFront;

    vel_pub_front_right_->publish(msg);
  }

  {
    std_msgs::msg::Float64 msg;
    msg.data = vLeftRear;

    vel_pub_rear_left_->publish(msg);
  }

  {
    std_msgs::msg::Float64 msg;
    msg.data = vRightRear;

    vel_pub_rear_right_->publish(msg);
  }
 
  //RCLCPP_INFO(this->get_logger(), "Path: %u, %u", it->i, it->j);
}

void OmniOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_1 = encoder->ticks[0];
    last_ticks_2 = encoder->ticks[1];
    last_ticks_3 = encoder->ticks[2];
    last_ticks_4 = encoder->ticks[3];
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t delta_ticks_front_left = encoder->ticks[0] - last_ticks_1;
  int32_t delta_ticks_front_right = encoder->ticks[1] - last_ticks_2;
  int32_t delta_ticks_rear_left = encoder->ticks[2] - last_ticks_3;
  int32_t delta_ticks_rear_right = encoder->ticks[3] - last_ticks_4;

  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  double omega_1 = M_PI * 2 * delta_ticks_front_left  / (delta_t * ENCODER_TICKS);
  double omega_2 = M_PI * 2 * delta_ticks_front_right / (delta_t * ENCODER_TICKS);
  double omega_3 = M_PI * 2 * delta_ticks_rear_left   / (delta_t * ENCODER_TICKS);
  double omega_4 = M_PI * 2 * delta_ticks_rear_right  / (delta_t * ENCODER_TICKS);
 
  double vx = (omega_1 + omega_2 + omega_3 + omega_4) * WHEEL_RADIUS / 4;
  double vy = (-omega_1 + omega_2 + omega_3 - omega_4) * WHEEL_RADIUS / 4;
  double omega = (-omega_1 + omega_2 - omega_3 + omega_4) * WHEEL_RADIUS / (4 * 2 *WHEEL_BASELINE);
  
  double vx_inercial = vx * cos(theta_) - vy * sin(theta_);
  double vy_inercial = vx * sin(theta_) + vy * cos(theta_);
  double omega_inercial = omega;
  
  double delta_x = vx_inercial * delta_t;
  double delta_y = vy_inercial * delta_t;
  double delta_theta = omega_inercial * delta_t;


  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = omega;

  pub_odometry_->publish( msg );

  // Actualizo las variables de estado

  last_ticks_1 = encoder->ticks[0];
  last_ticks_2 = encoder->ticks[1];
  last_ticks_3 = encoder->ticks[2];
  last_ticks_4 = encoder->ticks[3];
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);


}
