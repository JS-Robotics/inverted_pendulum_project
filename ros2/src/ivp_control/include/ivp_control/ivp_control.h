//
// Created by ICraveSleep on 12.03.23.
//

#ifndef IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
#define IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace ivp {

struct State {
  double angle;  // Angle
  double d_angle;  // Angular velocity
  double position;  // Position
  double d_position; // Linear velocity
};

class Control : public rclcpp::Node {
 public:
  Control();
  ~Control() override;
  bool Configure();
  void RunOnce();
  void Sleep();
  void Publish(float force_setpoint);
  [[nodiscard]] bool NodeOk() const { return !stop_node_; }
 private:
  double SwingUp(const State &state);
 private:
  bool stop_node_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
  std::chrono::time_point<std::chrono::steady_clock> t_end_;
  float kTimerSleep = 1.f / 100.f;

  std_msgs::msg::Float32 float_message_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pendulum_subscription_;
  void PendulumCallback(const geometry_msgs::msg::Vector3 &msg);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cart_subscription_;
  void CartCallback(const std_msgs::msg::Float32 &msg);

  ivp::State state_{};
};

// namespace ivp
}

#endif //IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
