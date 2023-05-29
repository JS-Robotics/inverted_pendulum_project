//
// Created by ICraveSleep on 12.03.23.
//

#ifndef IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
#define IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace ivp {

struct StateFeedbackGain{
  double k1 = -4.47213595; // -3.9223227; //-4.47213595;
  double k2 = -5.30616268; // -5.08303039; //-5.32032761;
  double k3 = 31.10301188; // 31.93768246; //30.5649182;
  double k4 = 8.61122567; // 8.0384753; //6.59008009;
};

struct SystemRef{
  double p_ref{};
  double v_ref{};
  double t_ref{};
  double w_ref{};
};

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
  float SwingUp(const State &state);
  float Balancing(const State &state);
  template <typename T> int GetSign(T value);
 private:
  bool stop_node_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
  std::chrono::time_point<std::chrono::steady_clock> t_end_;
  std::chrono::time_point<std::chrono::steady_clock> t_init_;
  float kTimerSleep = 1.f / 100.f;
  float kPi = 3.14159265359;
  bool use_lqr;

  std_msgs::msg::Float32 float_message_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pendulum_subscription_;
  void PendulumCallback(const geometry_msgs::msg::Vector3 &msg);

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cart_subscription_;
  void CartCallback(const geometry_msgs::msg::Vector3 &msg);

  ivp::State state_{};

  SystemRef ref_;
  StateFeedbackGain feedback_gain_;
};

// namespace ivp
}

#endif //IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
