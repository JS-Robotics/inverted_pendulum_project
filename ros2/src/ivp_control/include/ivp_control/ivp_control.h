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
  float k1 = -1; //-3.16227766; //-31.622776;
  float k2 = -1.53667584; //-34.01478446;
  float k3 = 13.18171772; //23.9763163; //166.47118146;
  float k4 = 2.99319516; //33.6406612;
};

struct SystemRef{
  float p_ref{};
  float v_ref{};
  float t_ref{};
  float w_ref{};
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

  std_msgs::msg::Float32 float_message_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pendulum_subscription_;
  void PendulumCallback(const geometry_msgs::msg::Vector3 &msg);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cart_subscription_;
  void CartCallback(const std_msgs::msg::Float32 &msg);

  ivp::State state_{};

  SystemRef ref_;
  StateFeedbackGain feedback_gain_;
};

// namespace ivp
}

#endif //IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
