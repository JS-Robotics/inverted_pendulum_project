//
// Created by ICraveSleep on 12.03.23.
//

#ifndef IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
#define IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace ivp {
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
  bool stop_node_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
  std::chrono::time_point<std::chrono::steady_clock> t_end_;
  float kTimerSleep = 1.f/100.f;

  std_msgs::msg::Float32 message_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

};

// namespace ivp
}

#endif //IVP_CONTROL_INCLUDE_IVP_CONTROL_IVP_CONTROL_H_
