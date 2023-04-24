#include <cstdio>
#include "ivp_control/ivp_control.h"
namespace ivp {
Control::Control() : Node("IvpControlNode") {
  stop_node_ = false;
  std::cout << "Hello" << stop_node_ << std::endl;
}

Control::~Control() {
  std::cout << "Destructor" << std::endl;
}

bool Control::Configure() {
  publisher_ = this->create_publisher<std_msgs::msg::Float32>("ivp/force_setpoint", rclcpp::SensorDataQoS());
  return true;
}

void Control::RunOnce() {
  t_start_ = std::chrono::steady_clock::now();  //Begin execution timer
  float test = 0.01;
  Publish(test);
}

void Control::Sleep() {
  t_end_ = std::chrono::steady_clock::now();
  auto t_duration = std::chrono::duration<double>(t_end_ - t_start_);
  RCLCPP_DEBUG(this->get_logger(), "Loop time: %f[s]", t_duration.count());
  if (t_duration.count() < kTimerSleep) {
    std::this_thread::sleep_for(std::chrono::duration<double>(kTimerSleep - t_duration.count()));
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Loop time: %f[ms], Overtime: %f[s] ",
                t_duration.count() * 1000,
                kTimerSleep - t_duration.count());
  }
}

void Control::Publish(float force_setpoint) {
  message_.data = force_setpoint;
  publisher_->publish(message_);
}

// namespace ivp
}