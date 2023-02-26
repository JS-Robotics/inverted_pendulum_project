//
// Created by ICraveSleep on 23.02.23.
//

#ifndef INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
#define INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include "config.h"
#include "amt21_driver.h"
#include <chrono>

class Pendulum : public rclcpp::Node {
 public:
  Pendulum();

  ~Pendulum();

  void RunOnce();
  void Configure();
  void CleanUp();
  bool NodeOk();

 private:
  Amt21Driver* encoder_;
  bool stop_node_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
  std_msgs::msg::UInt16 message_ = std_msgs::msg::UInt16();
};

#endif //INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
