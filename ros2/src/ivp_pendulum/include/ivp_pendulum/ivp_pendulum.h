//
// Created by ICraveSleep on 23.02.23.
//

#ifndef INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
#define INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

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
  void Sleep();

 private:
  Amt21Driver *encoder_;
  bool stop_node_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  geometry_msgs::msg::Vector3 message_ = geometry_msgs::msg::Vector3();
  float pos_old_;
  float pos_filtered_;
  float vel_filtered_;
  float dt_;
  std::chrono::time_point<std::chrono::steady_clock> t_start_;
  std::chrono::time_point<std::chrono::steady_clock> t_end_;

  static constexpr float kPi = 3.14159265359;
  static constexpr float kOverflowDiffThreshold = 5.5;
};

#endif //INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
