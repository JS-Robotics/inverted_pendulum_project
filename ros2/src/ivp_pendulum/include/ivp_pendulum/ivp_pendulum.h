//
// Created by sondre on 23.02.23.
//

#ifndef INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
#define INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_

#include "rclcpp/rclcpp.hpp"
#include "config.h"
#include "amt21_driver.h"
#include <chrono>

class Pendulum : public rclcpp::Node {
 public:
  Pendulum();

  ~Pendulum();

  void RunOnce();
  void Configure();
  bool NodeOk();

 private:
  Amt21Driver* encoder_;
  bool stop_node_;
};

#endif //INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
