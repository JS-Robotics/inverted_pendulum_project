//
// Created by sondre on 23.02.23.
//

#ifndef INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
#define INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_

#include "rclcpp/rclcpp.hpp"
#include "config.h"
#include <chrono>

class Pendulum : public rclcpp::Node {
 public:
  Pendulum() : Node("ThisIsNodeName") {};

  ~Pendulum() = default;

  void RunOnce();

};

#endif //INVERTED_PENDULUM_PROJECT_ROS2_SRC_IVP_PENDULUM_SRC_PENDULUM_H_
