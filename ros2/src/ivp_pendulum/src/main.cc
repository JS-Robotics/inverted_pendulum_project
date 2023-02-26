//
// Created by ICraveSleep on 23.02.23.
//

#include "ivp_pendulum/ivp_pendulum.h"

[[maybe_unused]] static void SignalHandler(int) {
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;


// This method does not seem to call destructor of objects used by node
//  signal(SIGINT, SignalHandler);
//  rclcpp::init(argc, argv);
//  auto node = std::make_shared<Pendulum>();
//  node->Configure();
//  while (rclcpp::ok() && !node->NodeOk()) {
//    node->RunOnce();
//    rclcpp::spin_some(node);
//  }
//  rclcpp::shutdown();
//  return 0;


  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<Pendulum>();
  node->Configure();
  while (rclcpp::ok() && !node->NodeOk()) {
    node->RunOnce();
    executor.spin_node_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
