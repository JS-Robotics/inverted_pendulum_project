//
// Created by ICraveSleep on 24.04.23.
//

#include "ivp_control/ivp_control.h"

[[maybe_unused]] static void SignalHandler(int) {
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<ivp::Control> node = std::make_shared<ivp::Control>();
  node->Configure();
  while (rclcpp::ok() && node->NodeOk()) {
    node->RunOnce();
    executor.spin_node_some(node);
    node->Sleep();
  }
  rclcpp::shutdown();
  return 0;
}