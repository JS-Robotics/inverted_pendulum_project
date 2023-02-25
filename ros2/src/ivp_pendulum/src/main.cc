

#include "ivp_pendulum/ivp_pendulum.h"

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pendulum>();
  node->Configure();
  while (rclcpp::ok() && !stop_node) {
    node->RunOnce();
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
