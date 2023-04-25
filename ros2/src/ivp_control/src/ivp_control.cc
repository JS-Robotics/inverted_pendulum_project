#include <cstdio>
#include "ivp_control/ivp_control.h"
namespace ivp {
Control::Control() : Node("IvpControlNode") {
  stop_node_ = false;
  state_ = State{};
  std::cout << state_.angle << state_.d_angle << state_.position << state_.d_position << std::endl;
  std::cout << "Hello" << stop_node_ << std::endl;
}

Control::~Control() {
  std::cout << "Destructor" << std::endl;
}

bool Control::Configure() {
  publisher_ = this->create_publisher<std_msgs::msg::Float32>("ivp/force_setpoint", rclcpp::SensorDataQoS());

  pendulum_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("ivp/pendulum_state",
                                                                                  rclcpp::SensorDataQoS(),
                                                                                  std::bind(&Control::PendulumCallback,
                                                                                            this,
                                                                                            std::placeholders::_1));
  cart_subscription_ = this->create_subscription<std_msgs::msg::Float32>("ivp/cart_position",
                                                                         rclcpp::SensorDataQoS(),
                                                                         std::bind(&Control::CartCallback,
                                                                                   this,
                                                                                   std::placeholders::_1));

  return true;
}

void Control::RunOnce() {
  t_start_ = std::chrono::steady_clock::now();  //Begin execution timer
//  auto torque = static_cast<float>(SwingUp(state_));
//  Publish(torque);

//  std::cout << "------------------------------------------------------------------" << std::endl;
//  std::cout << "angle: " << state_.angle << std::endl;
//  std::cout << "d_angle: " << state_.d_angle << std::endl;
//  std::cout << "position: " << state_.position << std::endl;
//  std::cout << "------------------------------------------------------------------" << std::endl;

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
                t_duration.count() - kTimerSleep);
  }
}

void Control::Publish(float force_setpoint) {
  float_message_.data = force_setpoint;
  publisher_->publish(float_message_);
}

double Control::SwingUp(const State &state) {
  double e_p = 0.0f;
  double I_p = 0.00466;
  double m_p = 0.071f;
  double m_c = 0.288f;
  double L_p = (0.685f - 0.246f);
  double g = 9.81f;
  double F_m = 0;
  double b_c = 0.095f;
  double e_t = m_p * g * L_p;
  // 1/2 * masspole * (2 * length)**2 / 3 *  x[3]**2 + np.cos(x[2]) * polemass_length * gravity
  // 0.5 * mv² + cos(theta)*m_p*l_p*g
  // 0.5 * m*(r*w)² + cos(theta)*m_p*l_p*g
  // 0.5 * m*r²*w² + cos(theta)*m_p*l_p*g
  // E = 0.5 * J * w² + mgh, h = l*cos(theta)
  //  if (state.angle > 1.571 || state.angle < 4.712) {
//    e_p = 0.5 * I_p * state.d_angle * state.d_angle + m_p * g * L_p * cos(state.angle);
    e_p = m_p * g * L_p * cos(state.angle);
    double value = ((e_t - e_p) * state.d_angle * cos(state.angle));
    return value;
}

void Control::PendulumCallback(const geometry_msgs::msg::Vector3 &msg) {
  state_.angle = msg.x;
  state_.d_angle = msg.y;
}

void Control::CartCallback(const std_msgs::msg::Float32 &msg) {
  state_.position = msg.data;
}

// namespace ivp
}