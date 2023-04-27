#include <cstdio>
#include "ivp_control/ivp_control.h"
#include <cmath>
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
  auto torque = static_cast<float>(SwingUp(state_));
  Publish(torque);

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
  double e_t = 0.0f; //m_p * g * L_p;
  double pi = 3.14159265359;

  double angle = state.angle;
  if(state.angle < 2*pi && state.angle >  2*pi - 0.5*pi){
    angle = state.angle - 2*pi;
  }
  if(state.angle < 2*pi - 0.5*pi && state.angle > 2*pi - 1.5*pi ){
    angle = state.angle - pi;
  }

    e_p = 0.5*I_p*state.d_angle*state.d_angle + m_p * g * L_p * (cos(angle));
    std::cout << "Cos: " << cos(angle) << " Angle: " << angle << std::endl;
    double value = (e_t - e_p) * state.d_angle * (cos(angle));
    std::cout << "e_t: " << e_t << " e_p: " << e_p << " force: " << value << std::endl;
    if (state.angle > 2.61799388 && state.angle < 3.66519143){
      value = 0.0;
    }

    return value;
}

void Control::PendulumCallback(const geometry_msgs::msg::Vector3 &msg) {
  state_.angle = msg.x;
  state_.d_angle = msg.y;
}

void Control::CartCallback(const std_msgs::msg::Float32 &msg) {
  state_.position = msg.data;
}

template<typename T>
int Control::GetSign(T value) {
  return (T(0) < value) - (value < T(0));
}



// namespace ivp
}