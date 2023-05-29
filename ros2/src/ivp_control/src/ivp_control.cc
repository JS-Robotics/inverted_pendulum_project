#include <cstdio>
#include "ivp_control/ivp_control.h"
#include <cmath>
namespace ivp {

Control::Control() : Node("IvpControlNode") {
  stop_node_ = false;
  ref_.p_ref = 0;
  ref_.v_ref = 0;
  ref_.t_ref = kPi;
  ref_.w_ref = 0;
  state_ = State{};
  std::cout << state_.angle << state_.d_angle << state_.position << state_.d_position << std::endl;
  use_lqr = false;
  std::cout << "Hello" << stop_node_ << std::endl;
}

Control::~Control() {
  std::cout << "Destructor" << std::endl;
}

bool Control::Configure() {
  publisher_ = this->create_publisher<std_msgs::msg::Float32>("force_setpoint", rclcpp::SensorDataQoS());

  pendulum_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("pendulum_state",
                                                                                  rclcpp::SensorDataQoS(),
                                                                                  std::bind(&Control::PendulumCallback,
                                                                                            this,
                                                                                            std::placeholders::_1));
  cart_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("cart_state",
                                                                              rclcpp::SensorDataQoS(),
                                                                              std::bind(&Control::CartCallback,
                                                                                        this,
                                                                                        std::placeholders::_1));

  t_init_ = std::chrono::steady_clock::now();

  return true;
}

void Control::RunOnce() {
  t_start_ = std::chrono::steady_clock::now();  //Begin execution timer
  float force;
  if (state_.angle == 0 && state_.d_angle == 0) {
    force = 0;
  } else {
    if (use_lqr) {
      force = Balancing(state_);
    } else {
      force = SwingUp(state_);
    }

  }
//  if(state_.angle > 1.74533 || state_.angle < 1.39626){
//    force = 0;
//  }

  Publish(force);

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

float Control::SwingUp(const State &state) {
  double e_p = 0.0f;
  double I_p = 0.00466;
  double m_p = 0.071f;
  double m_c = 0.288f;
  double L_p = (0.685f - 0.246f);
  double g = 9.81f;
  double e_t = m_p * g * L_p;
  double pi = 3.14159265359;
  double b_p = 0.00112297;
  auto time = std::chrono::steady_clock::now();
  double u_t;

  e_p = 0.5f * I_p * state.d_angle * state.d_angle - m_p * g * L_p * cos(state.angle);
  u_t = 0;

  if (cos(state.angle) > 0.8) {
    u_t = (e_p - e_t) * state.d_angle * cos(state.angle) * 2.155;
  }

  std::cout << u_t << "  --  energy: " << (e_t - e_p) << std::endl;
//  std::cout << "e_t=" << e_t << " e_p=" << e_p << "  --  diff " << (e_t - e_p) <<  std::endl;
//  std::cout << u_t << " Error: " << error << std::endl;


  if (std::abs(u_t) > 20.0f) {
    if (u_t < 0) {
      u_t = -20.0f;
    } else {
      u_t = 20.0f;
    }
  }
  if(state.angle <= 3.22886 && state.angle >= 3.05433){
    use_lqr = true;
  }
//  u_t = 0;
  return static_cast<float>(u_t);
}

float Control::Balancing(const State &state) {
  ref_.t_ref += 0.0000035f * state.d_position;
//  std::cout << ref_.t_ref << std::endl;
  // --> 2.5 N
  // <-- -2.4 N
  double u_t = -feedback_gain_.k1 * (state.position - ref_.p_ref)
      - feedback_gain_.k2 * (state.d_position - ref_.v_ref)
      - feedback_gain_.k3 * (state.angle - ref_.t_ref)
      - feedback_gain_.k4 * (state.d_angle - ref_.w_ref);
  double u_init = u_t;
//  if (std::abs(state.d_position) <= 0.0005 && u_t < 0.01) {
  if (std::abs(state.d_position) <= 0.05 && std::abs(u_t) >= 0.01) {
//  if (std::abs(u_t) > 0.35) {
    u_t += 2.1 * copysign(1.0, u_t);
//    u_t += 2.0 * copysign(1.0, u_t);
  } else {
    u_t += 2.6 * std::tanh(100 * state.d_position);
//    u_t += 2.5 * std::tanh(100 * state.d_position);
  }

  if (std::abs(u_t) > 25.0f) {
    if (u_t < 0) {
      u_t = -25.0f;
    } else {
      u_t = 25.0f;
    }
  }

  std::cout << "Compensated F:" << u_t << "  -  Original F:" << u_init << std::endl;

//  if(state.angle >= 2.0944 || state.angle <= 1.0472){
//    u_t = 0;
//  }

  return static_cast<float>(u_t);
}

void Control::PendulumCallback(const geometry_msgs::msg::Vector3 &msg) {
  state_.angle = msg.x;
  state_.d_angle = msg.y;
}

void Control::CartCallback(const geometry_msgs::msg::Vector3 &msg) {
  state_.position = msg.x;
  state_.d_position = msg.y;
}

template<typename T>
int Control::GetSign(T value) {
  return (T(0) < value) - (value < T(0));
}



// namespace ivp
}