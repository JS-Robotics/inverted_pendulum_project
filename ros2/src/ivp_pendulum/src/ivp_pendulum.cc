//
// Created by ICraveSleep on 23.02.23.
//

#include "ivp_pendulum/ivp_pendulum.h"

Pendulum::Pendulum() : Node("ThisIsNodeName") {
  encoder_ = nullptr;
  stop_node_ = false;
  message_.z = 0.0f;
  pos_old_ = 0.0f;
  pos_filtered_ = 0.0f;
  dt_ = static_cast<float>(PendulumConfig::timer_sleep);
}

Pendulum::~Pendulum() {
  stop_node_ = true;
  this->CleanUp();
}

void Pendulum::Configure() {

  publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(PublisherConfig::kTopic, rclcpp::SensorDataQoS());

  std::string usb_port = EncoderConfig::kUsbPort;
  encoder_ = new Amt21Driver(usb_port,
                             EncoderConfig::kEncoderResolution,
                             EncoderConfig::kEncoderBaudRate,
                             EncoderConfig::kEncoderTurnType);

  bool opened = encoder_->Open();
  if (!opened) {
    RCLCPP_ERROR(this->get_logger(), "Not able to access port: %s", usb_port.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Connected to encoder with error code: %u", encoder_->GetEncoderError());
}

void Pendulum::RunOnce() {
  t_start_ = std::chrono::steady_clock::now();  //Begin execution timer

  float encoder_value = encoder_->GetEncoderAngle();
  if (encoder_->ChecksumFailed()) {
    RCLCPP_WARN(this->get_logger(), "Checksum failed");
    publisher_->publish(message_);  // If checksum fails just republish previous message. Not much difference due to sampling.
  } else {
    pos_filtered_ = PendulumConfig::alpha * encoder_value + ((1 - PendulumConfig::alpha) * pos_filtered_);
    float vel = (pos_filtered_ - pos_old_) / dt_;
    pos_old_ = pos_filtered_;
    message_.x = pos_filtered_;
    message_.y = vel;
    publisher_->publish(message_);
  }

}
bool Pendulum::NodeOk() {
  return stop_node_;
}

void Pendulum::CleanUp() {
  if (encoder_->PortOpen() && encoder_ != nullptr) {
    encoder_->Close();
    RCLCPP_INFO(this->get_logger(), "Closed USB/UART port");
  }
  if (encoder_ != nullptr) {
    delete encoder_;
    encoder_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "Deleted encoder object");
  }
}

void Pendulum::Sleep() {
  t_end_ = std::chrono::steady_clock::now();
  auto t_duration = std::chrono::duration<double>(t_end_ - t_start_);
  RCLCPP_DEBUG(this->get_logger(), "Loop time: %f[s]", t_duration.count());
  if (t_duration.count() < PendulumConfig::timer_sleep) {
    std::this_thread::sleep_for(std::chrono::duration<double>(PendulumConfig::timer_sleep - t_duration.count()));
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Loop time: %f[ms], Overtime: %f[s] ",
                t_duration.count() * 1000,
                PendulumConfig::timer_sleep - t_duration.count());
  }
}
