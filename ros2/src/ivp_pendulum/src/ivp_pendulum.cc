//
// Created by ICraveSleep on 23.02.23.
//

#include "../include/ivp_pendulum/ivp_pendulum.h"

Pendulum::Pendulum() : Node("ThisIsNodeName") {
  encoder_ = nullptr;
  stop_node_ = false;
}

Pendulum::~Pendulum() {
  stop_node_ = true;
  this->CleanUp();
}

void Pendulum::Configure() {

  publisher_ = this->create_publisher<std_msgs::msg::UInt16>(PublisherConfig::kTopic, 10);

  std::string usb_port = "/dev/ttyUSB0";
  encoder_ = new Amt21Driver(usb_port,
                             EncoderConfig::kEncoderResolution,
                             EncoderConfig::kEncoderBaudRate,
                             EncoderConfig::kEncoderTurnType);

  bool opened = encoder_->Open();
  if (!opened) {
    RCLCPP_ERROR(this->get_logger(), "Not able to access port: %s", usb_port.c_str());
  }

//  RCLCPP_DEBUG(this->get_logger(), "DEBUG PRINT");
//  RCLCPP_INFO(this->get_logger(), "INFO PRINT");
//  RCLCPP_WARN(this->get_logger(), "WARN PRINT");
//  RCLCPP_ERROR(this->get_logger(), "ERROR PRINT");

  RCLCPP_INFO(this->get_logger(), "Connected to encoder with error code: %u", encoder_->GetEncoderError());

}

void Pendulum::RunOnce() {
  std::chrono::time_point t_start = std::chrono::steady_clock::now();
  uint16_t encoder_value = encoder_->GetEncoderPosition();
  message_.data = encoder_value;
  publisher_->publish(message_);
  std::chrono::time_point t_stop = std::chrono::steady_clock::now();
  auto t_duration = std::chrono::duration<double>(t_stop - t_start);
//  std::cout << "Request time: " << t_duration.count() << "Encoder: " << encoder_value << std::endl;
  if (t_duration.count() < PendulumConfig::timer_sleep) {

    std::this_thread::sleep_for(std::chrono::duration<double>(PendulumConfig::timer_sleep - t_duration.count()));
  } else {
    RCLCPP_WARN(this->get_logger(), "Overtime: %f", t_duration.count());
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
