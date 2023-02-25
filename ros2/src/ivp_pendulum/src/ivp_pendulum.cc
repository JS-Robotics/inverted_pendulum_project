//
// Created by sondre on 23.02.23.
//

#include "../include/ivp_pendulum/ivp_pendulum.h"

Pendulum::Pendulum() : Node("ThisIsNodeName") {
  encoder_ = nullptr;
  stop_node = false;
}

Pendulum::~Pendulum(){
  if(encoder_->PortOpen() && encoder_ != nullptr){
    encoder_->Close();
    RCLCPP_INFO(this->get_logger(), "Closed USB/UART port");
  }
  if(encoder_ != nullptr){
    delete encoder_;
    encoder_= nullptr;
    RCLCPP_INFO(this->get_logger(), "Deleted encoder object");
  }
}

void Pendulum::Configure() {
  std::string usb_port = "/dev/ttyUSB0";
  encoder_ = new Amt21Driver(usb_port,
                             EncoderConfig::kEncoderResolution,
                             EncoderConfig::kEncoderBaudRate,
                             EncoderConfig::kEncoderTurnType);




  bool opened = encoder_->Open();
  if(!opened){
    RCLCPP_ERROR(this->get_logger(), "Not able to access port: %s", usb_port.c_str());
  }
  std::cout << "Error code: " << +encoder_->GetEncoderError() << std::endl;
}

void Pendulum::RunOnce() {
  std::chrono::time_point t_start = std::chrono::steady_clock::now();
  uint16_t encoder_value = encoder_->GetEncoderPosition();

//  std::cout << "Encoder position is: " << encoder_value << std::endl;
//  std::cout << "Hello" << std::endl;

  std::chrono::time_point t_stop = std::chrono::steady_clock::now();
  auto t_duration = std::chrono::duration<double>(t_stop - t_start);

  if (t_duration.count() < PendulumConfig::timer_sleep) {
    std::this_thread::sleep_for(std::chrono::duration<double>(PendulumConfig::timer_sleep - t_duration.count()));
  } else {
    std::cout << "Overtime: " << t_duration.count() <<  std::endl;
  }

//  uint16_t encoder_value;
//  float angle;
//  Amt21Driver driver = Amt21Driver("/dev/ttyUSB0",
//                                   AMT21Resolution::k14Bit,
//                                   AMT21BaudRate::k115200,
//                                   AMT21TurnType::kSingleTurn);
//
//  bool opened = driver.Open();
//  if(!opened){
//    std::cout << "No access to usb port. Please give read and write access" << std::endl;
//  }
//  std::cout << "ErrorCode: " << +driver.GetEncoderError() << std::endl;
//
//  encoder_value = driver.GetEncoderPosition();
//  std::cout << "Encoder Position: " << encoder_value << std::endl;
//  int counter = 0;
//  while (counter < 10) {
//    angle = driver.GetEncoderPosition();
//    std::cout << "Angle: " << angle << std::endl;
//    std::cout << "ErrorCode: " << +driver.GetEncoderError() << std::endl;
//    counter++;
//  }
//
//  driver.Close();

}
