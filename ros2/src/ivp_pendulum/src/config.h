//
// Created by ICraveSleep on 23.02.23.
//

#ifndef IVP_PENDULUM_SRC_CONFIG_H_
#define IVP_PENDULUM_SRC_CONFIG_H_

#include "amt21_driver.h"

namespace EncoderConfig {
static constexpr AMT21TurnType kEncoderTurnType = AMT21TurnType::kSingleTurn;
static constexpr AMT21Resolution kEncoderResolution = AMT21Resolution::k14Bit;
static constexpr AMT21BaudRate kEncoderBaudRate = AMT21BaudRate::k115200;
static constexpr char kUsbPort[] = "/dev/ttyUSB0";
}

namespace PendulumConfig {
static constexpr double timer_sleep = 0.02f;
static constexpr float alpha = 0.5f; // Low pass Filter - high frequency attenuation constant
}

namespace PublisherConfig {
static constexpr char kTopic[15] = "pendulum_state";
}

#endif //IVP_PENDULUM_SRC_CONFIG_H_
