//
// Created by sondre on 23.02.23.
//

#include "../include/ivp_pendulum/ivp_pendulum.h"

void Pendulum::RunOnce() {
  std::chrono::time_point t_start = std::chrono::steady_clock::now();

  std::cout << "Hello pendulum" << std::endl;

  std::chrono::time_point t_stop = std::chrono::steady_clock::now();
  auto t_duration = std::chrono::duration<double>(t_stop - t_start);

  if (t_duration.count() < PendulumConfig::timer_sleep) {
    std::this_thread::sleep_for(std::chrono::duration<double>(PendulumConfig::timer_sleep - t_duration.count()));
  }
  else {
    std::cout << "Overtime" << std::endl;
  }
}
