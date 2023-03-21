//
// Created by ICraveSleep on 26.02.23.
//

#include "amt21_driver.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <ctime>
#include <iomanip>
#include <thread>

std::string format_time(uint64_t nanoseconds_since_epoch) {
  std::time_t seconds_since_epoch =
      static_cast<std::time_t>(nanoseconds_since_epoch / 1000000000ull);

  std::tm tm_struct = *std::gmtime(&seconds_since_epoch);
  uint32_t nanoseconds =
      static_cast<uint32_t>(nanoseconds_since_epoch % 1000000000ull);

  std::ostringstream oss;
  oss << std::put_time(&tm_struct, "%Y-%m-%dT%H:%M:%S");
  oss << "." << std::setw(9) << std::setfill('0') << nanoseconds << "Z";

  return oss.str();
}

std::string format_time_3decimals(uint64_t nanoseconds_since_epoch) {
  std::time_t seconds_since_epoch =
      static_cast<std::time_t>(nanoseconds_since_epoch / 1000000000ull);

  std::tm tm_struct = *std::gmtime(&seconds_since_epoch);
  uint32_t milliseconds =
      static_cast<uint32_t>((nanoseconds_since_epoch / 1000000ull) % 1000);

  std::ostringstream oss;
  oss << std::put_time(&tm_struct, "%Y-%m-%dT%H:%M:%S");
  oss << "." << std::setw(3) << std::setfill('0') << milliseconds << "Z";

  return oss.str();
}

struct Sample {
  uint16_t position{};
  uint64_t epoch_time_ns{};
  std::string iso8601_time{};
};

bool FileExists(const std::string &path) {
  std::ifstream file(path.c_str());
  return file.good();
}

int main() {

  // Path starts from workspace (ros2 directory)
  std::string sample_set_description = "Sample set for CW rotation start at Horizontal plane";
  std::string file_path = "./src/ivp_pendulum/tools/pendulum_data/position_data_sets/";
  std::string file_prefix = "cw_sample_set";
  std::string file = "cw_sample_set0.csv";
  std::string full_path = file_path + file;
  int index = 0;
  while (FileExists(full_path)) {
    std::cout << "File: " << file << ", Already exists." << std::endl;
    index++;
    file = file_prefix + std::to_string(index) + ".csv";
    full_path = file_path + file;
  }

  std::cout << "Creating file: " << file << std::endl;
  std::ofstream DataFile(full_path);

  // Prepare encoder
  uint16_t encoder_value;
  Amt21Driver driver = Amt21Driver("/dev/ttyUSB0",
                                   AMT21Resolution::k14Bit,
                                   AMT21BaudRate::k115200,
                                   AMT21TurnType::kSingleTurn);
  bool opened = driver.Open();
  if (!opened) {
    std::cout << "No access to usb port. Please give read and write access" << std::endl;
    return 0;
  }

  //Create and reserve large vector for data collection
  std::vector<Sample> SampleSet;
  double timer_sleep = 0.02f;
  int set_size = 4500;  // Sample for 60seconds
  SampleSet.reserve(set_size);
  SampleSet.resize(set_size);
  uint16_t encoder_value_old;
  for (int i = 0; i < set_size; i++) {
    std::chrono::time_point t_start = std::chrono::steady_clock::now();

    encoder_value = driver.GetEncoderPosition();

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<uint64_t, std::nano>
        nano_seconds = std::chrono::duration_cast<std::chrono::duration<uint64_t, std::nano>>(now.time_since_epoch());

    if(!driver.ChecksumFailed()){
      SampleSet.at(i).position = encoder_value;
      encoder_value_old = encoder_value;
    }
    else{
      SampleSet.at(i).position = encoder_value_old;
    }

    SampleSet.at(i).epoch_time_ns = nano_seconds.count();

    std::chrono::time_point t_stop = std::chrono::steady_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < timer_sleep) {
      std::this_thread::sleep_for(std::chrono::duration<double>(timer_sleep - t_duration.count()));
    } else {
      std::cout << "Time overflow: " << t_duration.count() << std::endl;
    }
  }
  driver.Close();

  DataFile << sample_set_description << "\n";
  DataFile << "Position, Epoch time ns, ISO8601\n";
  for (int i = 0; i < set_size; i++) {
    DataFile << SampleSet.at(i).position << "," << SampleSet.at(i).epoch_time_ns << ","
             << format_time(SampleSet.at(i).epoch_time_ns) << "\n";
  }
  DataFile.close();
  return 0;
}

void time_demo() {
  std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::system_clock::now();

  std::chrono::duration<uint64_t, std::nano> nano_seconds =
      std::chrono::duration_cast<std::chrono::duration<uint64_t, std::nano>>(
          now.time_since_epoch());

  uint64_t nanoseconds = nano_seconds.count();
  std::cout << nanoseconds << std::endl;

  std::string isostandard = format_time(nanoseconds);
  std::cout << isostandard << std::endl;

  std::string isostandard2 = format_time_3decimals(nanoseconds);
  std::cout << isostandard2 << std::endl;
}