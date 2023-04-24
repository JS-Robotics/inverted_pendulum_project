cmake_minimum_required(VERSION 3.16)
project(ivp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(IvpControl src/ivp_control.cc)
target_include_directories(IvpControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(IvpControl rclcpp::rclcpp ${std_msgs_TARGETS})


add_executable(ivp_control src/main.cc)
target_link_libraries(ivp_control IvpControl)
#target_include_directories(ivp_control PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

target_compile_features(ivp_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ivp_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
