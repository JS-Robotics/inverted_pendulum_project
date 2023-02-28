#######################################
# Created by ICraveSleep on 23.02.23. #
#######################################

cmake_minimum_required(VERSION 3.8)
project(ivp_pendulum)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amt21_driver REQUIRED)


add_library(config_header src/config.h)
target_link_libraries(config_header amt21_driver)
set_target_properties(config_header PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(config_header PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        )

add_library(IvpPendulum src/ivp_pendulum.cc include/ivp_pendulum/ivp_pendulum.h)
ament_target_dependencies(IvpPendulum rclcpp geometry_msgs)
target_link_libraries(IvpPendulum config_header amt21_driver)
target_include_directories(IvpPendulum PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        )


add_executable(ivp_pendulum src/main.cc)
target_link_libraries(ivp_pendulum IvpPendulum)


add_executable(pendulum_sampler tools/pendulum_data/data_sampler.cpp)
target_link_libraries(pendulum_sampler amt21_driver)

target_include_directories(ivp_pendulum PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(ivp_pendulum PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ivp_pendulum pendulum_sampler
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
