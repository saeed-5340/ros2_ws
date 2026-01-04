cmake_minimum_required(VERSION 3.8)
project(esp32_cheak)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(talker src/node_1.cpp)
add_executable(listener src/node_2.cpp)
add_executable(uart_sub src/node_3.cpp)

ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(uart_sub rclcpp std_msgs)

install(TARGETS
  talker
  listener
  uart_sub
  DESTINATION lib/${PROJECT_NAME})


ament_package()
