cmake_minimum_required(VERSION 3.8)
project(movement_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(movement_publisher_exec src/movement_publisher_node.cpp)
ament_target_dependencies(movement_publisher_exec rclcpp)

install(TARGETS
  movement_publisher_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
