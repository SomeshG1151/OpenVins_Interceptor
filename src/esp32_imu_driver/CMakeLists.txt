cmake_minimum_required(VERSION 3.8)
project(esp32_imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Use system libserial
find_library(SERIAL_LIBRARY serial)

add_executable(esp32_imu_driver src/esp32_imu_driver.cpp)
ament_target_dependencies(esp32_imu_driver rclcpp sensor_msgs)
target_link_libraries(esp32_imu_driver ${SERIAL_LIBRARY})

install(TARGETS esp32_imu_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()