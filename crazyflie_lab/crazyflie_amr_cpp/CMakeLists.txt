cmake_minimum_required(VERSION 3.8)
project(crazyflie_amr_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)


add_subdirectory(src)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
