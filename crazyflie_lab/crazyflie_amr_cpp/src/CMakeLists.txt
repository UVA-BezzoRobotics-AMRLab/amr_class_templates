add_executable(fly_node fly_node.cpp)
ament_target_dependencies(fly_node rclcpp crazyflie_interfaces geometry_msgs)

add_executable(pid_tuner_cpp pid_tuner_answer.cpp)
ament_target_dependencies(pid_tuner_cpp rclcpp crazyflie_interfaces geometry_msgs) 

install(TARGETS
    fly_node
    pid_tuner_cpp
    DESTINATION lib/${PROJECT_NAME})