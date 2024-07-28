#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>

using namespace std::chrono_literals;

class TurtlebotTurn : public rclcpp::Node
{
public:
  TurtlebotTurn() 
      : Node("ttb_turn")
  {
    // Create Subscribe to the IMU data topic fromm Turtlebot
    imu_sub  = this->create_subscription<sensor_msgs::msg::Imu>(
      "TTBXX/imu", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
      [this](const sensor_msgs::msg::Imu &msg) { this->imu_callback(msg); }
    );

    // Create Publisher for Turtlebot velocity commands
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("TTBXX/cmd_vel", 10);

    // Timer object that controls how often your command loop function is called
    timer = this->create_wall_timer(
      100ms,                                           // Period of rate that function is called
      [this] (void) { this->command_loop_function(); } // Which function to call
    );
  }

private:

  // Callback function for receiving Imu sensor data
  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    angular_velocity_z = msg.angular_velocity.z;
  }

  // Function called repeatedly by node.
  void command_loop_function(void)
  {
    RCLCPP_INFO(this->get_logger(), "Angular Vel: %0.3f", angular_velocity_z);
    if(count < 50) {
      vel_cmd.angular.z = 0.4;
    } else {
      vel_cmd.angular.z = 0.0;
    }

    vel_pub->publish(vel_cmd);

    ++count;
  }

  // ---------------------------------//
  // Variable used by the Node Object //
  // ---------------------------------//

  // ROS subsriber object
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  // ROS publisher object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  // Control timing control
  rclcpp::TimerBase::SharedPtr timer;

  // Velocity command message
  geometry_msgs::msg::Twist vel_cmd;

  // Variable to hold angular velocity data
  double angular_velocity_z;

  // Counter
  int count = 0;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotTurn>());
  rclcpp::shutdown();

  return 0;
}