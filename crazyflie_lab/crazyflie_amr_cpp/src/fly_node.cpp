#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <crazyflie_interfaces/srv/arm.hpp>


#include <chrono>

using namespace std::chrono_literals;

class FlyNode : public rclcpp::Node
{
public:

  ~FlyNode(void) = default;

  FlyNode(const std::string &cf_name)
    : Node("fly_node"),
    _is_armed(false)
  {
    // Create Publisher for Turtlebot velocity commands
    std::string vel_topic = "/" + cf_name + "/cmd_vel_legacy";
    _vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10);

    // Arm Request
    _arm_client = this->create_client<crazyflie_interfaces::srv::Arm>("/all/arm");
    if(!_arm_client->wait_for_service(5s)) {
      RCLCPP_WARN(rclcpp::get_logger("fly_node"), "Unable to reach arm service");
    }

    // Timer for setup function
    _setup_timer = this->create_wall_timer(1000ms,
      [this] (void) { this->initial_setup(); });
  }

private:

  void initial_setup(void) 
  {
    if(_is_armed) {
        RCLCPP_INFO(rclcpp::get_logger("fly_node"), "Turning on linear thrust.");
        _vel_cmd.linear.z = kLinearZThrust;
        _setup_timer->cancel();

    } else {
      if( _result.valid()) {
        if(_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          RCLCPP_INFO(rclcpp::get_logger("fly_node"), "Crazyflie Armed.");

          _cmd_timer   = this->create_wall_timer(10ms, [this] (void) { this->command_loop_function(); });
          _setup_timer = this->create_wall_timer(1000ms, [this] (void) { this->initial_setup(); });

          _vel_cmd.linear.x = 0.0;
          _vel_cmd.linear.y = 0.0;
          _vel_cmd.linear.z = 0.0;
          _is_armed = true;
        }
      } else {
        auto req = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
        req->arm = true;

        _result = _arm_client->async_send_request(req).share();
        RCLCPP_INFO(rclcpp::get_logger("fly_node"), "Sending Arm Request");

        _setup_timer = this->create_wall_timer(100ms, [this] (void) { this->initial_setup(); });
      }
    }
  }

  // Function called repeatedly by node.
  void command_loop_function(void)
  {
    _vel_pub->publish(_vel_cmd);
  }

  // ----------------------------------//
  // Variables used by the Node Object //
  // ----------------------------------//

  // ROS service client object
  rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr _arm_client;

  // ROS message publisher object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;

  // Timer for callbacks
  rclcpp::TimerBase::SharedPtr _cmd_timer;
  rclcpp::TimerBase::SharedPtr _setup_timer;

  // Velocity command message
  geometry_msgs::msg::Twist _vel_cmd;

  // Result from service client
  rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedFuture _result;

  // Has the crazyflie been armed??
  bool _is_armed;

  // Constant thrust command in the vertical direction
  static constexpr double kLinearZThrust = 55000.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlyNode>("cf62"));
  rclcpp::shutdown();
  return 0;
}