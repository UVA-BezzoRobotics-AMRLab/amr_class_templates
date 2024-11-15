#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>

#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

class PidTunerNode : public rclcpp::Node
{
public:
  ~PidTunerNode(void) = default;

  PidTunerNode(const std::string &cf_name="cf80") 
    : Node("pid_tuner"),
    _cf_name(cf_name),
    _vel_cmd_active(false),
    _ang_vel_received(false)
  {
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data);

    _vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/" + cf_name + "/cmd_vel_legacy", qos_profile,
      [this](const geometry_msgs::msg::Twist &msg) { this->vel_callback(msg); });

    _gyro_sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/" + cf_name + "/gyro_data", qos_profile,
      [this](const crazyflie_interfaces::msg::LogDataGeneric &msg) { this->gyro_callback(msg); });

    _accel_sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/" + cf_name + "/accel_data", qos_profile,
      [this](const crazyflie_interfaces::msg::LogDataGeneric &msg) { this->accel_callback(msg); });

    _setup_timer = this->create_wall_timer(500ms, [this](void) { this->setup(); });

    _set_param_client = this->create_client<rcl_interfaces::srv::SetParameters>("/crazyflie_server/set_parameters");
  }

private:

  void setup(void) 
  {
    if(_set_param_client->service_is_ready()) {
      RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Set Parameter Service is ready.");

      _pitch_Kp = kStartKp;
      _pitch_Kd = kStartKd;

      rcl_interfaces::msg::Parameter kp;
      kp.name               = _cf_name + ".params.pid_rate.pitch_kp";
      kp.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      kp.value.double_value = _pitch_Kp;

      rcl_interfaces::msg::Parameter kd;
      kd.name               = _cf_name + ".params.pid_rate.pitch_kd";
      kd.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      kd.value.double_value = _pitch_Kd;

      rcl_interfaces::msg::Parameter ki;
      ki.name               = _cf_name + ".params.pid_rate.pitch_ki";
      ki.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      ki.value.double_value = 0.0;

      _set_param_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      _set_param_req->parameters = std::vector<rcl_interfaces::msg::Parameter>({kp, kd, ki});
      _set_param_result = _set_param_client->async_send_request(_set_param_req).share();

      _cmd_timer = this->create_wall_timer(1000ms, [this](void) { this->control_func(); });
      _setup_timer->cancel();

      // --------------------------- //
      // Optional setup actions here //

      // --------------------------- //
    } else {
      RCLCPP_WARN(rclcpp::get_logger("pid_tuner"), "Set Parameter Service is NOT ready.");
    }
  }

  void control_func(void)
  {
    if(_set_param_result.valid()) {
      if(_set_param_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        _set_param_resp = _set_param_result.get();
        std::vector<rcl_interfaces::msg::SetParametersResult> results =  _set_param_resp->results;

        // Reset the future so that valid will return false
        _set_param_result = rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture();
      }
    }

    // If call has finished, update gains and set parameters again
    if(!_set_param_result.valid() && _vel_cmd_active) {
      // --------------------------- //
      //    Your logic goes here     //

    
      // Example: Setting proportional and derivative gains
      _pitch_Kp = 0.0;
      _pitch_Kd = 0.0;

      // ---------------------------  //

      // Call to set parameters service
      _set_param_req->parameters[0].value.double_value = _pitch_Kp;
      _set_param_req->parameters[1].value.double_value = _pitch_Kd;

      _set_param_result = _set_param_client->async_send_request(_set_param_req).share();
    }
  }

  void gyro_callback(const crazyflie_interfaces::msg::LogDataGeneric &msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Ang Velocity X: %.2f deg/s", msg.values[0]);
    RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Ang Velocity Y: %.2f deg/s", msg.values[1]);
    RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Ang Velocity Z: %.2f deg/s", msg.values[2]);


    // ------------------------------------------ //
    //    Store angular velocity data [deg/s]     //

    // ------------------------------------------ //
  }

  void accel_callback(const crazyflie_interfaces::msg::LogDataGeneric &msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Accel X: %.2f g", msg.values[0]);
    RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Accel Y: %.2f g", msg.values[1]);
    RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Accel Z: %.2f g", msg.values[2]);

    // ------------------------------------------ //
    //          Store accleration data [g]        //
    

    // ------------------------------------------ //
  }

  void vel_callback(const geometry_msgs::msg::Twist &msg) 
  {
    _vel_cmd_active = msg.linear.z > 0.0;
  }


  // Variables used by the Node Object //

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _vel_sub;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr _gyro_sub;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr _accel_sub;
  rclcpp::TimerBase::SharedPtr _cmd_timer;
  rclcpp::TimerBase::SharedPtr _setup_timer;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr    _set_param_client;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture _set_param_result;
  std::shared_ptr<rcl_interfaces::srv::SetParameters::Request>     _set_param_req;
  std::shared_ptr<rcl_interfaces::srv::SetParameters::Response>    _set_param_resp;

  std::string _cf_name;

  bool _vel_cmd_active;

  double _pitch_Kp;
  double _pitch_Kd;

  static constexpr double kStartKp = 0.0;
  static constexpr double kStartKd = 0.0;

  // ------------------------------------- //
  // Optional: Add variables you want here //

  // ------------------------------------- //
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidTunerNode>("cf80"));
  rclcpp::shutdown();

  return 0;
}