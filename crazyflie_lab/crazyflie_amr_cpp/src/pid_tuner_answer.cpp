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

    _data_sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/" + cf_name + "/gyro_data", qos_profile,
      [this](const crazyflie_interfaces::msg::LogDataGeneric &msg) { this->data_callback(msg); });

    _setup_timer = this->create_wall_timer(500ms, [this](void) { this->setup(); });

    _set_param_client = this->create_client<rcl_interfaces::srv::SetParameters>("/crazyflie_server/set_parameters");
  }

private:

  void setup(void) 
  {
    if(_set_param_client->service_is_ready()) {
      RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Set Parameter Service is ready.");

      _pitch_Kp = kStartKp;
      _pitch_Ki = kStartKi;
      _pitch_Kd = kStartKd;

      _stable_1  = false;
      _stable_2  = false;
      _oscillate = false;

      rcl_interfaces::msg::Parameter kp;
      kp.name               = _cf_name + ".params.pid_rate.pitch_kp";
      kp.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      kp.value.double_value = _pitch_Kp;

      rcl_interfaces::msg::Parameter ki;
      ki.name               = _cf_name + ".params.pid_rate.pitch_ki";
      ki.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      ki.value.double_value = _pitch_Ki;

      rcl_interfaces::msg::Parameter kd;
      kd.name               = _cf_name + ".params.pid_rate.pitch_kd";
      kd.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      kd.value.double_value = _pitch_Kd;

      _set_param_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      _set_param_req->parameters = std::vector<rcl_interfaces::msg::Parameter>({kp, ki, kd});
      _set_param_result = _set_param_client->async_send_request(_set_param_req).share();

      _cmd_timer = this->create_wall_timer(1000ms, [this](void) { this->control_func(); });
      _setup_timer->cancel();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("pid_tuner"), "Set Parameter Service is NOT ready.");
    }
  }

  void control_func(void)
  {
    if(_ang_vel_received && _vel_cmd_active) {
      
      if(_set_param_result.valid()) {
        if(_set_param_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          _set_param_resp = _set_param_result.get();
          std::vector<rcl_interfaces::msg::SetParametersResult> results =  _set_param_resp->results;

          // Reset the future so that valid will return false
          _set_param_result = rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture();
        }
      }

      // If call has finished, update gains and set parameters again
      if(!_set_param_result.valid()) {

        // Standard deviation of data
        std::vector<double> stdev;
        stdev.push_back(stdev_compute(_ang_data_x));
        stdev.push_back(stdev_compute(_ang_data_y));
        stdev.push_back(stdev_compute(_ang_data_y));

        // Min/Max standard deviation of data
        auto minmax_stdev = std::minmax_element(stdev.begin(), stdev.end());
        double min_stdev = *minmax_stdev.first;
        double max_stdev = *minmax_stdev.second;

        RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), std::string("Min Stdev: " + std::to_string(min_stdev)).c_str()  );
        RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), std::string("Max Stdev: " + std::to_string(max_stdev)).c_str()  );

        // Determine if UAV has switched from statble to oscillating to back to stable
        if ((!_stable_1) && (min_stdev < 5.0) && (max_stdev < 10.0)) {
          RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Stablized: First");
          _stable_1 = true;
        } else if (_stable_1 && (max_stdev > 65.0)) {
          RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Oscillating");
          _oscillate = true;
        } else if (_stable_1 && _oscillate && (min_stdev < 5.0) && (max_stdev < 10.0)) {
          RCLCPP_INFO(rclcpp::get_logger("pid_tuner"), "Crazyflie has Stabilized!");
          _stable_2 = true;
        }

        // Adjust gains if necessary
        if ((!_stable_1) || (!_oscillate)) {
          _pitch_Kp += 20.0; //20.0;
        } else if (_stable_1 && _oscillate && (!_stable_2)) {
          _pitch_Kd += 0.2; //0.2;
        }

        // Call to set parameters service
        _set_param_req->parameters[0].value.double_value = _pitch_Kp;
        _set_param_req->parameters[1].value.double_value = _pitch_Ki;
        _set_param_req->parameters[2].value.double_value = _pitch_Kd;

        _set_param_result = _set_param_client->async_send_request(_set_param_req).share();
      }

      _ang_vel_received = false;
    }
  }

  void data_callback(const crazyflie_interfaces::msg::LogDataGeneric &msg)
  {
    _ang_data_x.push_back(msg.values[0]);
    _ang_data_y.push_back(msg.values[1]);
    _ang_data_z.push_back(msg.values[2]);

    if(_ang_data_x.size() > kMaxDataLength) {
      _ang_data_x.erase(_ang_data_x.begin());
      _ang_data_y.erase(_ang_data_y.begin());
      _ang_data_z.erase(_ang_data_z.begin());

      _ang_vel_received = true;
    }
  }

  void vel_callback(const geometry_msgs::msg::Twist &msg) 
  {
    _vel_cmd_active = msg.linear.z > 0.0;
  }

  double stdev_compute(const std::vector<double> &data)
  {
    // Number of elements
    size_t size = data.size();

    // Calculate the sum of elements in the vector
    double mean = std::accumulate(data.begin(), data.end(), 0.0) / size;

    // Calculate the sum of squared differences from the mean
    double stdev = 0.0;
    for (int i = 0; i < size; ++i) {
        stdev += pow(data[i] - mean, 2);
    }

    // Calculate the square root of the variance
    return sqrt(stdev / size);
  }

  // ---------------------------------//
  // Variable used by the Node Object //
  // ---------------------------------//

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _vel_sub;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr _data_sub;
  rclcpp::TimerBase::SharedPtr _cmd_timer;
  rclcpp::TimerBase::SharedPtr _setup_timer;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr    _set_param_client;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture _set_param_result;
  std::shared_ptr<rcl_interfaces::srv::SetParameters::Request>     _set_param_req;
  std::shared_ptr<rcl_interfaces::srv::SetParameters::Response>    _set_param_resp;

  std::string _cf_name;

  bool _vel_cmd_active;
  bool _ang_vel_received;
  std::vector<double> _ang_data_x;
  std::vector<double> _ang_data_y;
  std::vector<double> _ang_data_z;

  double _pitch_Kp;
  double _pitch_Ki;
  double _pitch_Kd;

  bool _stable_1;
  bool _stable_2;
  bool _oscillate;

  static constexpr double kStartKp = 10.0;
  static constexpr double kStartKi = 0.0;
  static constexpr double kStartKd = 0.0;

  static constexpr size_t kMaxDataLength = 5;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidTunerNode>());
  rclcpp::shutdown();

  return 0;
}