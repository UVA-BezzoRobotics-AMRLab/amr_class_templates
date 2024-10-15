#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class GapReader : public rclcpp::Node
{
public:
    GapReader() : Node("gap_reader")
    {
        using namespace std::chrono_literals;

        gap_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "TTB06/gaps",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
            [this](const geometry_msgs::msg::PoseArray &msg)
            { this->gap_callback(msg); });
    }

private:
    void gap_callback(const geometry_msgs::msg::PoseArray &msg)
    {
        std::vector<geometry_msgs::msg::Pose> gap_list = msg.poses;

        for (size_t i = 0; i < gap_list.size() - 1; i += 2)
        {
            geometry_msgs::msg::Pose p1 = gap_list[i];
            geometry_msgs::msg::Pose p2 = gap_list[i + 1];

            std::cout << "gap " << i / 2 << ":\n\tstart: (" << p1.position.x << ", " << p1.position.y << ")\n";
            std::cout << "\tend: (" << p2.position.x << ", " << p2.position.y << ")\n";
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gap_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapReader>());
    rclcpp::shutdown();

    return 0;
}
