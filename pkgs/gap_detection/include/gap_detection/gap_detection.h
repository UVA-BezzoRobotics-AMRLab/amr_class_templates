#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>

#include <chrono>

class GapDetector : public rclcpp::Node
{
public:
    GapDetector();

private:
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    void laser_callback(const sensor_msgs::msg::LaserScan &msg);

    void find_gaps(void);

    void publish_gaps(const std::vector<Eigen::Vector4d> &gaps);
    void visualize_gaps(const std::vector<Eigen::Vector4d> &gaps);

    bool is_gap_near_scan(const Eigen::Vector2d &gap);
    int get_indices_from_point(const Eigen::Vector2d &point);

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr gap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_arr_pub_;

    bool odom_init_;
    bool laser_init_;

    double threshold_;
    double gap_thresh_;
    double vehicle_width_;

    Eigen::Vector3d odom_;

    sensor_msgs::msg::LaserScan laser_msg_;
};
