#include <gap_detection/gap_detection.h>

#include <vector>

#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

GapDetector::GapDetector() : Node("gap_detector")
{
    using namespace std::chrono_literals;

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "TTB06/scan",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
        [this](const sensor_msgs::msg::LaserScan &msg)
        { this->laser_callback(msg); });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "TTB06/odom",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
        [this](const nav_msgs::msg::Odometry &msg)
        { this->odom_callback(msg); });

    gap_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("TTB06/gaps", 10);
    marker_arr_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("TTB06/gap_viz", 10);

    timer_ = this->create_wall_timer(
        100ms, // Period of rate that function is called
        [this](void)
        { this->find_gaps(); } // Which function to call
    );

    odom_init_ = false;
    laser_init_ = false;

    // parameters
    this->declare_parameter<double>("vehicle_width", .2);
    this->declare_parameter<double>("detection_threshold", .3);
    this->declare_parameter<double>("min_gap_size", .2);

    this->get_parameter("vehicle_width", vehicle_width_);
    this->get_parameter("detection_threshold", threshold_);
    this->get_parameter("min_gap_size", gap_thresh_);

    RCLCPP_INFO(this->get_logger(), "vehicle_width: '%.2f'", vehicle_width_);
    RCLCPP_INFO(this->get_logger(), "detection_threshold: '%.2f'", threshold_);
    RCLCPP_INFO(this->get_logger(), "min_gap_size: '%.2f'", gap_thresh_);
}

void GapDetector::odom_callback(const nav_msgs::msg::Odometry &msg)
{

    odom_init_ = true;
    tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    odom_ = Eigen::Vector3d(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        yaw + M_PI / 2);
}

void GapDetector::laser_callback(const sensor_msgs::msg::LaserScan &msg)
{
    laser_init_ = true;
    laser_msg_ = msg;
}

void GapDetector::find_gaps(void)
{
    // if either callback has not been executed yet, cannot find gaps
    if (!odom_init_ || !laser_init_)
        return;

    std::vector<Eigen::Vector4d> gaps;

    // xy robot position
    Eigen::Vector2d odom2 = Eigen::Vector2d(odom_(0), odom_(1));

    // scan counter clockwise looking for rising jumps
    for (unsigned int i = 0; i < laser_msg_.ranges.size() - 1;)
    {
        double a1 = laser_msg_.angle_min + i * laser_msg_.angle_increment + odom_(2);
        double a2 = laser_msg_.angle_min + (i + 1) * laser_msg_.angle_increment + odom_(2);

        Eigen::Vector2d p1(laser_msg_.ranges[i] * cos(a1), laser_msg_.ranges[i] * sin(a1));
        Eigen::Vector2d p2(laser_msg_.ranges[i + 1] * cos(a2), laser_msg_.ranges[i + 1] * sin(a2));

        p1 += odom2;
        p2 += odom2;

        // if gap jump isn't large enough or ranges[i] > ranges[i+1], continue
        if ((p1 - p2).norm() < threshold_ ||
            laser_msg_.ranges[i] > laser_msg_.ranges[i + 1] ||
            std::isinf(laser_msg_.ranges[i]) ||
            laser_msg_.ranges[i] < laser_msg_.range_min)
        {
            ++i;
            continue;
        }

        int champ_ind = i + 1;
        double champ_dist = 10000;
        bool found = false;

        for (unsigned int j = i + 1; j < laser_msg_.ranges.size(); ++j)
        {
            if (laser_msg_.ranges[j] < laser_msg_.range_min || laser_msg_.ranges[j] > laser_msg_.range_max)
                continue;

            double angle = laser_msg_.angle_min + j * laser_msg_.angle_increment + odom_(2);

            double deltaA = angle - a1;

            if (deltaA > 2 * M_PI)
                deltaA -= 2 * M_PI;
            else if (deltaA < -2 * M_PI)
                deltaA += 2 * M_PI;

            if (fabs(deltaA) > M_PI)
                break;

            Eigen::Vector2d pk(laser_msg_.ranges[j] * cos(angle), laser_msg_.ranges[j] * sin(angle));
            pk += odom2;

            double d = (p1 - pk).squaredNorm();
            if (d < champ_dist)
            {
                found = true;
                champ_dist = d;
                champ_ind = j;
            }
        }

        if (found)
        {
            double angle2 = laser_msg_.angle_min + champ_ind * laser_msg_.angle_increment + odom_(2);
            Eigen::Vector2d pe(laser_msg_.ranges[champ_ind] * cos(angle2), laser_msg_.ranges[champ_ind] * sin(angle2));
            pe += odom2;

            Eigen::Vector2d mp = (p1 + pe) / 2;

            if (!is_gap_near_scan(mp) &&
                !std::isinf((p1 - pe).norm()) &&
                (p1 - pe).norm() >= gap_thresh_)
            {
                Eigen::Vector4d gap(p1.x(), p1.y(), pe.x(), pe.y());
                gaps.push_back(gap);
            }
        }

        ++i;
    }

    // now go clockwise direction for descending jumps
    for (int i = laser_msg_.ranges.size() - 1; i > 0;)
    {
        double a1 = laser_msg_.angle_min + i * laser_msg_.angle_increment + odom_(2);
        double a2 = laser_msg_.angle_min + (i - 1) * laser_msg_.angle_increment + odom_(2);

        Eigen::Vector2d p1(laser_msg_.ranges[i] * cos(a1), laser_msg_.ranges[i] * sin(a1));
        Eigen::Vector2d p2(laser_msg_.ranges[i - 1] * cos(a2), laser_msg_.ranges[i - 1] * sin(a2));

        p1 += odom2;
        p2 += odom2;

        if ((p1 - p2).norm() < threshold_ ||
            laser_msg_.ranges[i] > laser_msg_.ranges[i - 1] ||
            std::isinf(laser_msg_.ranges[i]) ||
            laser_msg_.ranges[i] < laser_msg_.range_min)
        {
            --i;
            continue;
        }

        int champ_ind = i - 1;
        double champ_dist = 10000;
        bool found = false;

        for (int j = i - 1; j >= 0; --j)
        {
            if (laser_msg_.ranges[j] < laser_msg_.range_min || laser_msg_.ranges[j] > laser_msg_.range_max)
                continue;

            double angle = laser_msg_.angle_min + j * laser_msg_.angle_increment + odom_(2);

            double deltaA = angle - a1;

            if (deltaA > 2 * M_PI)
                deltaA -= 2 * M_PI;
            else if (deltaA < -2 * M_PI)
                deltaA += 2 * M_PI;

            if (fabs(deltaA) > M_PI)
                break;

            Eigen::Vector2d pk(laser_msg_.ranges[j] * cos(angle), laser_msg_.ranges[j] * sin(angle));
            pk += odom2;

            double d = (p1 - pk).squaredNorm();
            if (d < champ_dist)
            {
                found = true;
                champ_dist = d;
                champ_ind = j;
            }
        }

        if (found)
        {
            double angle2 = laser_msg_.angle_min + champ_ind * laser_msg_.angle_increment + odom_(2);
            Eigen::Vector2d pe(laser_msg_.ranges[champ_ind] * cos(angle2), laser_msg_.ranges[champ_ind] * sin(angle2));
            pe += odom2;

            Eigen::Vector2d mp = (p2 + pe) / 2;

            if (!is_gap_near_scan(mp) &&
                !std::isinf((p2 - pe).norm()) &&
                (p2 - pe).norm() >= gap_thresh_)
            {
                Eigen::Vector4d gap(p2.x(), p2.y(), pe.x(), pe.y());
                gaps.push_back(gap);
            }
        }

        --i;
    }

    publish_gaps(gaps);
    visualize_gaps(gaps);
    // lidar_debug();
}

int GapDetector::get_indices_from_point(const Eigen::Vector2d &point)
{
    double err = atan2(point.y() - odom_.y(), point.x() - odom_.x());
    if (err < 0)
        err += 2 * M_PI;

    err -= odom_.z();
    return (int)(err / laser_msg_.angle_increment + laser_msg_.ranges.size() / 2);
}

bool GapDetector::is_gap_near_scan(const Eigen::Vector2d &gap)
{
    int ind = get_indices_from_point(gap);

    for (int i = ind - 10; i < ind + 11 && i < laser_msg_.ranges.size(); i++)
    {
        if (ind < 0)
            continue;

        double angle = laser_msg_.angle_min + i * laser_msg_.angle_increment + odom_(2);
        Eigen::Vector2d p(laser_msg_.ranges[i] * cos(angle), laser_msg_.ranges[i] * sin(angle));
        p += Eigen::Vector2d(odom_(0), odom_(1));

        if ((gap - p).squaredNorm() < vehicle_width_ * vehicle_width_ / 6)
            return true;
    }

    return false;
}

void GapDetector::publish_gaps(const std::vector<Eigen::Vector4d> &gaps)
{
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";

    for(size_t i = 0; i < gaps.size(); ++i){
        Eigen::Vector4d gap = gaps[i];

        geometry_msgs::msg::Pose p1;
        p1.position.x = gap(0);
        p1.position.y = gap(1);
        p1.position.z = 0;
        p1.orientation.w = 1;
        msg.poses.push_back(p1);

        geometry_msgs::msg::Pose p2;
        p2.position.x = gap(2);
        p2.position.y = gap(3);
        p2.position.z = 0;
        p2.orientation.w = 1;
        msg.poses.push_back(p2);
    }

    gap_pub_->publish(msg);
}

void GapDetector::visualize_gaps(const std::vector<Eigen::Vector4d> &gaps)
{

    visualization_msgs::msg::Marker msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.ns = "gaps";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    msg.scale.x = .2;
    msg.color.r = 243.0 / 255.0;
    msg.color.g = 167.0 / 255.0;
    msg.color.b = 18.0 / 255.0;
    msg.color.a = 1.0;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;

    for (size_t i = 0; i < gaps.size(); ++i)
    {
        geometry_msgs::msg::Point p1;
        p1.x = gaps[i](0);
        p1.y = gaps[i](1);
        p1.z = 0;
        msg.points.push_back(p1);

        geometry_msgs::msg::Point p2;
        p2.x = gaps[i](2);
        p2.y = gaps[i](3);
        p2.z = 0;
        msg.points.push_back(p2);
    }

    marker_arr_pub_->publish(msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapDetector>());
    rclcpp::shutdown();

    return 0;
}
