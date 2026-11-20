#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "uav_research/p01_geometric_controller.hpp"

namespace uav_research
{

// Phát dữ liệu quan sát từ cùng một mẫu tính P01.
// Không tính điều khiển và không gửi lệnh tới PX4.
class P01Telemetry
{
    using Point = geometry_msgs::msg::PointStamped;
    using Pose = geometry_msgs::msg::PoseStamped;
    using Vector = geometry_msgs::msg::Vector3Stamped;
    using Scalar = std_msgs::msg::Float64;

public:
    explicit P01Telemetry(rclcpp::Node &node)
    {
        position_pub_ = node.create_publisher<Point>(
            "/p01/position_ned", 10);
        reference_pub_ = node.create_publisher<Point>(
            "/p01/reference_ned", 10);
        error_pub_ = node.create_publisher<Vector>(
            "/p01/error_ned", 10);
        norm_pub_ = node.create_publisher<Scalar>(
            "/p01/error_norm", 10);
        // Vị trí + tư thế UAV (NED, thân FRD) để RViz vẽ mô hình UAV; xem p01_path_visualizer.
        pose_pub_ = node.create_publisher<Pose>(
            "/p01/pose_ned", 10);
    }

    void publish(
        std::uint64_t sample,
        const rclcpp::Time &stamp,
        const P01State &state,
        const P01Reference &reference,
        const P01Control &control)
    {
        // Tư thế UAV phát ở mọi lần gọi (~100 Hz) để mô hình trong RViz chuyển động mượt.
        // R đổi vectơ thân FRD sang hệ NED; quaternion lấy trực tiếp từ R.
        Pose pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map_ned";
        pose.pose.position.x = state.x.x();
        pose.pose.position.y = state.x.y();
        pose.pose.position.z = state.x.z();
        const Eigen::Quaterniond q(state.R);
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose_pub_->publish(pose);

        // Giới hạn dữ liệu hiển thị khoảng 10 Hz theo thời gian mẫu PX4.
        // Vòng tính điều khiển vẫn chạy theo mọi mẫu odometry hợp lệ.
        if (last_sample_ != 0 &&
            sample >= last_sample_ &&
            sample - last_sample_ < 100000)
        {
            return;
        }
        last_sample_ = sample;

        Point position;
        position.header.stamp = stamp;
        position.header.frame_id = "map_ned";
        position.point.x = state.x.x();
        position.point.y = state.x.y();
        position.point.z = state.x.z();

        Point target;
        target.header = position.header;
        target.point.x = reference.x.x();
        target.point.y = reference.x.y();
        target.point.z = reference.x.z();

        Vector error;
        error.header = position.header;
        error.vector.x = control.ex.x();
        error.vector.y = control.ex.y();
        error.vector.z = control.ex.z();

        Scalar norm;
        norm.data = control.ex.norm();

        position_pub_->publish(position);
        reference_pub_->publish(target);
        error_pub_->publish(error);
        norm_pub_->publish(norm);
    }

private:
    rclcpp::Publisher<Point>::SharedPtr position_pub_;
    rclcpp::Publisher<Point>::SharedPtr reference_pub_;
    rclcpp::Publisher<Vector>::SharedPtr error_pub_;
    rclcpp::Publisher<Scalar>::SharedPtr norm_pub_;
    rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
    std::uint64_t last_sample_ = 0;
};

}  // namespace uav_research