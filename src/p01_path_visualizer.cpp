#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class P01PathVisualizer : public rclcpp::Node
{
    using Point = geometry_msgs::msg::PointStamped;
    using Path = nav_msgs::msg::Path;

public:
    P01PathVisualizer() : Node("p01_path_visualizer")
    {
        // Giữ bản tin đường bay mới nhất để RViz mở sau vẫn nhận được.
        const auto qos = rclcpp::QoS(1).reliable().transient_local();

        actual_pub_ = create_publisher<Path>("/p01/actual_path", qos);
        reference_pub_ = create_publisher<Path>("/p01/reference_path", qos);

        actual_sub_ = create_subscription<Point>(
            "/p01/position_ned", 10,
            [this](Point::ConstSharedPtr msg)
            {
                append(*msg, actual_path_, actual_pub_);
            });

        reference_sub_ = create_subscription<Point>(
            "/p01/reference_ned", 10,
            [this](Point::ConstSharedPtr msg)
            {
                append(*msg, reference_path_, reference_pub_);
            });
    }

private:
    void append(
        const Point &msg,
        Path &path,
        const rclcpp::Publisher<Path>::SharedPtr &publisher)
    {
        // Chỉ xử lý đúng hệ tọa độ mà telemetry đang cung cấp.
        if (msg.header.frame_id != "map_ned")
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Bo qua diem: frame_id phai la map_ned.");
            return;
        }

        if (!std::isfinite(msg.point.x) ||
            !std::isfinite(msg.point.y) ||
            !std::isfinite(msg.point.z))
        {
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.header.frame_id = "map";

        // NED: Bắc, Đông, Xuống → ENU: Đông, Bắc, Lên.
        pose.pose.position.x = msg.point.y;
        pose.pose.position.y = msg.point.x;
        pose.pose.position.z = -msg.point.z;

        // Giá trị phụ để vẽ đường; không biểu diễn tư thế thật của UAV.
        pose.pose.orientation.w = 1.0;

        path.header = pose.header;
        path.poses.push_back(pose);

        // Giữ tối đa 3000 điểm gần nhất, khoảng 5 phút ở 10 Hz.
        if (path.poses.size() > 3000)
        {
            path.poses.erase(path.poses.begin());
        }

        publisher->publish(path);
    }

    Path actual_path_;
    Path reference_path_;

    rclcpp::Publisher<Path>::SharedPtr actual_pub_;
    rclcpp::Publisher<Path>::SharedPtr reference_pub_;
    rclcpp::Subscription<Point>::SharedPtr actual_sub_;
    rclcpp::Subscription<Point>::SharedPtr reference_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<P01PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}