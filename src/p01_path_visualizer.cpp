// NODE HIỂN THỊ CHO RVIZ: hai đường bay + mô hình UAV.
//   /p01/position_ned  → /p01/actual_path      (đường bay thực, nav_msgs/Path)
//   /p01/reference_ned → /p01/reference_path   (đường tham chiếu, nav_msgs/Path)
//   /p01/pose_ned      → /p01/uav_marker       (mô hình X500 từ mesh PX4, visualization_msgs/MarkerArray)
// Mọi đầu ra ở frame "map" theo quy ước RViz: ENU (Đông, Bắc, Lên), thân FLU (trước, trái, lên).
// Node chỉ đổi hệ tọa độ để vẽ; không tính điều khiển và không gửi lệnh tới PX4.
#include <array>
#include <cmath>
#include <cstdlib>
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class P01PathVisualizer : public rclcpp::Node
{
    using Point = geometry_msgs::msg::PointStamped;
    using Pose = geometry_msgs::msg::PoseStamped;
    using Path = nav_msgs::msg::Path;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
    P01PathVisualizer() : Node("p01_path_visualizer")
    {
        // Giữ bản tin đường bay mới nhất để RViz mở sau vẫn nhận được.
        const auto qos = rclcpp::QoS(1).reliable().transient_local();

        actual_pub_ = create_publisher<Path>("/p01/actual_path", qos);
        reference_pub_ = create_publisher<Path>("/p01/reference_path", qos);
        marker_pub_ = create_publisher<MarkerArray>("/p01/uav_marker", 10);
        // Thư mục mesh X500 của PX4; đổi bằng -p mesh_dir:=... nếu PX4-Autopilot nằm chỗ khác.
        const char *home = std::getenv("HOME");
        mesh_dir_ = declare_parameter<std::string>(
            "mesh_dir", std::string(home ? home : "") + "/PX4-Autopilot/Tools/simulation/gz/models/x500_base/meshes");

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

        pose_sub_ = create_subscription<Pose>(
            "/p01/pose_ned", 10,
            [this](Pose::ConstSharedPtr msg) { draw_uav(*msg); });
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

    // Vẽ X500 bằng chính mesh của PX4 (Tools/simulation/gz/models/x500_base/meshes), đúng vị trí và tư thế.
    // Vị trí/góc từng chi tiết lấy từ x500_base/model.sdf (hệ thân FLU, cùng quy ước RViz).
    // Hai cánh quạt trước (x > 0) tô đỏ để nhận hướng mũi.
    void draw_uav(const Pose &msg)
    {
        if (msg.header.frame_id != "map_ned") return;

        // Giới hạn ~30 Hz: đủ mượt cho mắt, đỡ tải RViz (telemetry phát ~100 Hz).
        const rclcpp::Time now = msg.header.stamp;
        if (last_draw_.nanoseconds() != 0 && (now - last_draw_).seconds() < 0.033) return;
        last_draw_ = now;

        const Eigen::Vector3d p_ned(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        const Eigen::Quaterniond q_ned(msg.pose.orientation.w, msg.pose.orientation.x,
                                       msg.pose.orientation.y, msg.pose.orientation.z);
        if (!p_ned.allFinite() || !std::isfinite(q_ned.norm()) || q_ned.norm() < 0.5) return;

        // Đổi hệ: T đổi NED→ENU (đổi x,y; đảo z); D đổi FLU→FRD (đảo y,z).
        // R_enu_flu = T · R_ned_frd · D.
        Eigen::Matrix3d T;
        T << 0, 1, 0,
             1, 0, 0,
             0, 0, -1;
        const Eigen::Matrix3d D = Eigen::Vector3d(1, -1, -1).asDiagonal();
        const Eigen::Matrix3d R = T * q_ned.normalized().toRotationMatrix() * D;
        const Eigen::Vector3d p = T * p_ned;

        MarkerArray arr;
        int id = 0;
        // Thêm một mesh: offset/góc quay quanh z trong hệ thân FLU → đổi sang map.
        // rgba[3] = 0 nghĩa là dùng màu có sẵn trong tệp .dae.
        auto add = [&](const std::string &file, const Eigen::Vector3d &offset, double yaw_local,
                       double scale, const std::array<float, 4> &rgba)
        {
            Marker m;
            m.header.frame_id = "map";
            m.header.stamp = msg.header.stamp;
            m.ns = "uav";
            m.id = id++;
            m.type = Marker::MESH_RESOURCE;
            m.action = Marker::ADD;
            m.mesh_resource = "file://" + mesh_dir_ + "/" + file;
            const Eigen::Vector3d pos = p + R * offset;
            const Eigen::Quaterniond q(R * Eigen::AngleAxisd(yaw_local, Eigen::Vector3d::UnitZ()));
            m.pose.position.x = pos.x();
            m.pose.position.y = pos.y();
            m.pose.position.z = pos.z();
            m.pose.orientation.w = q.w();
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();
            m.scale.x = m.scale.y = m.scale.z = scale;
            if (rgba[3] > 0.0f)
            {
                m.color.r = rgba[0];
                m.color.g = rgba[1];
                m.color.b = rgba[2];
                m.color.a = rgba[3];
            }
            else
            {
                m.mesh_use_embedded_materials = true;
                m.color.a = 1.0f;   // RViz vẫn cần alpha > 0 để hiện mesh
            }
            arr.markers.push_back(m);
        };

        const std::array<float, 4> embedded{0, 0, 0, 0};
        const std::array<float, 4> prop_front{0.85f, 0.15f, 0.15f, 0.85f};
        const std::array<float, 4> prop_back{0.2f, 0.2f, 0.2f, 0.85f};

        // Khung carbon: pose 0 0 0.025, quay 180° quanh z.
        add("NXP-HGD-CF.dae", Eigen::Vector3d(0, 0, 0.025), M_PI, 1.0, embedded);

        // Bốn rotor (FLU): vị trí tâm rotor_i, chiều quay cánh theo model.sdf.
        const double a = 0.174;
        const std::array<Eigen::Vector3d, 4> c{Eigen::Vector3d(a, -a, 0.06), Eigen::Vector3d(-a, a, 0.06),
                                               Eigen::Vector3d(a, a, 0.06), Eigen::Vector3d(-a, -a, 0.06)};
        const std::array<const char *, 4> prop{"1345_prop_ccw.stl", "1345_prop_ccw.stl",
                                               "1345_prop_cw.stl", "1345_prop_cw.stl"};
        const double prop_scale = 0.8461538461538461;
        for (int i = 0; i < 4; ++i)
        {
            add("5010Base.dae", Eigen::Vector3d(c[i].x(), c[i].y(), 0.032), -0.45, 1.0, embedded);   // đế motor
            add("5010Bell.dae", c[i] + Eigen::Vector3d(0, 0, -0.032), 0.0, 1.0, embedded);          // chuông motor
            add(prop[i], c[i] + Eigen::Vector3d(-0.022, -0.14638461538461536, -0.016), 0.0, prop_scale,
                c[i].x() > 0 ? prop_front : prop_back);                                              // cánh quạt
        }

        marker_pub_->publish(arr);
    }

    std::string mesh_dir_;
    Path actual_path_;
    Path reference_path_;
    rclcpp::Time last_draw_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<Path>::SharedPtr actual_pub_;
    rclcpp::Publisher<Path>::SharedPtr reference_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<Point>::SharedPtr actual_sub_;
    rclcpp::Subscription<Point>::SharedPtr reference_sub_;
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<P01PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}
