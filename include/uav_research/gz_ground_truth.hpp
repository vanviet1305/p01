#pragma once
// TRẠNG THÁI THẬT TỪ GAZEBO (chỉ dùng trong mô phỏng).
// Đọc /model/x500_0/odometry (gz.msgs.Odometry, ~250 Hz) do plugin OdometryPublisher phát.
// Plugin được gắn lúc chạy bằng tools/gan_odom_gazebo.sh (không sửa tệp PX4).
// Lý do (21/11/2026): EKF2 của PX4 ước lượng sai roll 1–4° khi bay gắt và trôi dần qua các lần bay,
// làm lẫn lỗi ước lượng vào so sánh P01/P02. Dùng trạng thái thật để tách lỗi điều khiển khỏi lỗi ước lượng.
//
// Chỉ dùng phần pose (vị trí + quaternion), là phần không mơ hồ về hệ trục:
//   - vị trí trong hệ odom của Gazebo (ENU, gốc tại vị trí đặt model lúc đầu),
//   - quaternion quay hệ thân FLU sang ENU.
// Vận tốc = vi phân vị trí theo thời gian mô phỏng trong header (vị trí thật không nhiễu nên vi phân sạch).
// Không dùng twist của bản tin vì chưa xác nhận được nó biểu diễn trong hệ thân hay hệ thế giới.
// Vận tốc góc KHÔNG lấy ở đây: node giữ vận tốc góc từ con quay của PX4 (không bị lỗi EKF2).
#include <Eigen/Dense>
#include <chrono>
#include <mutex>
#include <string>

#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>

namespace uav_research
{
class GzGroundTruth
{
public:
    // Đăng ký topic. Trả về false nếu gz-transport không đăng ký được.
    bool start(const std::string &topic)
    {
        return node_.Subscribe(topic, &GzGroundTruth::on_odom, this);
    }

    // Lấy trạng thái thật đã đổi sang NED / FRD (cùng quy ước PX4 và P01State).
    // Trả về false nếu chưa có đủ 2 bản tin (chưa tính được vận tốc).
    // age_s: thời gian (đồng hồ thật) kể từ bản tin cuối, để node cảnh báo khi mất dữ liệu.
    bool get(Eigen::Vector3d &x_ned, Eigen::Vector3d &v_ned, Eigen::Matrix3d &R_ned_frd, double &age_s) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_vel_) return false;
        // T đổi ENU→NED (đổi x,y; đảo z); D đổi FRD↔FLU (đảo y,z). R_ned_frd = T · R_enu_flu · D.
        Eigen::Matrix3d T;
        T << 0, 1, 0,
             1, 0, 0,
             0, 0, -1;
        const Eigen::Matrix3d D = Eigen::Vector3d(1, -1, -1).asDiagonal();
        x_ned = T * p_enu_;
        v_ned = T * v_enu_;
        R_ned_frd = T * q_enu_flu_.toRotationMatrix() * D;
        age_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_rx_).count();
        return true;
    }

private:
    void on_odom(const gz::msgs::Odometry &m)
    {
        const double t = m.header().stamp().sec() + 1e-9 * m.header().stamp().nsec();
        const Eigen::Vector3d p(m.pose().position().x(), m.pose().position().y(), m.pose().position().z());
        const Eigen::Quaterniond q(m.pose().orientation().w(), m.pose().orientation().x(),
                                   m.pose().orientation().y(), m.pose().orientation().z());
        std::lock_guard<std::mutex> lock(mutex_);
        // gz-transport đôi khi dồn vài bản tin liền nhau; bỏ bản tin trùng hoặc lùi thời gian.
        if (have_pose_ && t <= t_)
            return;
        if (have_pose_)
        {
            v_enu_ = (p - p_enu_) / (t - t_);
            have_vel_ = true;
        }
        p_enu_ = p;
        q_enu_flu_ = q.normalized();
        t_ = t;
        have_pose_ = true;
        last_rx_ = std::chrono::steady_clock::now();
    }

    gz::transport::Node node_;
    mutable std::mutex mutex_;
    Eigen::Vector3d p_enu_ = Eigen::Vector3d::Zero(), v_enu_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_enu_flu_ = Eigen::Quaterniond::Identity();
    double t_ = 0.0;
    bool have_pose_ = false, have_vel_ = false;
    std::chrono::steady_clock::time_point last_rx_{};
};
}  // namespace uav_research
