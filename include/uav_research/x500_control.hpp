#pragma once
// GIAO DIỆN VỚI X500: (1) tham số UAV/hệ số điều khiển, (2) phân bổ f,M thành
// lực bốn rotor, (3) đổi lực rotor sang lệnh chuẩn hóa cho ActuatorMotors.
// Đây là phần triển khai theo mô hình X500, tách khỏi công thức của P01 CDC.
// Hàm ở đây chỉ tính toán; việc publish thực sự nằm trong p01_offboard_hover.cpp.

#include "uav_research/p01_geometric_controller.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace uav_research
{
// Bộ tham số X500 đã bay tốt trong Gazebo. Giá trị nằm sẵn trong P01Parameters
// (p01_geometric_controller.hpp); hàm này giữ lại để các node gọi như cũ.
inline P01Parameters x500_parameters()
{
    return P01Parameters{};
}

struct X500Actuators
{
    double arm = 0.174;           // Độ lệch rotor theo x/y thân FRD (m), không phải đường chéo.
    double moment_ratio = 0.016;  // Mômen phản lực/lực đẩy (m), theo mô hình động cơ Gazebo.
    double motor_constant = 8.54858e-6; // Hệ số lực: T=k*omega^2, đơn vị N/(rad/s)^2.
    double omega_min = 150.0;
    double omega_max = 1000.0;
    // Hai giới hạn tốc độ phía trên phải khớp SIM_GZ_EC_MIN/MAX đang áp dụng.
    // thrust_factor phải khớp THR_MDL_FAC trong đường đầu ra PX4, không tự đọc từ PX4.
    // Người dùng đã tra cứu: MIN1..4=150, MAX1..4=1000, THR_MDL_FAC=0.
    double thrust_factor = 0.0;

    void validate() const
    {
        if (!std::isfinite(arm) || arm <= 0 || !std::isfinite(moment_ratio) ||
            moment_ratio <= 0 || !std::isfinite(motor_constant) || motor_constant <= 0 ||
            !std::isfinite(omega_min) || omega_min < 0 || !std::isfinite(omega_max) ||
            omega_max <= omega_min || !std::isfinite(thrust_factor) ||
            thrust_factor < 0 || thrust_factor > 1)
            throw std::invalid_argument("Invalid X500 actuator parameters");
    }
    double min_thrust() const { return motor_constant * omega_min * omega_min; }
    double max_thrust() const { return motor_constant * omega_max * omega_max; }

    double command_from_thrust(double thrust) const
    {
        // Lực rotor (N) -> tốc độ cần (rad/s) -> tín hiệu đầu ra chuẩn hóa 0..1.
        // Bước cuối tính lệnh theo đường cong lực của PX4 để PX4 đổi ngược ra
        // tín hiệu đầu ra mong muốn. Lệnh này không phải số RPM hay lực tính bằng N.
        validate();
        if (!std::isfinite(thrust)) throw std::invalid_argument("Non-finite rotor thrust");
        const double speed = std::sqrt(std::clamp(thrust, min_thrust(), max_thrust()) / motor_constant);
        const double u = std::clamp((speed - omega_min) / (omega_max - omega_min), 0.0, 1.0);
        return (1.0 - thrust_factor) * u + thrust_factor * u * u;
    }
};

struct X500Allocation
{
    // thrusts/commands là kết quả TÍNH TOÁN, không phải số đo từ động cơ Gazebo.
    std::array<double, 4> thrusts{};
    std::array<double, 4> commands{};
    double achieved_f = 0;
    // achieved_f/M được dựng lại từ lực rotor sau giới hạn; chữ achieved ở CSV
    // không có nghĩa đã đo trực tiếp lực/mômen vật lý của mô hình Gazebo.
    Vector3 achieved_M = Vector3::Zero();
    double roll_pitch_scale = 1;
    bool saturated = false;
};

inline Eigen::Vector4d x500_wrench(const std::array<double, 4> &t,
                                 const X500Actuators &a = {})
{
    // Dựng lại [f,Mx,My,Mz] từ T0..T3 theo thứ tự động cơ X500 tiêu chuẩn.
    // FRD là trước-phải-xuống; các dấu mômen phụ thuộc vị trí/chiều quay rotor.
    // Không dùng nguyên dấu/thứ tự đánh rotor của sơ đồ khác để thay vào đây.
    return Eigen::Vector4d(t[0]+t[1]+t[2]+t[3],
        a.arm*(-t[0]+t[1]+t[2]-t[3]),
        a.arm*(t[0]-t[1]+t[2]-t[3]),
        a.moment_ratio*(t[0]+t[1]-t[2]-t[3]));
}

inline X500Allocation allocate_x500(double f, const Vector3 &moment,
                                    const X500Actuators &a = {})
{
    a.validate();
    if (!std::isfinite(f) || !moment.allFinite())
        throw std::invalid_argument("Non-finite requested wrench");
    const double lo = a.min_thrust(), hi = a.max_thrust();
    // Chia đều lực tổng làm nền. std::clamp giới hạn giá trị trong khoảng lo..hi.
    const double collective = std::clamp(f / 4.0, lo, hi);
    const double b = -moment.x() / (4.0*a.arm);
    const double c = moment.y() / (4.0*a.arm);
    const std::array<double, 4> rp{{b+c, -b-c, -b+c, b-c}};
    for (double value : rp)
        if (!std::isfinite(value)) throw std::invalid_argument("Requested torque overflows allocation");
    X500Allocation out;
    // Ưu tiên lực tổng -> mômen nghiêng ngang/dọc -> mômen quay hướng mũi.
    // Nếu thiếu biên lực, giảm chung thành phần nghiêng, thay vì bỏ toàn bộ lệnh.
    // Chính sách ưu tiên này là lựa chọn kỹ thuật cho X500, không phải luật P01.
    for (double delta : rp)
    {
        if (delta > 0) out.roll_pitch_scale = std::min(out.roll_pitch_scale, (hi-collective)/delta);
        if (delta < 0) out.roll_pitch_scale = std::min(out.roll_pitch_scale, (lo-collective)/delta);
    }
    // Tính biên mômen hướng mũi còn khả thi sau khi đã dành lực cho hai mục trên.
    double yaw_lo = -hi, yaw_hi = hi;
    for (int i = 0; i < 4; ++i)
    {
        out.thrusts[i] = collective + out.roll_pitch_scale*rp[i];
        if (i < 2)
        {
            yaw_lo = std::max(yaw_lo, lo-out.thrusts[i]);
            yaw_hi = std::min(yaw_hi, hi-out.thrusts[i]);
        }
        else
        {
            yaw_lo = std::max(yaw_lo, out.thrusts[i]-hi);
            yaw_hi = std::min(yaw_hi, out.thrusts[i]-lo);
        }
    }
    // Bao gồm 0 trong khoảng khả thi để tránh sai số làm tròn ở sát giới hạn.
    yaw_lo = std::min(yaw_lo, 0.0);
    yaw_hi = std::max(yaw_hi, 0.0);
    const double yaw = std::clamp(moment.z()/(4*a.moment_ratio), yaw_lo, yaw_hi);
    for (int i = 0; i < 4; ++i)
    {
        out.thrusts[i] = std::clamp(out.thrusts[i] + (i < 2 ? yaw : -yaw), lo, hi);
        out.commands[i] = a.command_from_thrust(out.thrusts[i]);
    }
    // Dựng lại đầu ra khả thi và so với yêu cầu. saturated=true vẫn có thể tạo
    // đủ bốn lệnh hữu hạn; cờ này không tự làm mất tín hiệu duy trì Offboard.
    const auto achieved = x500_wrench(out.thrusts, a);
    out.achieved_f = achieved[0];
    out.achieved_M = achieved.tail<3>();
    out.saturated = std::abs(out.achieved_f-f) > 1e-8 || (out.achieved_M-moment).norm() > 1e-8;
    return out;
}
// ===== X650 (mô hình Gazebo x650, nhân bản từ x500, 25/11/2026) =====
// Số ƯỚC LƯỢNG theo cấu hình X650 + 4x MN4014 KV330 + cánh 1555 + pin 6S; thay bằng số đo khi có máy thật.
// Khớp PX4-Autopilot/Tools/simulation/gz/models/x650_base + x650 và airframe 4022_gz_x650.
inline P01Parameters x650_parameters()
{
    P01Parameters p;
    p.m = 3.0;                                              // kg: thân 2,9357 + 4 rotor 0,0161
    p.J = Vector3(0.05999, 0.06009, 0.11200).asDiagonal();  // kg.m²: thân + rotor quy về khối tâm
    p.kx = 4.0 * p.m;
    p.kv = 3.2 * p.m;
    // Hệ số tư thế/vận tốc góc nhân theo tỉ lệ J (x650/x500) để giữ nguyên tần số riêng vòng tư thế.
    p.attitude_scale = Vector3(1.5 * 0.05999 / 0.02383948, 1.5 * 0.06009 / 0.02394241, 0.35 * 0.11200 / 0.04399995);
    p.rate_scale = Vector3(0.35 * 0.05999 / 0.02383948, 0.35 * 0.06009 / 0.02394241, 0.22 * 0.11200 / 0.04399995);
    return p;
}

inline X500Actuators x650_actuators()
{
    X500Actuators a;
    a.arm = 0.23;              // m, theo x/y thân
    a.moment_ratio = 0.016;    // m, momentConstant (chưa đo)
    a.motor_constant = 6.4e-5; // N/(rad/s)^2, motorConstant
    a.omega_min = 100.0;       // SIM_GZ_EC_MIN của 4022_gz_x650
    a.omega_max = 630.0;       // SIM_GZ_EC_MAX của 4022_gz_x650
    return a;
}

} // namespace uav_research
