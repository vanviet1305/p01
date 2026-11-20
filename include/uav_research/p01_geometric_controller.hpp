#pragma once
// LÕI TOÁN P01: nhận trạng thái + tham chiếu + tham số, trả lực f và mômen M.
// Đây là phép tính toán thuần túy, không gửi lệnh tới PX4. Node ROS gọi hàm này trong vòng lặp điều khiển.
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

#include "uav_research/p01_reference.hpp"

namespace uav_research
{

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

// Trạng thái UAV hiện tại: x (m), v (m/s), omega (rad/s), R (ma trận không đơn vị).
// R đổi vectơ từ hệ thân sang hệ quán tính có trục z hướng xuống.
// x/v ở hệ quán tính; omega ở hệ thân. Node lấy chúng từ VehicleOdometry của PX4.
struct P01State
{
    Vector3 x = Vector3::Zero();
    Vector3 v = Vector3::Zero();
    Matrix3 R = Matrix3::Identity();
    Vector3 omega = Vector3::Zero();
};

// Tham số vật lý và hệ số phản hồi của X500 (giá trị mặc định = bộ đã bay tốt trong Gazebo).
// m (kg), g (m/s²), J (kg.m²). kx, kv: hệ số vị trí/vận tốc (N/m, N.s/m).
// attitude_scale, rate_scale: hệ số tư thế / vận tốc góc riêng từng trục (bài báo dùng một số vô hướng).
// Node chính có thể ghi đè các giá trị này bằng tham số ROS lúc khởi động.
struct P01Parameters
{
    double m = 2.0643076923;
    double g = 9.81;
    double kx = 4.0 * 2.0643076923;
    double kv = 3.2 * 2.0643076923;
    Vector3 attitude_scale = Vector3(1.5, 1.5, 0.35);
    Vector3 rate_scale = Vector3(0.35, 0.35, 0.22);
    Matrix3 J = Vector3(0.02383948, 0.02394241, 0.04399995).asDiagonal();
};

// Kết quả điều khiển kèm các đại lượng phục vụ kiểm tra và ghi log.
// ex/ev: sai số vị trí/vận tốc; eR/eOmega: sai số tư thế/vận tốc góc.
// Rd, omega_d, omega_d_dot: tư thế, vận tốc góc và gia tốc góc mong muốn.
// Psi là hàm sai số tư thế vô hướng; không phải góc tính bằng radian.
struct P01Control
{
    double f = 0.0;              // Lực tổng yêu cầu (N), chưa xét giới hạn rotor.
    Vector3 M = Vector3::Zero(); // Mômen yêu cầu trong hệ thân (N.m).
    Vector3 ex = Vector3::Zero();
    Vector3 ev = Vector3::Zero();
    Vector3 eR = Vector3::Zero();
    Vector3 eOmega = Vector3::Zero();
    Vector3 A = Vector3::Zero();
    Matrix3 Rd = Matrix3::Identity();
    Vector3 omega_d = Vector3::Zero();
    Vector3 omega_d_dot = Vector3::Zero();
    double Psi = 0.0;
};

// Toán tử hat: hat3(a) * b = a × b; vee3 lấy vector từ phần phản đối xứng.
inline Matrix3 hat3(const Vector3 &a)
{
    Matrix3 S;
    S << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    return S;
}

inline Vector3 vee3(const Matrix3 &S)
{
    const Matrix3 skew = 0.5 * (S - S.transpose());
    return Vector3(skew(2, 1), skew(0, 2), skew(1, 0));
}

// Vector đơn vị và hai đạo hàm, dùng để dựng các trục của tư thế mong muốn.
// Đây là bước triển khai đạo hàm giải tích; không phải bộ lọc dữ liệu đo.
struct UnitVectorKinematics
{
    Vector3 q, q_dot, q_ddot;
};

inline UnitVectorKinematics normalized_with_derivatives(const Vector3 &u, const Vector3 &u_dot,
                                                        const Vector3 &u_ddot)
{
    const double n = u.norm();
    if (!(n > 1e-9))
    {
        throw std::runtime_error("P01: cannot normalize a near-zero vector");
    }

    UnitVectorKinematics out;
    out.q = u / n;
    const double n_dot = out.q.dot(u_dot);
    const double n_ddot = (u_dot.squaredNorm() + u.dot(u_ddot) - n_dot * n_dot) / n;
    out.q_dot = (u_dot - out.q * n_dot) / n;
    out.q_ddot = (u_ddot - out.q * n_ddot - 2.0 * out.q_dot * n_dot) / n;
    return out;
}

// Chuyển từ mã MATLAB của dự án, có phần bù đón trước (feedforward) chuyển động.
// state: trạng thái phản hồi; p: tham số vật lý/hệ số; r: tham chiếu cùng thời điểm.
// Các khối dưới đây tương ứng luật P01; tên biến A là viết gọn biểu thức trong (15).
inline P01Control compute_p01_control_with_reference(const P01State &state, const P01Parameters &p,
                                                     const P01Reference &r)
{

    if (!(p.m > 0.0))
    {
        throw std::invalid_argument("P01 mass must be positive");
    }

    const Vector3 e3 = Vector3::UnitZ();
    P01Control out;

    // P01 (6),(7),(15), PDF trang 3: sai số và lực yêu cầu A trong hệ quán tính.
    // Chiếu A lên trục lực hiện tại Re3 để có lực tổng f; lực đẩy thực hướng -Re3.
    out.ex = state.x - r.x;
    out.ev = state.v - r.v;
    out.A = -p.kx * out.ex - p.kv * out.ev - p.m * p.g * e3 + p.m * r.a;
    const Vector3 b = state.R * e3;
    out.f = -out.A.dot(b);

    // Tính đạo hàm A để dựng Rd_dot, Rd_ddot rồi Omega_d, Omega_dot_d.
    // acceleration/jerk ở đây SUY TỪ mô hình lý tưởng của P01 (3), không phải
    // gia tốc/jerk đo riêng từ PX4. Bước này không xét lực cản hay bão hòa rotor.
    const Vector3 acceleration = p.g * e3 - (out.f / p.m) * b;
    const Vector3 A_dot = -p.kx * out.ev - p.kv * (acceleration - r.a) + p.m * r.j;
    const Matrix3 omega_hat = hat3(state.omega);
    const Vector3 b_dot = state.R * omega_hat * e3;
    const double f_dot = -A_dot.dot(b) - out.A.dot(b_dot);
    const Vector3 jerk = -(f_dot * b + out.f * b_dot) / p.m;
    const Vector3 A_ddot = -p.kx * (acceleration - r.a) - p.kv * (jerk - r.j) + p.m * r.snap;

    // P01 (12),(13): hướng trục thân thứ ba mong muốn là hướng -A đã chuẩn hóa.
    // Nếu A gần 0, hàm chuẩn hóa ném ngoại lệ thay vì tạo phép chia không hợp lệ.
    const auto b3 = normalized_with_derivatives(-out.A, -A_dot, -A_ddot);

    // Dựng hai trục còn lại từ hướng mũi ưu tiên r.b1 và b3.
    // Hai hướng song song cũng làm chuẩn hóa thất bại, đúng điều kiện cần kiểm tra.
    const Vector3 c2 = b3.q.cross(r.b1);
    const Vector3 c2_dot = b3.q_dot.cross(r.b1) + b3.q.cross(r.b1_dot);
    const Vector3 c2_ddot =
        b3.q_ddot.cross(r.b1) + 2.0 * b3.q_dot.cross(r.b1_dot) + b3.q.cross(r.b1_ddot);
    const auto b2 = normalized_with_derivatives(c2, c2_dot, c2_ddot);

    const Vector3 b1 = b2.q.cross(b3.q);
    const Vector3 b1_dot = b2.q_dot.cross(b3.q) + b2.q.cross(b3.q_dot);
    const Vector3 b1_ddot =
        b2.q_ddot.cross(b3.q) + 2.0 * b2.q_dot.cross(b3.q_dot) + b2.q.cross(b3.q_ddot);

    // Ghép ba trục trực chuẩn thành tư thế mong muốn Rd: mỗi trục là một cột.
    out.Rd.col(0) = b1;
    out.Rd.col(1) = b2.q;
    out.Rd.col(2) = b3.q;

    Matrix3 Rd_dot, Rd_ddot;
    Rd_dot.col(0) = b1_dot;
    Rd_dot.col(1) = b2.q_dot;
    Rd_dot.col(2) = b3.q_dot;
    Rd_ddot.col(0) = b1_ddot;
    Rd_ddot.col(1) = b2.q_ddot;
    Rd_ddot.col(2) = b3.q_ddot;

    // Tốc độ/gia tốc góc mong muốn ở hệ thân mong muốn, từ đạo hàm ma trận quay.
    out.omega_d = vee3(out.Rd.transpose() * Rd_dot);
    out.omega_d_dot = vee3(Rd_dot.transpose() * Rd_dot + out.Rd.transpose() * Rd_ddot);

    // P01 (10),(11): đưa đại lượng góc mong muốn về hệ thân hiện tại để trừ.
    const Matrix3 T = state.R.transpose() * out.Rd;
    out.eR = 0.5 * vee3(out.Rd.transpose() * state.R - state.R.transpose() * out.Rd);
    out.eOmega = state.omega - T * out.omega_d;
    // P01 (16): phản hồi tư thế/tốc độ góc + bù ghép quay + bù chuyển động đích.
    // cwiseProduct nhân từng thành phần (hệ số riêng từng trục của X500).
    out.M = -p.attitude_scale.cwiseProduct(out.eR) -
            p.rate_scale.cwiseProduct(out.eOmega) + state.omega.cross(p.J * state.omega) -
            p.J * (omega_hat * T * out.omega_d - T * out.omega_d_dot);
    // P01 (8): Psi là hàm sai số tư thế, không phải góc radian hoặc độ.
    out.Psi = 0.5 * (Matrix3::Identity() - out.Rd.transpose() * state.R).trace();

    if (!std::isfinite(out.f) || !out.M.allFinite())
    {
        throw std::runtime_error("P01 control contains non-finite values");
    }
    return out;
}

} // namespace uav_research
