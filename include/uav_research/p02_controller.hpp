#pragma once
// LÕI TOÁN P02: PID hình học trên SE(3) (Goodarzi, Lee, Lee — arXiv:1304.6765v1, Mục IV).
// Nhận trạng thái + tham chiếu + tham số, trả lực f và mômen M. Không gửi lệnh tới PX4.
// Khác P01 ở 3 chỗ:
//   (1) thêm tích phân vị trí e_i (18) có bão hòa sat_sigma trong lực yêu cầu A (20),(25);
//   (2) thêm tích phân tư thế e_I (14) trong mômen M (26);
//   (3) mômen M theo dạng (26): không bù Omega x J Omega, bù đón trước theo R^T Rc Omega_c.
// Khác P01, bộ điều khiển này CÓ TRẠNG THÁI (hai biến tích phân) nên viết thành class.
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

#include "uav_research/p01_geometric_controller.hpp"  // hat3, vee3, normalized_with_derivatives, P01State...

namespace uav_research
{

// Hệ số riêng của P02 (phần P, D và tham số vật lý dùng lại P01Parameters).
// Giá trị mặc định là ĐỀ XUẤT cho X500 (chưa bay thử), suy từ điều kiện (29),(30),(16):
//   c1 < min{1.95, 2} với kx = 4m, kv = 3.2m  → chọn 1.0
//   ki = 1.0 m, sigma = 3 → ki*sigma ≈ 6.2 N > delta_x (sai khối lượng tới ~0.6 kg);
//   hằng số thời gian khử lệch ≈ kx/(ki*c1) = 4 s (ki = 0.4 m cho ~10 s, quá chậm khi thử bằng mô phỏng đơn giản)
//   kI nhỏ vì Gazebo gần như không có nhiễu mômen Delta_R; c2 = 0.8 như bài.
struct P02Gains
{
    double ki = 1.0 * 2.0643076923;   // hệ số tích phân vị trí (N/(m.s))
    double c1 = 1.0;                  // trọng số e_x trong tích phân vị trí (1/s)
    double sigma = 3.0;               // ngưỡng bão hòa của e_i, từng thành phần (m.s)
    Vector3 kI = Vector3(0.1, 0.1, 0.03);  // hệ số tích phân tư thế theo trục (N.m/(rad.s))
    double c2 = 0.8;                  // trọng số e_R trong tích phân tư thế (1/s)
};

// Kết quả P02 = mọi trường của P01Control (để dùng lại telemetry/log) + hai biến tích phân.
// Ở P02, Rd/omega_d/omega_d_dot chính là Rc/Omega_c/Omega_c_dot trong bài.
struct P02Control : P01Control
{
    Vector3 ei = Vector3::Zero();  // tích phân vị trí (18) tại thời điểm tính
    Vector3 eI = Vector3::Zero();  // tích phân tư thế (14) tại thời điểm tính
};

// Bão hòa từng phần tử về [-sigma, sigma] (định nghĩa sat_sigma, Mục IV-A).
inline Vector3 sat_sigma(const Vector3 &y, double sigma)
{
    return y.cwiseMax(-sigma).cwiseMin(sigma);
}

class P02Controller
{
public:
    // Xóa hai tích phân (gọi khi bắt đầu một lần bay mới).
    void reset()
    {
        ei_.setZero();
        eI_.setZero();
    }

    // state: phản hồi; p: tham số vật lý + kx, kv, kR (attitude_scale), kOmega (rate_scale);
    // g: hệ số P02; r: tham chiếu; dt: bước tích phân (s); integrate: có cộng dồn tích phân không.
    // Tích phân dùng Euler tiến: lực/mômen bước này dùng e_i, e_I tích lũy tới bước trước.
    P02Control compute(const P01State &state, const P01Parameters &p, const P02Gains &g,
                       const P01Reference &r, double dt, bool integrate)
    {
        if (!(p.m > 0.0) || !(g.sigma > 0.0))
        {
            throw std::invalid_argument("P02: mass and sigma must be positive");
        }

        const Vector3 e3 = Vector3::UnitZ();
        P02Control out;

        // (17): sai số vị trí, vận tốc.
        out.ex = state.x - r.x;
        out.ev = state.v - r.v;

        // (18)+(20): lực yêu cầu A có thêm -ki*sat(e_i). (25): f = -A . Re3.
        const Vector3 s = sat_sigma(ei_, g.sigma);
        out.A = -p.kx * out.ex - p.kv * out.ev - g.ki * s - p.m * p.g * e3 + p.m * r.a;
        const Vector3 b = state.R * e3;
        out.f = -out.A.dot(b);

        // Mặt nạ bão hòa: thành phần nào đang bị cắt thì đạo hàm sat(e_i) bằng 0.
        Vector3 mask;
        for (int k = 0; k < 3; ++k)
        {
            mask[k] = (std::abs(ei_[k]) < g.sigma) ? 1.0 : 0.0;
        }

        // Đạo hàm A để dựng Rc_dot, Rc_ddot (giống P01, thêm số hạng tích phân).
        // e_i_dot = e_v + c1 e_x; e_i_ddot = (gia tốc - a_d) + c1 e_v. Gia tốc/jerk suy từ mô hình lý tưởng.
        const Vector3 acceleration = p.g * e3 - (out.f / p.m) * b;
        const Vector3 s_dot = mask.cwiseProduct(out.ev + g.c1 * out.ex);
        const Vector3 A_dot =
            -p.kx * out.ev - p.kv * (acceleration - r.a) - g.ki * s_dot + p.m * r.j;
        const Matrix3 omega_hat = hat3(state.omega);
        const Vector3 b_dot = state.R * omega_hat * e3;
        const double f_dot = -A_dot.dot(b) - out.A.dot(b_dot);
        const Vector3 jerk = -(f_dot * b + out.f * b_dot) / p.m;
        const Vector3 s_ddot = mask.cwiseProduct((acceleration - r.a) + g.c1 * out.ev);
        const Vector3 A_ddot = -p.kx * (acceleration - r.a) - p.kv * (jerk - r.j) -
                               g.ki * s_ddot + p.m * r.snap;

        // (19),(20) và Mục IV-C: b3c = -A/|A|, b1c = hình chiếu của b1d. Giống P01.
        const auto b3 = normalized_with_derivatives(-out.A, -A_dot, -A_ddot);

        const Vector3 c2v = b3.q.cross(r.b1);
        const Vector3 c2v_dot = b3.q_dot.cross(r.b1) + b3.q.cross(r.b1_dot);
        const Vector3 c2v_ddot =
            b3.q_ddot.cross(r.b1) + 2.0 * b3.q_dot.cross(r.b1_dot) + b3.q.cross(r.b1_ddot);
        const auto b2 = normalized_with_derivatives(c2v, c2v_dot, c2v_ddot);

        const Vector3 b1 = b2.q.cross(b3.q);
        const Vector3 b1_dot = b2.q_dot.cross(b3.q) + b2.q.cross(b3.q_dot);
        const Vector3 b1_ddot =
            b2.q_ddot.cross(b3.q) + 2.0 * b2.q_dot.cross(b3.q_dot) + b2.q.cross(b3.q_ddot);

        out.Rd.col(0) = b1;
        out.Rd.col(1) = b2.q;
        out.Rd.col(2) = b3.q;
        Matrix3 Rd_dot, Rd_ddot;
        Rd_dot << b1_dot, b2.q_dot, b3.q_dot;
        Rd_ddot << b1_ddot, b2.q_ddot, b3.q_ddot;

        // (19): Omega_c^ = Rc^T Rc_dot; Omega_c_dot lấy từ đạo hàm tiếp theo.
        out.omega_d = vee3(out.Rd.transpose() * Rd_dot);
        out.omega_d_dot = vee3(Rd_dot.transpose() * Rd_dot + out.Rd.transpose() * Rd_ddot);

        // (21): sai số tư thế và vận tốc góc.
        const Matrix3 T = state.R.transpose() * out.Rd;
        out.eR = 0.5 * vee3(out.Rd.transpose() * state.R - state.R.transpose() * out.Rd);
        out.eOmega = state.omega - T * out.omega_d;

        // (26): M = -kR eR - kOmega eOmega - kI eI + (R^T Rc Omega_c)^ J R^T Rc Omega_c + J R^T Rc Omega_c_dot.
        // Hệ số theo trục (cwiseProduct) như P01 cho X500; bài dùng hệ số vô hướng.
        const Vector3 w = T * out.omega_d;
        out.M = -p.attitude_scale.cwiseProduct(out.eR) - p.rate_scale.cwiseProduct(out.eOmega) -
                g.kI.cwiseProduct(eI_) + w.cross(p.J * w) + p.J * (T * out.omega_d_dot);

        out.Psi = 0.5 * (Matrix3::Identity() - out.Rd.transpose() * state.R).trace();
        out.ei = ei_;
        out.eI = eI_;

        if (!std::isfinite(out.f) || !out.M.allFinite())
        {
            throw std::runtime_error("P02 control contains non-finite values");
        }

        // Cộng dồn tích phân (18),(14) cho bước sau. dt bị chặn để tránh nhảy khi vòng lặp trễ.
        if (integrate && dt > 0.0)
        {
            const double h = std::min(dt, 0.05);
            ei_ += h * (out.ev + g.c1 * out.ex);
            eI_ += h * (out.eOmega + g.c2 * out.eR);
        }
        return out;
    }

    const Vector3 &ei() const { return ei_; }
    const Vector3 &eI() const { return eI_; }

private:
    Vector3 ei_ = Vector3::Zero();
    Vector3 eI_ = Vector3::Zero();
};

}  // namespace uav_research
