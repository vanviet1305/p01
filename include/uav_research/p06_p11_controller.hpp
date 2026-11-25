#pragma once
// P06 + P11: BỘ ĐIỀU KHIỂN HÌNH HỌC THÍCH NGHI (geometric adaptive controller).
//  P06: Goodarzi, Lee, Lee, "Geometric adaptive tracking control of a quadrotor UAV on SE(3)" (arXiv:1411.2986v1).
//       Tư thế liên kết (coupled): sai số e_R trên SO(3), như P01/P02.
//  P11: Gamagedara, Bisheban, Kaufman, Lee, "Geometric adaptive controls of a quadrotor UAV
//       with decoupled attitude dynamics" (bản thảo tác giả, ASME). Tư thế TÁCH (decoupled): roll/pitch trên S²
//       (hướng b3) và yaw trên S¹ (quay quanh b3).
// Chuyển từ mã MATLAB của tác giả P11: fdcl-gwu/decoupled-yaw-controller-comparison, commit 1d067d5,
//  FDCL/position_control.m, attitude_control.m (P11), attitude_control_coupled.m (P06).
// Quy ước giống P01: hệ quán tính NED (e3 hướng xuống), hệ thân FRD, R: thân → quán tính.
// Nhiễu mô hình: W_x = I, W_R = I (nhiễu lực và mômen hằng chưa biết, đúng trường hợp thử của tác giả).
//
// Đối chiếu (25/11/2026): mô phỏng vòng kín C++ (RK4) trùng kết quả MATLAB tới 0,01 mm ở 3 ca
//  (FDCL/decoupled.mat, FDCL/coupled.mat của người dùng; Data/Circle_with_uncertainties/decoupled.mat của tác giả).
//  Ca Data/Circle_with_uncertainties/coupled.mat lệch (avg_ex 0,465 so với 0,486): có thể dữ liệu đó sinh từ phiên bản mã khác.
// Các chỗ viết khác mã gốc:
//  (a) b1c_ddot: mã gốc ghi hat(b2c)*b2c_ddot, đúng là hat(b2c)*b3c_ddot. Lỗi này không ảnh hưởng kết quả MATLAB vì
//      vee.m của tác giả chỉ đọc các phần tử không chứa cột b1c_ddot; ở đây dùng công thức đúng.
//  (b) c3: mã gốc tính mẫu số bằng 4·J1·k_R; công thức (71) của P11 dùng 4·J3·k_y. Ở đây c3 do người dùng truyền vào.
//  (c) Đạo hàm ma trận chiếu I_theta_dot (chỉ dùng khi ‖θ̄_x‖ chạm biên B_theta_x): mã gốc chia cho ‖θ‖³ và trừ
//      một vector vào mọi cột ma trận; ở đây dùng đạo hàm đúng của I − θθᵀ/‖θ‖². Không ảnh hưởng khi ‖θ̄_x‖ < B.
// Hệ số tư thế: dùng vector theo trục như P01 (attitude_scale, rate_scale của P01Parameters):
//  P06: M = -kR∘e_R - kW∘e_W ...;  P11: k_b = kR.x, k_w = kW.x (cần kR.x ≈ kR.y), k_y = kR.z, k_wy = kW.z.

#include "uav_research/p01_geometric_controller.hpp"
#include <algorithm>
#include <cmath>

namespace uav_research
{
struct AdaptiveGains
{
    double c1 = 1.0;          // trọng số e_x trong luật thích nghi vị trí
    double c2 = 1.0;          // trọng số e_R (P06) / e_b (P11) trong luật thích nghi tư thế
    double c3 = 1.0;          // trọng số e_y trong luật thích nghi yaw (chỉ P11)
    double gamma_x = 1.0;     // tốc độ thích nghi lực
    double gamma_R = 0.1;     // tốc độ thích nghi mômen
    double B_theta_x = 10.0;  // chặn ‖θ̄_x‖ (N) của phép chiếu (projection)
};

struct AdaptiveControl : P01Control
{
    Vector3 theta_x = Vector3::Zero();  // θ̄_x: ước lượng nhiễu lực (N), hệ quán tính
    Vector3 theta_R = Vector3::Zero();  // θ̄_R: ước lượng nhiễu mômen (N.m), hệ thân
    Vector3 eb = Vector3::Zero();       // P11: sai số hướng b3 trên S²
    double ey = 0.0;                    // P11: sai số yaw
    double ewy = 0.0;                   // P11: sai số vận tốc yaw
    Vector3 theta_x_dot = Vector3::Zero(), theta_R_dot = Vector3::Zero();
};

// u = q/‖q‖ và hai đạo hàm (Common/deriv_unit_vector.m)
inline void deriv_unit(const Vector3 &q, const Vector3 &qd, const Vector3 &qdd, Vector3 &u, Vector3 &ud, Vector3 &udd)
{
    const double n = q.norm(), n3 = n * n * n, n5 = n3 * n * n, qqd = q.dot(qd);
    u = q / n;
    ud = qd / n - q * qqd / n3;
    udd = qdd / n - qd / n3 * (2.0 * qqd) - q / n3 * (qd.dot(qd) + q.dot(qdd)) + 3.0 * q / n5 * qqd * qqd;
}

class AdaptiveGeometricController
{
public:
    explicit AdaptiveGeometricController(bool decoupled) : decoupled_(decoupled) {}
    void reset() { theta_x_.setZero(); theta_R_.setZero(); }
    void set_theta(const Vector3 &tx, const Vector3 &tR) { theta_x_ = tx; theta_R_ = tR; }

    // Tính f, M từ trạng thái và θ̄ hiện tại; nếu integrate, cập nhật θ̄ bằng Euler với bước dt.
    AdaptiveControl compute(const P01State &s, const P01Parameters &p, const AdaptiveGains &g,
                            const P01Reference &r, double dt, bool integrate)
    {
        AdaptiveControl out = evaluate(s, p, g, r, theta_x_, theta_R_, integrate);
        if (integrate && dt > 0.0)
        {
            const double h = std::min(dt, 0.05);
            theta_x_ += h * out.theta_x_dot;
            theta_R_ += h * out.theta_R_dot;
        }
        return out;
    }

    // Hàm thuần (không đổi trạng thái): f, M, θ̄_dot tại (s, θ̄). Dùng cho cả kiểm tra offline.
    // adapting = false: θ̄ đang đứng yên (không cập nhật) → θ̄_dot = 0 trong luật và trong các đạo hàm A_dot, A_ddot
    // (đúng như mã gốc khi không có nhiễu: W_x = 0). adapting = true: θ̄ thay đổi theo luật thích nghi.
    AdaptiveControl evaluate(const P01State &s, const P01Parameters &p, const AdaptiveGains &g,
                             const P01Reference &r, const Vector3 &th_x, const Vector3 &th_R, bool adapting) const
    {
        AdaptiveControl o;
        const Vector3 e1 = Vector3::UnitX(), e2 = Vector3::UnitY(), e3 = Vector3::UnitZ();
        const double m = p.m, gr = p.g;
        const Matrix3 &R = s.R;
        const Vector3 &W = s.omega;
        o.theta_x = th_x;
        o.theta_R = th_R;

        // ---- Vòng vị trí, P11 (8)–(12) / P06 (18)–(20), W_x = I ----
        o.ex = s.x - r.x;
        o.ev = s.v - r.v;
        o.A = -p.kx * o.ex - p.kv * o.ev - m * gr * e3 + m * r.a - th_x;
        const Vector3 ev_c1ex = o.ev + g.c1 * o.ex;
        const double nth = th_x.norm();
        const bool inside = nth < g.B_theta_x || th_x.dot(ev_c1ex) <= 0.0;   // phép chiếu: trong biên hoặc đang đi vào
        Matrix3 Ith = Matrix3::Identity();
        if (!inside) Ith -= th_x * th_x.transpose() / th_x.squaredNorm();
        o.theta_x_dot = adapting ? Vector3(g.gamma_x * Ith * ev_c1ex) : Vector3::Zero();

        const Vector3 b3 = R * e3;
        o.f = -o.A.dot(b3);
        const Vector3 ev_dot = gr * e3 - o.f / m * b3 - r.a + th_x / m;
        const Vector3 A_dot = -p.kx * o.ev - p.kv * ev_dot + m * r.j - o.theta_x_dot;
        Vector3 th_x_ddot = Vector3::Zero();
        if (!adapting) {}
        else if (inside)
            th_x_ddot = g.gamma_x * (ev_dot + g.c1 * o.ev);
        else
        {
            // d/dt[I − θθᵀ/‖θ‖²] = −(θ̇θᵀ + θθ̇ᵀ)/‖θ‖² + 2(θᵀθ̇)θθᵀ/‖θ‖⁴  (viết lại đúng; xem ghi chú (c))
            const double n2 = th_x.squaredNorm();
            const Matrix3 Ith_dot = -(o.theta_x_dot * th_x.transpose() + th_x * o.theta_x_dot.transpose()) / n2 +
                                    2.0 * th_x.dot(o.theta_x_dot) * th_x * th_x.transpose() / (n2 * n2);
            th_x_ddot = g.gamma_x * Ith_dot * ev_c1ex + g.gamma_x * Ith * (ev_dot + g.c1 * o.ev);
        }
        const Vector3 b3_dot = R * hat3(W) * e3;
        const double f_dot = -A_dot.dot(b3) - o.A.dot(b3_dot);
        const Vector3 ev_2dot = -f_dot / m * b3 - o.f / m * b3_dot - r.j + o.theta_x_dot / m;
        const Vector3 A_ddot = -p.kx * ev_dot - p.kv * ev_2dot + m * r.snap - th_x_ddot;

        // ---- Tư thế mong muốn R_c và đạo hàm ----
        Vector3 b3c, b3c_d, b3c_dd, b2c, b2c_d, b2c_dd;
        deriv_unit(-o.A, -A_dot, -A_ddot, b3c, b3c_d, b3c_dd);
        const Vector3 A2 = -hat3(r.b1) * b3c;
        const Vector3 A2_d = -hat3(r.b1_dot) * b3c - hat3(r.b1) * b3c_d;
        const Vector3 A2_dd = -hat3(r.b1_ddot) * b3c - 2.0 * hat3(r.b1_dot) * b3c_d - hat3(r.b1) * b3c_dd;
        deriv_unit(A2, A2_d, A2_dd, b2c, b2c_d, b2c_dd);
        const Vector3 b1c = hat3(b2c) * b3c;
        const Vector3 b1c_d = hat3(b2c_d) * b3c + hat3(b2c) * b3c_d;
        const Vector3 b1c_dd = hat3(b2c_dd) * b3c + 2.0 * hat3(b2c_d) * b3c_d +
                               hat3(b2c) * b3c_dd;
        Matrix3 Rc, Rc_d, Rc_dd;
        Rc << b1c, b2c, b3c;
        Rc_d << b1c_d, b2c_d, b3c_d;
        Rc_dd << b1c_dd, b2c_dd, b3c_dd;
        const Vector3 Wc = vee3(Rc.transpose() * Rc_d);
        const Vector3 Wc_dot = vee3(Rc.transpose() * Rc_dd - hat3(Wc) * hat3(Wc));
        o.Rd = Rc;
        o.omega_d = Wc;
        o.omega_d_dot = Wc_dot;
        o.eR = 0.5 * vee3(Rc.transpose() * R - R.transpose() * Rc);
        o.Psi = 0.5 * (Matrix3::Identity() - Rc.transpose() * R).trace();

        const Vector3 kR = p.attitude_scale, kW = p.rate_scale;
        const Matrix3 &J = p.J;
        if (!decoupled_)
        {
            // ---- P06: tư thế liên kết (attitude_control_coupled.m), W_R = I ----
            const Vector3 RtRcWc = R.transpose() * Rc * Wc;
            o.eOmega = W - RtRcWc;
            o.M = -kR.cwiseProduct(o.eR) - kW.cwiseProduct(o.eOmega) - th_R +
                  hat3(RtRcWc) * J * RtRcWc + J * R.transpose() * Rc * Wc_dot;
            o.theta_R_dot = adapting ? Vector3(g.gamma_R * (o.eOmega + g.c2 * o.eR)) : Vector3::Zero();
        }
        else
        {
            // ---- P11: roll/pitch trên S² + yaw trên S¹ (attitude_control.m), W_R = I ----
            const Vector3 b1 = R * e1, b2 = R * e2;
            const double W3 = b3.dot(Rc * Wc);
            const double W3_dot = b3.dot(Rc * Wc_dot) + (R * hat3(W) * e3).dot(Rc * Wc);
            const Vector3 w = W.x() * b1 + W.y() * b2;
            const Vector3 b3_dot_w = hat3(w) * b3;
            const Vector3 wd = hat3(b3c) * b3c_d;
            const Vector3 wd_dot = hat3(b3c) * b3c_dd;
            const Matrix3 hb3 = hat3(b3);
            o.eb = hat3(b3c) * b3;
            const Vector3 ew = w + hb3 * hb3 * wd;
            const double J1 = J(0, 0), J3 = J(2, 2);
            const Vector3 tau = -kR.x() * o.eb - kW.x() * ew - J1 * b3.dot(wd) * b3_dot_w - J1 * hb3 * hb3 * wd_dot -
                                th_R.x() * b1 - th_R.y() * b2;
            const double tau1 = b1.dot(tau), tau2 = b2.dot(tau);
            o.ey = -b2.dot(b1c);
            o.ewy = W.z() - W3;
            o.M = Vector3(tau1 + J3 * W.z() * W.y(), tau2 - J3 * W.z() * W.x(),
                          -kR.z() * o.ey - kW.z() * o.ewy - th_R.z() + J3 * W3_dot);
            // e_Omega để ghi log và so sánh: dùng dạng như P06
            o.eOmega = W - R.transpose() * Rc * Wc;
            const Vector3 ew_c2eb = ew + g.c2 * o.eb;
            o.theta_R_dot = adapting ? Vector3(g.gamma_R * Vector3(ew_c2eb.dot(b1), ew_c2eb.dot(b2), o.ewy + g.c3 * o.ey))
                                     : Vector3::Zero();
        }
        if (!std::isfinite(o.f) || !o.M.allFinite()) throw std::runtime_error("P06/P11 control contains non-finite values");
        return o;
    }

    const Vector3 &theta_x() const { return theta_x_; }
    const Vector3 &theta_R() const { return theta_R_; }

private:
    bool decoupled_;
    Vector3 theta_x_ = Vector3::Zero(), theta_R_ = Vector3::Zero();
};
} // namespace uav_research
