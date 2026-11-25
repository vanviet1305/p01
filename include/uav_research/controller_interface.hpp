#pragma once
// GIAO DIỆN CHUNG CHO CÁC BỘ ĐIỀU KHIỂN (P01, P02, P06, P11).
// Mọi bộ nhận CÙNG trạng thái (P01State), CÙNG tham chiếu (P01Reference), CÙNG tham số UAV (P01Parameters)
// và trả về lực f + mômen M (P01Control). Node uav_simple chỉ gọi qua giao diện này,
// nên phân bổ rotor, quỹ đạo, ghi CSV là chung → so sánh công bằng giữa các bài.
// Lõi toán của từng bài giữ nguyên trong tệp riêng (p01_geometric_controller.hpp, p02_controller.hpp);
// tệp này chỉ "bọc" (adapter) chúng lại.

#include "uav_research/p01_geometric_controller.hpp"
#include "uav_research/p02_controller.hpp"
#include "uav_research/p06_p11_controller.hpp"
#include <cmath>
#include <cstdio>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>

namespace uav_research
{
class Controller
{
public:
    virtual ~Controller() = default;

    // Tên ngắn in ra log, ví dụ "p01".
    virtual std::string name() const = 0;

    // Xóa trạng thái bên trong (tích phân, ước lượng thích nghi). Node gọi khi chạm đất.
    virtual void reset() {}

    // Tính f, M. dt: bước thời gian thực (s). integrate: node cho phép cộng dồn trạng thái
    // bên trong ở giai đoạn này (cất cánh, bay, bay về); false lúc chờ trên đất và lúc hạ cánh.
    virtual P01Control compute(const P01State &state, const P01Parameters &p,
                               const P01Reference &r, double dt, bool integrate) = 0;

    // Cột CSV riêng của bộ điều khiển: mỗi cột bắt đầu bằng dấu phẩy, ví dụ ",ei_x,ei_y".
    // Giá trị của lần compute() gần nhất được ghi bằng csv_row() theo đúng thứ tự đó.
    virtual std::string csv_header() const { return ""; }
    virtual void csv_row(std::ostream &) const {}

    // Chuỗi ngắn thêm vào dòng log 1 lần/giây, ví dụ " ei_z=0.123".
    virtual std::string status() const { return ""; }

    // Mô tả hệ số, in một lần lúc khởi động.
    virtual std::string describe() const { return ""; }
};

// ===== P01: Lee, Leok, McClamroch, CDC 2010. Không có trạng thái bên trong. =====
class P01Ctrl : public Controller
{
public:
    std::string name() const override { return "p01"; }

    P01Control compute(const P01State &state, const P01Parameters &p,
                       const P01Reference &r, double, bool) override
    {
        return compute_p01_control_with_reference(state, p, r);
    }
};

// ===== P02: Goodarzi, Lee, Lee, arXiv:1304.6765v1. Hai tích phân e_i (vị trí), e_I (tư thế). =====
class P02Ctrl : public Controller
{
public:
    P02Ctrl(const P02Gains &g, bool use_integral) : g_(g), use_integral_(use_integral) {}

    std::string name() const override { return "p02"; }
    void reset() override { ctrl_.reset(); }

    P01Control compute(const P01State &state, const P01Parameters &p,
                       const P01Reference &r, double dt, bool integrate) override
    {
        last_ = ctrl_.compute(state, p, g_, r, dt, use_integral_ && integrate);
        return last_;   // cắt về P01Control (f, M và các sai số chung)
    }

    std::string csv_header() const override { return ",ei_x,ei_y,ei_z,eI_x,eI_y,eI_z"; }
    void csv_row(std::ostream &o) const override
    {
        for (int i = 0; i < 3; ++i) o << ',' << last_.ei[i];
        for (int i = 0; i < 3; ++i) o << ',' << last_.eI[i];
    }
    std::string status() const override { return " ei_z=" + std::to_string(last_.ei.z()); }
    std::string describe() const override
    {
        return "ki=" + std::to_string(g_.ki) + " c1=" + std::to_string(g_.c1) + " sigma=" + std::to_string(g_.sigma) +
               " kI=(" + std::to_string(g_.kI.x()) + "," + std::to_string(g_.kI.z()) + ") c2=" + std::to_string(g_.c2) +
               " integral=" + std::to_string(use_integral_);
    }

private:
    P02Gains g_;
    bool use_integral_ = true;
    P02Controller ctrl_;
    P02Control last_;
};
// ===== P06 (decoupled = false) và P11 (decoupled = true): thích nghi hình học, xem p06_p11_controller.hpp =====
// θ̄_x, θ̄_R chỉ cập nhật khi node cho phép (integrate), đứng yên ở giai đoạn khác; xóa khi chạm đất.
class AdaptiveCtrl : public Controller
{
public:
    AdaptiveCtrl(bool decoupled, const AdaptiveGains &g) : decoupled_(decoupled), g_(g), ctrl_(decoupled) {}

    std::string name() const override { return decoupled_ ? "p11" : "p06"; }
    void reset() override { ctrl_.reset(); }

    P01Control compute(const P01State &state, const P01Parameters &p,
                       const P01Reference &r, double dt, bool integrate) override
    {
        last_ = ctrl_.compute(state, p, g_, r, dt, integrate);
        return last_;
    }

    std::string csv_header() const override
    {
        return ",thx_x,thx_y,thx_z,thR_x,thR_y,thR_z,eR_x,eR_y,eR_z,eb_x,eb_y,eb_z,ey,ewy";
    }
    void csv_row(std::ostream &o) const override
    {
        for (int i = 0; i < 3; ++i) o << ',' << last_.theta_x[i];
        for (int i = 0; i < 3; ++i) o << ',' << last_.theta_R[i];
        for (int i = 0; i < 3; ++i) o << ',' << last_.eR[i];
        for (int i = 0; i < 3; ++i) o << ',' << last_.eb[i];
        o << ',' << last_.ey << ',' << last_.ewy;
    }
    std::string status() const override
    {
        char b[96];
        std::snprintf(b, sizeof b, " thx=(%.2f,%.2f,%.2f)", last_.theta_x.x(), last_.theta_x.y(), last_.theta_x.z());
        return b;
    }
    std::string describe() const override
    {
        char b[200];
        std::snprintf(b, sizeof b, "gamma_x=%.3f gamma_R=%.3f B_theta_x=%.1f B_theta_R=%.3g c1=%.3f c2=%.3f c3=%.3f",
                      g_.gamma_x, g_.gamma_R, g_.B_theta_x, g_.B_theta_R, g_.c1, g_.c2, g_.c3);
        return b;
    }

    // Giá trị c1, c2, c3 mặc định theo điều kiện của bài (P06 (31)–(32) dạng mã tác giả; P11 (71) cho c3).
    static AdaptiveGains default_gains(const P01Parameters &p)
    {
        AdaptiveGains g;
        g.c1 = std::min(std::sqrt(p.kx / p.m), 4.0 * p.kx * p.kv / (p.kv * p.kv + 4.0 * p.m * p.kx));
        const double kR = p.attitude_scale.x(), kW = p.rate_scale.x(), J1 = p.J(0, 0), B2 = 1.0;
        g.c2 = std::min(std::sqrt(kR / J1), 4.0 * kR * kW / ((kW + J1 * B2) * (kW + J1 * B2) + 4.0 * J1 * kR));
        const double ky = p.attitude_scale.z(), kwy = p.rate_scale.z(), J3 = p.J(2, 2);
        g.c3 = std::min(std::sqrt(ky / J3), 4.0 * ky * kwy / (kwy * kwy + 4.0 * J3 * ky));
        g.gamma_x = 2.0;
        g.gamma_R = 0.5;
        g.B_theta_x = 10.0;
        return g;
    }

private:
    bool decoupled_;
    AdaptiveGains g_;
    AdaptiveGeometricController ctrl_;
    AdaptiveControl last_;
};
} // namespace uav_research
