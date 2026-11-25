#pragma once
// GIAO DIỆN CHUNG CHO CÁC BỘ ĐIỀU KHIỂN (P01, P02, sau này P06, P11).
// Mọi bộ nhận CÙNG trạng thái (P01State), CÙNG tham chiếu (P01Reference), CÙNG tham số UAV (P01Parameters)
// và trả về lực f + mômen M (P01Control). Node uav_simple chỉ gọi qua giao diện này,
// nên phân bổ rotor, quỹ đạo, ghi CSV là chung → so sánh công bằng giữa các bài.
// Lõi toán của từng bài giữ nguyên trong tệp riêng (p01_geometric_controller.hpp, p02_controller.hpp);
// tệp này chỉ "bọc" (adapter) chúng lại.

#include "uav_research/p01_geometric_controller.hpp"
#include "uav_research/p02_controller.hpp"
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
} // namespace uav_research
