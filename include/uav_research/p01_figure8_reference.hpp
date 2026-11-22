#pragma once
// THAM CHIẾU QUỸ ĐẠO SỐ 8 (lemniscate Gerono) cho X500 — tiện ích thí nghiệm, KHÔNG có trong bài P01/P02.
//
// Trong hệ ngang gắn theo hướng mũi lúc cất cánh (trục dọc = hướng mũi, trục ngang = bên phải):
//   dọc  = a · sin(θ)
//   ngang = b · sin(2θ)
//   cao độ giữ nguyên (bay số 8 trên mặt phẳng ngang).
// θ = ω · φ(t), ω = 2π / period. φ(t) là "thời gian pha" tăng tốc êm trong ramp giây, chạy đều,
// rồi giảm tốc êm trong ramp giây cuối (dùng lại phase_ramp của Case I), nên v, a, jerk, snap
// bằng 0 ở hai đầu và quỹ đạo bắt đầu/kết thúc đúng tại tâm (ngay trên điểm cất cánh).
// Thời gian bay: duration = laps · period + ramp.
// Vận tốc lớn nhất ≈ max(a·ω, 2·b·ω); gia tốc lớn nhất ≈ max(a·ω², 4·b·ω²).
// Mặc định a = 1,5 m, b = 0,75 m, period = 10 s: v ≈ 0,94 m/s, gia tốc ≈ 1,2 m/s² (nhẹ hơn Case I 1×).
// Giảm period để bay gắt hơn (gia tốc tỉ lệ 1/period²).
#include "uav_research/x500_reference.hpp"
#include <cmath>
#include <stdexcept>

namespace uav_research
{
inline double figure8_duration(double period, int laps, double ramp)
{
    return laps * period + ramp;
}

inline P01Reference reference_figure8(double t, const Eigen::Vector3d &center, const Eigen::Vector3d &heading,
                                      double a, double b, double period, int laps, double ramp)
{
    if (!std::isfinite(t) || !(period > 0) || laps < 1 || !(ramp > 0) || !(a > 0) || !(b > 0))
        throw std::invalid_argument("Invalid figure-8 parameters");
    const double duration = figure8_duration(period, laps, ramp);
    // Thời gian pha: tăng tốc trừ bản trễ để giảm tốc (giống reference_x500_case1).
    auto d = phase_ramp(t, ramp);
    const auto stop = phase_ramp(t - (duration - ramp), ramp);
    for (int i = 0; i < 5; ++i) d[i] -= stop[i];
    const double w = 2.0 * M_PI / period;
    // θ và các đạo hàm theo thời gian.
    const double th = w * d[0], t1 = w * d[1], t2 = w * d[2], t3 = w * d[3], t4 = w * d[4];

    // f(θ) = sin(kθ) và các đạo hàm theo θ; ghép với θ(t) bằng công thức Faà di Bruno tới bậc 4.
    auto chain = [&](double amp, double k, double out[5]) {
        const double s = std::sin(k * th), c = std::cos(k * th);
        const double f0 = amp * s, f1 = amp * k * c, f2 = -amp * k * k * s;
        const double f3 = -amp * k * k * k * c, f4 = amp * k * k * k * k * s;
        out[0] = f0;
        out[1] = f1 * t1;
        out[2] = f2 * t1 * t1 + f1 * t2;
        out[3] = f3 * t1 * t1 * t1 + 3 * f2 * t1 * t2 + f1 * t3;
        out[4] = f4 * std::pow(t1, 4) + 6 * f3 * t1 * t1 * t2 + 3 * f2 * t2 * t2 + 4 * f2 * t1 * t3 + f1 * t4;
    };
    double lon[5], lat[5];
    chain(a, 1.0, lon);   // dọc theo hướng mũi
    chain(b, 2.0, lat);   // ngang

    // Hướng mũi giữ cố định như lúc cất cánh (b1 từ reference_hover).
    P01Reference r = reference_hover(center, heading);
    const Eigen::Vector3d ex = r.b1;                                      // trục dọc (ngang mặt đất)
    const Eigen::Vector3d ey = Eigen::Vector3d::UnitZ().cross(ex);         // trục ngang: z (xuống) × dọc = bên phải
    r.x = center + lon[0] * ex + lat[0] * ey;
    r.v = lon[1] * ex + lat[1] * ey;
    r.a = lon[2] * ex + lat[2] * ey;
    r.j = lon[3] * ex + lat[3] * ey;
    r.snap = lon[4] * ex + lat[4] * ey;
    return r;
}
} // namespace uav_research
