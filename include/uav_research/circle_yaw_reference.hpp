#pragma once
// THAM CHIẾU "ĐƯỜNG TRÒN + YAW QUAY" — dựng theo thí nghiệm Mục 4.1 của P11 (mã tác giả: Common/command_circle.m).
//  Vị trí (hệ ngang gắn hướng mũi lúc cất cánh; dọc = hướng mũi, ngang = bên phải):
//    dọc  = r·(cos θ − 1),  ngang = r·sin θ   → đường tròn bán kính r đi qua điểm bắt đầu, cao độ giữ nguyên.
//    θ = 2π·φ(t)/period, φ(t) tăng tốc/giảm tốc êm trong ramp giây (phase_ramp như Case I, số 8) để v = 0 ở hai đầu.
//    Bài báo bắt đầu ngay với vận tốc 0,63 m/s và lệch vị trí 1,4 m; ở đây bỏ hai điều đó để cất cánh an toàn.
//  Hướng mũi mong muốn b1d: tại t = 0 NHẢY lệch yaw_offset (mặc định π − 0,01 rad ≈ 179,4°, như R0 của tác giả),
//    rồi quay đều với chu kỳ yaw_period (mặc định 40 s, như tác giả). Hết quỹ đạo thì giữ hướng cuối.
//  Đây là ca P11 cần để thể hiện lợi thế tách yaw: sai số yaw lớn không được làm hỏng việc bám vị trí.
#include "uav_research/x500_reference.hpp"
#include <cmath>
#include <stdexcept>

namespace uav_research
{
inline double circle_yaw_duration(double period, int laps, double ramp) { return laps * period + ramp; }

inline P01Reference reference_circle_yaw(double t, const Eigen::Vector3d &start, const Eigen::Vector3d &heading,
                                         double radius, double period, int laps, double ramp,
                                         double yaw_offset, double yaw_period)
{
    if (!std::isfinite(t) || !(period > 0) || laps < 1 || !(ramp > 0) || !(radius > 0) || !(yaw_period >= 0))
        throw std::invalid_argument("Invalid circle_yaw parameters");
    const double duration = circle_yaw_duration(period, laps, ramp);
    auto d = phase_ramp(t, ramp);
    const auto stop = phase_ramp(t - (duration - ramp), ramp);
    for (int i = 0; i < 5; ++i) d[i] -= stop[i];
    const double w = 2.0 * M_PI / period;
    const double th = w * d[0], t1 = w * d[1], t2 = w * d[2], t3 = w * d[3], t4 = w * d[4];
    // g(θ) và đạo hàm theo θ (g0..g4) → đạo hàm theo thời gian bằng Faà di Bruno tới bậc 4.
    auto chain = [&](const double g[5], double out[5]) {
        out[0] = g[0];
        out[1] = g[1] * t1;
        out[2] = g[2] * t1 * t1 + g[1] * t2;
        out[3] = g[3] * t1 * t1 * t1 + 3 * g[2] * t1 * t2 + g[1] * t3;
        out[4] = g[4] * std::pow(t1, 4) + 6 * g[3] * t1 * t1 * t2 + 3 * g[2] * t2 * t2 + 4 * g[2] * t1 * t3 + g[1] * t4;
    };
    const double c = std::cos(th), s = std::sin(th), r = radius;
    const double glon[5] = {r * (c - 1.0), -r * s, -r * c, r * s, r * c};
    const double glat[5] = {r * s, r * c, -r * s, -r * c, r * s};
    double lon[5], lat[5];
    chain(glon, lon);
    chain(glat, lat);

    P01Reference ref = reference_hover(start, heading);
    const Eigen::Vector3d ex = ref.b1;                               // dọc (ngang mặt đất)
    const Eigen::Vector3d ey = Eigen::Vector3d::UnitZ().cross(ex);    // ngang: z (xuống) × dọc = bên phải
    ref.x = start + lon[0] * ex + lat[0] * ey;
    ref.v = lon[1] * ex + lat[1] * ey;
    ref.a = lon[2] * ex + lat[2] * ey;
    ref.j = lon[3] * ex + lat[3] * ey;
    ref.snap = lon[4] * ex + lat[4] * ey;

    // Hướng mũi: trước t = 0 giữ hướng cất cánh; từ t = 0 lệch yaw_offset rồi quay đều; sau duration giữ nguyên.
    if (t >= 0.0)
    {
        const double wy = (yaw_period > 0) ? 2.0 * M_PI / yaw_period : 0.0;
        const double tau = std::min(t, duration);
        const double psi = yaw_offset + wy * tau;
        const double rate = (t < duration) ? wy : 0.0;
        const Eigen::Vector3d b1 = std::cos(psi) * ex + std::sin(psi) * ey;
        const Eigen::Vector3d b1p = -std::sin(psi) * ex + std::cos(psi) * ey;   // vuông góc b1, trong mặt ngang
        ref.b1 = b1;
        ref.b1_dot = rate * b1p;
        ref.b1_ddot = -rate * rate * b1;
    }
    return ref;
}
} // namespace uav_research
