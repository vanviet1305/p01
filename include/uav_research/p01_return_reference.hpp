#pragma once
// THAM CHIẾU BAY VỀ: đoạn thẳng từ điểm `from` tới điểm `to` (NED) trong `duration` giây,
// theo đa thức chuyển tiếp bậc 7 s(u) = 35u^4 − 84u^5 + 70u^6 − 20u^7, u = t/duration.
// v, a, jerk, snap bằng 0 ở hai đầu (cùng đa thức với phase_ramp ở x500_reference.hpp),
// nên nối êm với đoạn giữ trước đó và đoạn giữ/hạ cánh sau đó.
// Dùng để đưa UAV về ngay trên điểm cất cánh trước khi hạ: mọi lần bay sau đều xuất phát từ gốc.
// Lý do (log 21/11): sai số P01 tăng theo khoảng cách tới gốc (~−0,55 cm lệch Bắc mỗi mét về phía Đông).
// Không phải một phần của bài P01/P02; chỉ là tiện ích thí nghiệm.
#include "uav_research/p01_reference.hpp"
#include <algorithm>
#include <stdexcept>

namespace uav_research
{
inline P01Reference reference_return(double t, const Eigen::Vector3d &from, const Eigen::Vector3d &to,
                                     double duration, const Eigen::Vector3d &heading)
{
    if (!std::isfinite(t) || !std::isfinite(duration) || duration <= 0)
        throw std::invalid_argument("Invalid return timing");
    const double T = duration;
    const double u = std::clamp(t / T, 0.0, 1.0);
    const bool moving = t > 0 && t < T;   // ngoài khoảng [0, T]: đứng yên, mọi đạo hàm = 0
    // s và các đạo hàm theo thời gian (chia T^k cho đạo hàm bậc k).
    const double s = 35*std::pow(u, 4) - 84*std::pow(u, 5) + 70*std::pow(u, 6) - 20*std::pow(u, 7);
    const double s1 = moving ? (140*std::pow(u, 3) - 420*std::pow(u, 4) + 420*std::pow(u, 5) - 140*std::pow(u, 6)) / T : 0.0;
    const double s2 = moving ? (420*u*u - 1680*std::pow(u, 3) + 2100*std::pow(u, 4) - 840*std::pow(u, 5)) / (T*T) : 0.0;
    const double s3 = moving ? (840*u - 5040*u*u + 8400*std::pow(u, 3) - 4200*std::pow(u, 4)) / (T*T*T) : 0.0;
    const double s4 = moving ? (840 - 10080*u + 25200*u*u - 16800*std::pow(u, 3)) / (T*T*T*T) : 0.0;
    const Eigen::Vector3d d = to - from;
    // Hướng mũi giữ cố định như lúc cất cánh: lấy b1 từ reference_hover.
    P01Reference r = reference_hover(from, heading);
    r.x = from + s * d;
    r.v = s1 * d;
    r.a = s2 * d;
    r.j = s3 * d;
    r.snap = s4 * d;
    return r;
}
} // namespace uav_research
