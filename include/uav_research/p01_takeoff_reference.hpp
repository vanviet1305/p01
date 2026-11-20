#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include "uav_research/p01_reference.hpp"

namespace uav_research
{

// Tạo mục tiêu cất cánh cho P01, không gửi lệnh tới PX4.
//
// t: thời gian kể từ lúc bắt đầu nâng UAV, đơn vị giây.
// origin: vị trí NED lúc chuẩn bị cất cánh.
// height: độ cao tăng thêm so với origin, đơn vị mét.
// duration: thời gian đi hết đoạn lên cao, đơn vị giây.
inline P01Reference reference_takeoff(
    double t,
    const Eigen::Vector3d &origin,
    const Eigen::Vector3d &heading,
    double height,
    double duration)
{
    if (!std::isfinite(t) ||
        !origin.allFinite() || !heading.allFinite() ||
        !std::isfinite(height) || height <= 0.0 ||
        !std::isfinite(duration) || duration <= 0.0)
    {
        throw std::invalid_argument("Invalid takeoff reference");
    }

    // Ban đầu đứng yên tại origin, giữ hướng mũi.
    auto r = reference_hover(origin, heading);

    if (t <= 0.0)
    {
        return r;
    }

    if (t >= duration)
    {
        // NED có z hướng xuống: lên cao thì giảm z.
        r.x.z() -= height;
        return r;
    }

    const double u = t / duration;

    // Đa thức chuyển tiếp bậc 9.
    // Vị trí đi từ 0 đến 1; đạo hàm bậc 1..4 bằng 0 ở hai đầu.
    const std::array<double, 10> coefficients{
        0, 0, 0, 0, 0, 126, -420, 540, -315, 70
    };

    // d[k] là đạo hàm bậc k theo thời gian thực t.
    std::array<double, 5> d{};

    for (int k = 0; k <= 4; ++k)
    {
        double value = 0.0;

        // Tính đa thức đạo hàm bằng cách nhân lồng nhau.
        for (int n = 9; n >= k; --n)
        {
            double coefficient = coefficients[n];

            for (int j = 0; j < k; ++j)
            {
                coefficient *= n - j;
            }

            value = value * u + coefficient;
        }

        d[k] = value / std::pow(duration, k);
    }

    r.x.z() -= height * d[0];
    r.v.z() = -height * d[1];
    r.a.z() = -height * d[2];
    r.j.z() = -height * d[3];
    r.snap.z() = -height * d[4];

    return r;
}

} // namespace uav_research