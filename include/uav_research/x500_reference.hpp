#pragma once
// VAI TRÒ: tạo tham chiếu Case I đã điều chỉnh cho X500, không gửi lệnh PX4.
// Trong hai lần bay treo vừa chạy, reference_mode=hover nên node gọi
// reference_hover() ở p01_reference.hpp; hàm reference_x500_case1() bên dưới
// chưa được gọi. Tệp này có trong bản biên dịch không có nghĩa quỹ đạo đã chạy.
//
// Đầu vào: thời gian quỹ đạo, vị trí/hướng mũi lúc nhận quyền và cấu hình tốc độ.
// Đầu ra: P01Reference gồm vị trí, các đạo hàm và hướng mũi mong muốn.
// Đọc README.md của package để xem toàn bộ đường dữ liệu và kết quả đã chạy.
#include "uav_research/p01_reference.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace uav_research
{
// Tạo pha chuyển động tăng tốc êm bằng tích phân của đa thức chuyển tiếp bậc 7.
// Mảng trả về lần lượt là [pha, đạo hàm 1, đạo hàm 2, đạo hàm 3, đạo hàm 4].
// Pha đóng vai trò thời gian đầu vào của reference_case1(), không phải góc yaw.
// ramp là số giây dùng để đi từ tốc độ pha 0 đến 1; các đạo hàm nối trơn ở hai đầu.
inline std::array<double, 5> phase_ramp(double t, double ramp)
{
    // Trước khởi hành: đứng yên. Sau đoạn tăng tốc: pha tăng đều theo thời gian.
    if (t <= 0) return {{0, 0, 0, 0, 0}};
    if (t >= ramp) return {{t-ramp/2, 1, 0, 0, 0}};
    // Dùng tính đối xứng khi gần cuối đoạn nối để giảm sai số triệt tiêu số học.
    // Việc này hữu ích với đạo hàm bậc cao dùng cho phần bù đón trước của P01.
    if (t > ramp/2)
    {
        const auto reflected = phase_ramp(ramp-t, ramp);
        return {{t-ramp/2+reflected[0], 1-reflected[1], reflected[2],
            -reflected[3], reflected[4]}};
    }
    const double u = t/ramp;
    // Lambda giữ giá trị u để viết gọn các lũy thừa của thời gian chuẩn hóa.
    const auto pow = [u](int n) { return std::pow(u, n); };
    return {{ramp*(7*pow(5)-14*pow(6)+10*pow(7)-2.5*pow(8)),
        35*pow(4)-84*pow(5)+70*pow(6)-20*pow(7),
        (140*pow(3)-420*pow(4)+420*pow(5)-140*pow(6))/ramp,
        (420*pow(2)-1680*pow(3)+2100*pow(4)-840*pow(5))/(ramp*ramp),
        (840*u-5040*pow(2)+8400*pow(3)-4200*pow(4))/(ramp*ramp*ramp)}};
}

// Case I kỹ thuật: dịch về điểm đang bay, quay theo hướng mũi ban đầu, làm chậm,
// tăng/giảm tốc êm và giữ điểm cuối. Đây là đề xuất triển khai, KHÔNG phải ca
// nguyên bản ở Mục IV-A, PDF trang 5 của P01 CDC 2010.
// origin: vị trí NED (m); heading: hướng đầu thân trong mặt phẳng ngang.
// speed=0.25: tốc độ pha bằng 1/4 khi đã hết đoạn tăng tốc, không giảm biên độ.
// ramp=4 s nằm trong duration=30 s; settle_time_s=3 s được trừ ở node gọi hàm.
// Vì thế t có thể âm ở ba giây đầu sau nhận quyền; khi ấy tham chiếu đứng yên.
// rotate_heading=false giữ hướng mũi ban đầu, true cho hướng mũi quay theo pha.
inline P01Reference reference_x500_case1(double t, const Eigen::Vector3d &origin,
    const Eigen::Vector3d &heading, double speed = 0.25, double ramp = 4.0,
    double duration = 30.0, bool rotate_heading = false)
{
    if (!std::isfinite(t) || !std::isfinite(speed) || speed <= 0 ||
        !std::isfinite(ramp) || ramp <= 0 || !std::isfinite(duration) || duration < 2*ramp)
        throw std::invalid_argument("Invalid X500 trajectory timing");
    // Lấy tăng tốc trừ một bản trễ để tạo giảm tốc. Khi t>=duration, pha cố định
    // ở speed*(duration-ramp), mọi đạo hàm bằng 0: giữ vị trí cuối vô thời hạn.
    auto d = phase_ramp(t, ramp);
    const auto stop = phase_ramp(t-(duration-ramp), ramp);
    for (int i = 0; i < 5; ++i) d[i] = speed*(d[i]-stop[i]);
    const auto q = reference_case1(d[0]);
    const auto hold = reference_hover(origin, heading);
    // Các cột của rotation là trục dọc hướng mũi, trục ngang và trục xuống NED.
    // Đây là phép quay bố trí quỹ đạo trong thế giới, không phải tư thế UAV R.
    Eigen::Matrix3d rotation;
    rotation.col(0) = hold.b1;
    rotation.col(1) = Eigen::Vector3d::UnitZ().cross(hold.b1);
    rotation.col(2) = Eigen::Vector3d::UnitZ();
    P01Reference r;
    // Trừ điểm Case I ở pha 0 để tham chiếu bắt đầu chính xác tại origin.
    // Với quy ước z hướng xuống, phần cao độ tương đối 0.6*(cos(w*pha)-1)<=0:
    // tham chiếu không thấp hơn vị trí bắt đầu. Đây không phải kiểm tra va chạm.
    r.x = origin + rotation*(q.x-reference_case1(0).x);
    // Quy tắc đạo hàm hàm hợp: quỹ đạo dùng pha thay cho thời gian trực tiếp.
    // Không chỉ nhân v/a với cùng một hệ số; các hạng d[2..4] cần ở đoạn nối.
    r.v = rotation*(q.v*d[1]);
    r.a = rotation*(q.a*d[1]*d[1]+q.v*d[2]);
    r.j = rotation*(q.j*std::pow(d[1], 3)+3*q.a*d[1]*d[2]+q.v*d[3]);
    r.snap = rotation*(q.snap*std::pow(d[1], 4)+6*q.j*d[1]*d[1]*d[2]+
        3*q.a*d[2]*d[2]+4*q.a*d[1]*d[3]+q.v*d[4]);
    // Hướng mũi và hai đạo hàm đi cùng tham chiếu để tính Omega_d, Omega_dot_d.
    r.b1 = rotate_heading ? Eigen::Vector3d(rotation*q.b1) : hold.b1;
    r.b1_dot = rotate_heading ? Eigen::Vector3d(rotation*q.b1_dot*d[1]) : Eigen::Vector3d::Zero();
    r.b1_ddot = rotate_heading ? Eigen::Vector3d(rotation*(q.b1_ddot*d[1]*d[1]+q.b1_dot*d[2])) : Eigen::Vector3d::Zero();
    return r;
}
} // namespace uav_research
