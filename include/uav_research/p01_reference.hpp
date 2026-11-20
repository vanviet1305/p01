
#pragma once
// THAM CHIẾU: mô tả UAV cần ở đâu/chuyển động thế nào; không chứa trạng thái đo.
// Cùng hệ trục với P01State: quán tính có z hướng xuống, trục thân trước-phải-xuống.
// Header chỉ định nghĩa dữ liệu/hàm C++; không có ROS subscriber hay publisher.

#include <Eigen/Dense>
#include <cmath>

namespace uav_research
{

// Các đại lượng mong muốn cấp cho bộ điều khiển, cùng quy ước hệ tọa độ với trạng thái.
// x, v, a: vị trí (m), vận tốc (m/s), gia tốc (m/s²).
// j (jerk), snap: đạo hàm bậc ba và bậc bốn của vị trí (m/s³, m/s⁴).
// b1: hướng trục x thân mong muốn; b1_dot và b1_ddot là các đạo hàm theo thời gian.
struct P01Reference
{
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Vector3d snap;

    Eigen::Vector3d b1;
    Eigen::Vector3d b1_dot;
    Eigen::Vector3d b1_ddot;
};

// Tham chiếu theo dạng Case I, P01 CDC 2010, Mục IV-A, PDF trang 5.
// Đầu vào t (s) có thể là thời gian lý tưởng hoặc pha đã làm chậm từ x500_reference.
// Mã kế thừa dùng w=3.14 gần đúng pi; chưa đổi thành pi chính xác trong phiên này.
// Do đó phải ghi rõ sai khác khi so từng mẫu với bản MATLAB dùng pi.
// Hàm này KHÔNG được chọn trong những lần chạy reference_mode=hover vừa rồi.
inline P01Reference reference_case1(double t)
{
    const double w = 3.14;

    const double s = std::sin(w * t);
    const double c = std::cos(w * t);

    const double w2 = w * w;
    const double w3 = w2 * w;
    const double w4 = w3 * w;

    P01Reference r;

    r.x = Eigen::Vector3d(0.4 * t, 0.4 * s, 0.6 * c);

    r.v = Eigen::Vector3d(0.4, 0.4 * w * c, -0.6 * w * s);

    r.a = Eigen::Vector3d(0.0, -0.4 * w2 * s, -0.6 * w2 * c);

    r.j = Eigen::Vector3d(0.0, -0.4 * w3 * c, 0.6 * w3 * s);

    r.snap = Eigen::Vector3d(0.0, 0.4 * w4 * s, 0.6 * w4 * c);

    r.b1 = Eigen::Vector3d(c, s, 0.0);

    r.b1_dot = Eigen::Vector3d(-w * s, w * c, 0.0);

    r.b1_ddot = Eigen::Vector3d(-w2 * c, -w2 * s, 0.0);

    return r;
}
// Hàm ĐÃ DÙNG trong dry-run và hai lần P01 điều khiển bay treo Gazebo.
// target_position là mục tiêu cố định trong NED, không phải lệnh cất cánh -3 m.
// Node lấy mục tiêu từ vị trí đang bay; hàm này chỉ tạo dữ liệu tham chiếu.
inline P01Reference reference_hover(const Eigen::Vector3d &target_position,
                                    const Eigen::Vector3d &heading)
{
    P01Reference r;

    // Vi tri mong muon
    r.x = target_position;

    // Vi tri co dinh nen cac dao ham bang 0
    r.v = Eigen::Vector3d::Zero();
    r.a = Eigen::Vector3d::Zero();
    r.j = Eigen::Vector3d::Zero();
    r.snap = Eigen::Vector3d::Zero();

    // Chỉ giữ hướng mũi ngang; không sao chép toàn bộ tư thế đang đo làm mục tiêu.
    // Bộ điều khiển vẫn tự dựng Rd nghiêng phù hợp để sửa sai số vị trí ngang.
    r.b1 = heading;
    r.b1.z() = 0.0;

    // Dam bao vector huong hop le
    if (r.b1.norm() < 1e-9)
    {
        r.b1 = Eigen::Vector3d::UnitX();
    }
    else
    {
        r.b1.normalize();
    }

    // Huong mong muon khong thay doi
    r.b1_dot = Eigen::Vector3d::Zero();
    r.b1_ddot = Eigen::Vector3d::Zero();

    return r;
}
} // namespace uav_research
