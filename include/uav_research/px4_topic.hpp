#pragma once
// TÊN TOPIC PX4: chọn hậu tố _vN theo MESSAGE_VERSION của kiểu thông điệp đã build.
// Ví dụ lần chạy hiện tại: odometry không hậu tố; vehicle_status_v5, vehicle_command_v1.
// Hàm này chỉ tạo tên kênh; không chuyển đổi dữ liệu giữa các phiên bản message.
#include <string>
#include <type_traits>

namespace uav_research
{
// Template chọn mặc định 0 khi kiểu T cũ không khai báo MESSAGE_VERSION.
template<typename T, typename = void> struct MessageVersion
{
    static constexpr unsigned value = 0;
};
// Chuyên biệt hóa khi T có hằng MESSAGE_VERSION; std::void_t cần chuẩn C++17.
template<typename T> struct MessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>>
{
    static constexpr unsigned value = T::MESSAGE_VERSION;
};
// MESSAGE_VERSION=0 dùng tên gốc; khác 0 thêm _vN. Có thể đổi tên qua ROS remapping
// khi cầu nối dùng tên riêng, nhưng vẫn phải bảo đảm kiểu thông điệp tương thích.
template<typename T> std::string px4_topic(const std::string &base)
{
    return MessageVersion<T>::value == 0 ? base : base+"_v"+std::to_string(MessageVersion<T>::value);
}
} // namespace uav_research
