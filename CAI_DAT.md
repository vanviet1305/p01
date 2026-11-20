# Cài đặt từ máy trắng để chạy lại P01

Mục tiêu: dựng lại **đúng môi trường** đã bay P01 thành công (20/11/2026), rồi chạy theo [README.md](./README.md) mục 9.
Mỗi khối lệnh dưới đây chạy được độc lập; làm lần lượt từ trên xuống.

## 0. Phiên bản đã khóa

| Thành phần | Phiên bản | Ghi chú |
|---|---|---|
| Hệ điều hành | Ubuntu 22.04 (x86_64) | Phiên bản khác chưa thử |
| ROS 2 | Humble | Cài qua apt |
| PX4-Autopilot | commit `cf0797e4440e23d3e5393f197c156477cb4b5fdc` (v1.18.0-beta1-1260) | Không sửa mã PX4 |
| px4_msgs | commit `2e892300d8c20d3c29e198e324a10854b40ab8d3` | **Phải khớp PX4**, nếu lệch thì topic không nói chuyện được |
| Micro-XRCE-DDS-Agent | v2.4.2, commit `57d086216d01ec43121845d385894a25987f8a2c` | Build trong workspace |
| Gazebo Sim | 8.x (Harmonic), đã dùng 8.15.0 | Do script PX4 cài |
| QGroundControl | 5.0.8 | AppImage |

Các commit của px4_msgs và Agent nằm trong [deps.repos](./deps.repos), cài bằng `vcs import`.

## 1. ROS 2 Humble và công cụ

Cài ROS 2 Humble bản desktop theo hướng dẫn chính thức: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Sau đó cài thêm công cụ build và thư viện mà p01 cần:

```bash
sudo apt update && sudo apt install -y python3-colcon-common-extensions python3-vcstool python3-rosdep libeigen3-dev ros-humble-eigen3-cmake-module ros-humble-rqt-plot python3-numpy python3-matplotlib git
```

## 2. PX4-Autopilot đúng commit (kèm Gazebo)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
```

```bash
cd ~/PX4-Autopilot && git checkout cf0797e4440e23d3e5393f197c156477cb4b5fdc && git submodule update --init --recursive
```

Script của PX4 cài toolchain và Gazebo Harmonic. Chạy xong thì **khởi động lại máy**.

```bash
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
```

Build và mở thử mô phỏng. Lần đầu mất khoảng 5–15 phút; thấy dấu nhắc `pxh>` và cửa sổ Gazebo có drone X500 là đạt.

```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

## 3. Workspace ROS 2: p01 + px4_msgs + Agent

**Đường dẫn bắt buộc:** repo này phải nằm ở `~/ros2_px4_ws/src/p01`, vì README dùng đường dẫn đó.

```bash
mkdir -p ~/ros2_px4_ws/src && cd ~/ros2_px4_ws/src
```

```bash
git clone <URL_REPO_NAY> p01
```

```bash
vcs import < p01/deps.repos
```

Build lần đầu (px4_msgs khá lâu, khoảng 5–10 phút):

```bash
source /opt/ros/humble/setup.bash && cd ~/ros2_px4_ws && colcon build
```

```bash
source ~/ros2_px4_ws/install/setup.bash
```

Đạt khi Summary báo 3 package (p01, px4_msgs, microxrcedds_agent) và không có Failed.

## 4. Kiểm tra không cần mô phỏng

Bài test thuần C++, không phát lệnh tới PX4:

```bash
cd ~/ros2_px4_ws && colcon test --packages-select p01 --ctest-args -R ground_takeoff_core && colcon test-result --verbose
```

Đạt khi `ground_takeoff_core` báo passed. Các test lint (`ament_*`) có thể báo lỗi định dạng; chúng không ảnh hưởng chạy bay.

## 5. QGroundControl 5.0.8

Tải `QGroundControl-x86_64.AppImage` bản **v5.0.8** tại https://github.com/mavlink/qgroundcontrol/releases/tag/v5.0.8, đổi tên thành `~/QGroundControl-5.0.8.AppImage`, rồi:

```bash
chmod +x ~/QGroundControl-5.0.8.AppImage
```

Nếu AppImage không chạy được, cài `libfuse2` (`sudo apt install libfuse2`). Xem thêm hướng dẫn cài QGC trên Ubuntu trong tài liệu chính thức của QGC (thêm user vào nhóm `dialout`, gỡ `modemmanager`).

## 6. Chạy

Làm theo [README.md](./README.md) **mục 9**, Bước 1 → Bước 10. Luôn chạy dry-run (Bước 7) trước khi bật actuation.

So sánh kết quả của bạn với [ket_qua_mau/](./ket_qua_mau/README.md). Gazebo không tất định tuyệt đối, nên số liệu sẽ **gần giống** chứ không trùng từng mẫu.

## 7. Lỗi hay gặp

| Triệu chứng | Nguyên nhân thường gặp | Cách xử lý |
|---|---|---|
| `ros2 topic echo /fmu/out/vehicle_odometry` không in gì | Agent chưa chạy hoặc PX4 chưa kết nối | Chạy Agent trước PX4; ở `pxh>` gõ `uxrce_dds_client status` |
| Có topic nhưng node báo thiếu dữ liệu/sai kiểu | px4_msgs lệch commit PX4 | Kiểm tra lại commit ở mục 0, build lại px4_msgs |
| Node dừng ở stage 0, `ground_ready=0` | PX4 đang Offboard/armed từ lần trước | Ở `pxh>`: `commander mode auto:loiter`, rồi `commander disarm` |
| Bị từ chối Offboard/arm | QGC chưa mở hoặc PX4 chưa sẵn sàng (chờ GPS/EKF) | Chờ QGC báo Ready to fly rồi chạy lại |
| `plot_flight_log.py` lỗi `_ARRAY_API not found` | NumPy 2.x trong `~/.local` xung đột Matplotlib của Ubuntu | Chạy bằng `python3 -I ...` như README |
| `colcon build` không tìm thấy Eigen3 | Thiếu gói | `sudo apt install libeigen3-dev ros-humble-eigen3-cmake-module` |
