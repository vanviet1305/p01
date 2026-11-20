# Package p01 — P01 điều khiển X500 trong Gazebo

**Cập nhật 20/11/2026.** Tệp này giải thích code và các bước chạy (**mục 9**). Cài đặt từ máy trắng và phiên bản đã khóa: [CAI_DAT.md](./CAI_DAT.md). Hướng dẫn cho trợ lý AI: [AGENTS.md](./AGENTS.md). Log và đồ thị mẫu để đối chiếu: [ket_qua_mau/](./ket_qua_mau/).

**Node làm gì:** từ mặt đất, tự yêu cầu Offboard → arm → tăng dần lực rotor → P01 cất cánh 2 m/6 s → giữ vị trí (hover) hoặc bám Case I. Mặc định dry-run (chỉ tính, không phát lệnh); phải thêm `-p enable_actuation:=true` mới điều khiển motor. Chưa có tự hạ cánh, quỹ đạo tròn/số 8, P02.

## 1. Các thành phần tham gia

| Thành phần | Vai trò |
|---|---|
| Gazebo + mô hình X500 | Mô phỏng vật lý: thân UAV, 4 rotor, cảm biến |
| PX4 SITL (PX4 chạy trên máy tính) | Ước lượng trạng thái, quản lý arm/chế độ bay/sự cố, đưa lệnh tới motor |
| Micro XRCE-DDS Agent | Cầu nối bản tin giữa PX4 và ROS 2; không tính toán gì |
| px4_msgs | Định nghĩa cấu trúc bản tin PX4 cho C++ |
| Node `p01_offboard_hover` | Nhận trạng thái → tạo tham chiếu → P01 tính f, M → phân bổ rotor → phát lệnh |
| Node `p01_path_visualizer` | Vẽ đường bay thực và tham chiếu trong RViz2 |
| QGroundControl (QGC) | Giám sát, chọn Land/Hold; không tính P01 |

```
Gazebo ──cảm biến──► PX4 (ước lượng) ──VehicleOdometry──► Agent ──► node p01
   ▲                                                                  │
   └────── motor ◄── PX4 (xuất motor) ◄── ActuatorMotors ◄── Agent ◄──┘
```

Chế độ Offboard dùng `direct_actuator` (điều khiển trực tiếp cơ cấu chấp hành): node tự tính 4 lệnh rotor, PX4 bỏ qua vòng vị trí/tư thế của nó. Khác với bài Offboard cũ gửi `TrajectorySetpoint` (điểm đặt tọa độ) để PX4 tự điều khiển.

## 2. Bản đồ code — đọc theo thứ tự này

| Tệp | Đầu vào → đầu ra | Dùng ROS? |
|---|---|---|
| [src/p01_offboard_hover.cpp](./src/p01_offboard_hover.cpp) | Node chính: nối ROS/PX4 với các khối bên dưới | Có |
| [p01_ground_session.hpp](./include/uav_research/p01_ground_session.hpp) | Cờ trạng thái PX4/mặt đất → stage, quyền phát motor, lệnh Offboard/arm, hệ số trộn | Không |
| [p01_takeoff_reference.hpp](./include/uav_research/p01_takeoff_reference.hpp) | Thời gian → tham chiếu cất cánh bậc 9, đủ đạo hàm tới snap | Không |
| [p01_reference.hpp](./include/uav_research/p01_reference.hpp) | Kiểu dữ liệu tham chiếu; `reference_hover()` giữ một điểm | Không |
| [x500_reference.hpp](./include/uav_research/x500_reference.hpp) | Thời gian → Case I chỉnh cho X500 (chậm, tăng/giảm tốc êm) | Không |
| [p01_geometric_controller.hpp](./include/uav_research/p01_geometric_controller.hpp) | **Lõi P01**: trạng thái + tham chiếu → lực f (N), mômen M (N·m) | Không |
| [x500_control.hpp](./include/uav_research/x500_control.hpp) | Thông số X500; f, M → lực 4 rotor có giới hạn → lệnh chuẩn hóa u0..u3 | Không |
| [p01_telemetry.hpp](./include/uav_research/p01_telemetry.hpp) | Phát `/p01/*` (vị trí, tham chiếu, sai số) cho rqt_plot | Có |
| [px4_topic.hpp](./include/uav_research/px4_topic.hpp) | Ghép hậu tố phiên bản vào tên topic PX4 | Có |
| [src/p01_simple.cpp](./src/p01_simple.cpp) | Node học: cùng lõi, không lớp an toàn, chạy theo thời gian, tự hạ cánh/disarm; chỉ dùng mô phỏng (`ros2 run p01 p01_simple --ros-args -p reference_mode:=hover` hoặc `case1`) | Có |
| [src/p01_path_visualizer.cpp](./src/p01_path_visualizer.cpp) | Đổi NED→ENU, lưu 2 Path cho RViz2 | Có |
| [test/test_ground_takeoff.cpp](./test/test_ground_takeoff.cpp) | Kiểm tra máy trạng thái, đạo hàm tham chiếu, trộn lực; không cần Gazebo | Không |
| [tools/plot_flight_log.py](./tools/plot_flight_log.py) | CSV → PNG/PDF/summary.json | Không |
| [CMakeLists.txt](./CMakeLists.txt), [package.xml](./package.xml) | Khai báo build và phụ thuộc cho colcon | — |

Ý chính: chỉ các tệp có cột "Có" dính tới ROS. Toán học nằm trong header C++ thuần, nên kiểm tra được không cần Gazebo; làm P02 chỉ cần thêm một header luật điều khiển. Tên include/namespace `uav_research` giữ từ mã tham khảo cũ, không liên quan package ROS `uav_research`.

## 3. Các hàm trong node chính

| Hàm | Khi nào chạy | Làm gì |
|---|---|---|
| `main()` | Một lần | Tạo node, `spin` (chờ và gọi callback), bắt lỗi |
| Constructor | Một lần khi tạo node | Đọc tham số, tạo publisher (chỉ khi bật actuation), subscriber, mở CSV, tạo timer 100 ms |
| `odometry()` | Mỗi mẫu odometry (~100 Hz) | **Vòng điều khiển**, xem mục 4 |
| Callback status | Mỗi bản tin VehicleStatus | Lưu mode/armed/failsafe, gọi `advance()` |
| Callback land / ack | Khi có bản tin | Lưu cờ chạm đất / in kết quả lệnh |
| Timer 100 ms | 10 Hz | `advance()`, gửi heartbeat Offboard (PX4 cần ≥2 Hz), gửi motor, ghi CSV ra đĩa |
| `advance()` | Gọi từ 3 nơi trên | Gom cờ vào `GroundInputs` → `session_.step()` → làm theo `GroundActions` |
| `publish_motors()` | Khi được phép | Trộn lực từ mức tối thiểu sang P01 theo `control_blend`, phân bổ, gửi `ActuatorMotors` |
| `command_mode()` / `command_arm()` | Khi máy trạng thái yêu cầu | Gửi VehicleCommand chuyển chế độ / arm thường (không force) |

## 4. Lần theo một mẫu odometry

Trong `odometry()`, chú thích A–G:

- **A.** Chỉ nhận hệ NED (Bắc–Đông–Xuống; z âm là ở trên cao).
- **B.** Chép vào `P01State`: `x`, `v` (NED), `omega` (hệ thân FRD: trước–phải–xuống), quaternion → ma trận `R`. PX4 xếp quaternion theo w, x, y, z, trùng thứ tự Eigen. Loại mẫu có NaN, mẫu cũ, khoảng cách >50 ms (`rate_ok_=false`), hoặc PX4 đổi gốc (`reset_counter`).
- **C.** Khi Waiting, chốt gốc `origin_` và hướng mũi `heading_` theo vị trí hiện tại; từ Priming giữ cố định.
- **D.** `current_reference()`: trước cất cánh giữ gốc; Climbing theo `reference_takeoff()`; Active với `case1` theo `reference_x500_case1()`, bắt đầu sau `settle_time_s`.
- **E.** `compute_p01_control_with_reference()` tính f, M.
- **F.** `allocate_x500()` chia cho 4 rotor: ưu tiên lực tổng, rồi mômen nghiêng, cuối cùng mômen quay mũi.
- **G.** `advance()` hỏi máy trạng thái; chỉ khi `actions.motors=true` mới `publish_motors()`. Sau đó phát telemetry, in log (1 lần/giây), ghi CSV.

## 5. Máy trạng thái (stage)

| stage | Tên | Việc làm | Motor |
|---|---|---|---|
| 0 | Waiting | Chờ dữ liệu đủ, UAV trên đất, chưa arm, nằm ngang | Không phát |
| 1 | Priming | Gửi heartbeat ít nhất 1,2 s | Mức thấp |
| 2 | Requested | Yêu cầu Offboard, chờ xác nhận ≤3 s | Mức thấp |
| 5 | Arming | Yêu cầu arm, chờ ≤3 s | Mức thấp |
| 6 | Spooling | Trộn từ mức tối thiểu sang P01 trong 2 s | blend 0→1 |
| 7 | Climbing | Bám tham chiếu cất cánh, tối đa 20 s | P01 |
| 3 | Active | Lên đủ, ổn định 1 s (sai số <0,15 m, v <0,2 m/s, ω <0,3 rad/s); hover hoặc Case I | P01 |
| 4 | Stopped | PX4 thoát Offboard, lỗi, hoặc xung đột | Khóa, không tự nhận lại |

- Điều kiện bắt đầu: PX4 phải **disarm và không ở Offboard**. Nếu QGC còn hiện Offboard thì gõ ở `pxh>`: `commander mode auto:loiter`, rồi `commander disarm`.
- `Command ACK result=0` chỉ nghĩa là lệnh được nhận; node vẫn chờ VehicleStatus xác nhận.
- `motor_stream=ON` nghĩa là node đang gửi lệnh motor, không khẳng định PX4 đã arm.
- Hover không tự dừng; `trajectory_duration_s` chỉ là thời gian di chuyển của Case I, hết thì giữ điểm cuối. Kết thúc bằng Land trên QGC.

## 6. Liên hệ với bài CDC 2010 và phần riêng của X500

| Khối trong code | Bài CDC 2010 (Lee, Leok, McClamroch) |
|---|---|
| Sai số `ex`, `ev` | (6)–(7), Mục III-A, PDF tr. 3 |
| `Psi`, `eR`, `eOmega` | (8), (10)–(11) |
| Tư thế mong muốn `Rd` | (12)–(13), Mục III-B |
| Lực f, mômen M | (15)–(16) |
| `reference_case1()` | Mục IV-A, PDF tr. 5 |

| Phần riêng của dự án | Ghi chú |
|---|---|
| Thông số X500 | ~2,06 kg, lực bay treo ~20,25 N (bài báo: 4,34 kg) |
| Hệ số | kx = 4·m, kv = 3,2·m; tư thế theo trục [1,5; 1,5; 0,35]; tốc độ góc [0,35; 0,35; 0,22] (bài báo dùng hệ số vô hướng) |
| Phân bổ có giới hạn, máy trạng thái, cất cánh bậc 9, Case I chậm | Thiết kế kỹ thuật, không có trong bài báo |

Vì có giới hạn motor và hệ số theo trục, kết quả Gazebo không kế thừa trực tiếp chứng minh ổn định của bài báo.

## 7. Các cột CSV

| Cột | Nghĩa |
|---|---|
| `t` | Giây kể từ lúc bắt đầu Climbing (0 trước đó và khi dry-run) |
| `x,y,z` / `xd,yd,zd` | Vị trí thực / tham chiếu, NED, m |
| `position_error` | Chuẩn vector sai số 3 chiều, m (dòng log in là `ex`) |
| `Psi` | Hàm sai số tư thế, không phải góc |
| `f_requested`, `Mx,My,Mz` | Lực/mômen P01 yêu cầu (N, N·m) |
| `f_achieved`, `M*_achieved` | Tính lại sau phân bổ có giới hạn; không phải lực đo |
| `u0..u3` | 4 lệnh chuẩn hóa gửi vào ActuatorMotors (u=0,73 không phải 73% lực) |
| `saturated` | 1 nếu chạm giới hạn rotor |
| `stage` | Như mục 5; phân tích P01 dùng stage=3 (và 7 cho cất cánh) |
| `control_blend` | Mức trộn 0→1 khi Spooling |

## 8. Cú pháp C++ hay gặp

| Cú pháp | Nghĩa |
|---|---|
| `class A : public rclcpp::Node` | Kế thừa: node có sẵn các hàm ROS 2 |
| `using Odom = ...` | Đặt tên ngắn cho kiểu, giống `typedef` |
| `parameter<bool>(...)` | Hàm mẫu (template): kiểu trong `<>` là kiểu trả về |
| `[this](...) { ... }` | Lambda: hàm viết tại chỗ, dùng làm callback |
| `SharedPtr`, `->` | Con trỏ thông minh, tự giải phóng; `->` gọi hàm qua con trỏ |
| `const T &` | Truyền tham chiếu, không sao chép, không sửa |
| `auto` | Trình biên dịch tự suy kiểu |
| `try { } catch { }`, `throw` | Bắt lỗi / ném lỗi |
| `Eigen::Vector3d`, `Matrix3d` | Vector 3 phần tử, ma trận 3×3 |
| `#pragma once`, `namespace`, `inline` | Chống include lặp, gom tên, cho phép định nghĩa hàm trong header |

## 9. Các bước chạy (từ build tới vẽ đồ thị)

Mỗi tab Terminal dùng ROS 2 phải chạy 2 dòng source trước. Repo phải nằm ở `~/ros2_px4_ws/src/p01` (xem [CAI_DAT.md](./CAI_DAT.md)).

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_px4_ws/install/setup.bash
```

### Bước 0 — Build (chỉ khi vừa sửa .cpp/.hpp/CMake)

```bash
cd ~/ros2_px4_ws
```

```bash
colcon build --packages-select p01 --symlink-install
```

```bash
source ~/ros2_px4_ws/install/setup.bash
```

Chờ Summary báo p01 không Failed. Chỉ đổi tham số `-p` thì không cần build.

### Bước 1 — Tab 1: Agent (cầu nối PX4 ↔ ROS 2)

```bash
~/ros2_px4_ws/install/microxrcedds_agent/bin/MicroXRCEAgent udp4 -p 8888
```

### Bước 2 — Tab 2: PX4 + Gazebo

```bash
cd ~/PX4-Autopilot
```

```bash
make px4_sitl gz_x500
```

Nhấn Enter để có `pxh>`, kiểm tra kết nối Agent:

```bash
uxrce_dds_client status
```

### Bước 3 — Tab 3: QGroundControl

```bash
~/QGroundControl-5.0.8.AppImage
```

QGC phải hiện **Hold** và **disarmed**. Nếu còn **Offboard** (do lần chạy trước), gõ ở `pxh>` của Tab 2:

```bash
commander mode auto:loiter
```

```bash
commander disarm
```

### Bước 4 — Tab 4: kiểm tra có dữ liệu PX4

```bash
ros2 topic echo /fmu/out/vehicle_odometry --qos-reliability best_effort --once
```

Có in bản tin là đạt.

### Bước 5 — Tab 5, 6, 7: hiển thị (mở trước P01)

Tab 5, Path visualizer (chạy lại mỗi phiên để xóa đường cũ):

```bash
ros2 run p01 p01_path_visualizer
```

Tab 6, RViz2:

```bash
rviz2 -d ~/ros2_px4_ws/src/p01/p01_paths.rviz
```

Tab 7, rqt_plot (độ cao z thực và tham chiếu):

```bash
PYTHONNOUSERSITE=1 ros2 run rqt_plot rqt_plot /p01/position_ned/point/z /p01/reference_ned/point/z
```

### Bước 6 — Tab 8: vào thư mục log

```bash
mkdir -p ~/p01_logs && cd ~/p01_logs
```

Mỗi lần chạy đặt **tên mới** (ví dụ `p01_ground_hover_03`), không dùng lại tên cũ để tránh ghi đè.

### Bước 7 — Tab 8: dry-run (luôn chạy trước)

```bash
ros2 run p01 p01_offboard_hover --ros-args -p enable_actuation:=false -p reference_mode:=hover -p log_csv:="$PWD/p01_ground_dryrun_02.csv" 2>&1 | tee p01_ground_dryrun_02_terminal.log
```

Đạt khi log `Waiting/dry-run` có `status=1 odom=1 valid=1 rate_ok=1 land_fresh=1 ground_ready=1 enabled=0`. Dừng bằng Ctrl+C.

### Bước 8 — Tab 8: chạy thật (chọn MỘT)

Hover:

```bash
ros2 run p01 p01_offboard_hover --ros-args -p enable_actuation:=true -p reference_mode:=hover -p takeoff_height_m:=2.0 -p takeoff_duration_s:=6.0 -p log_csv:="$PWD/p01_ground_hover_03.csv" 2>&1 | tee p01_ground_hover_03_terminal.log
```

Case I chậm 0,25×:

```bash
ros2 run p01 p01_offboard_hover --ros-args -p enable_actuation:=true -p reference_mode:=case1 -p takeoff_height_m:=2.0 -p takeoff_duration_s:=6.0 -p trajectory_speed:=0.25 -p trajectory_ramp_s:=4.0 -p trajectory_duration_s:=30.0 -p settle_time_s:=3.0 -p trajectory_rotate_heading:=false -p log_csv:="$PWD/p01_ground_case1_02.csv" 2>&1 | tee p01_ground_case1_02_terminal.log
```

Theo dõi dòng `Session stage=...`: 1 → 2 → 5 → 6 → 7 → 3. Đến stage=3 là P01 đang giữ vị trí/bám quỹ đạo. Case I: bay từ khoảng t=10 s tới t=40 s rồi giữ điểm cuối.

### Bước 9 — Kết thúc

1. QGC chọn **Land**. Node thấy PX4 thoát Offboard → stage=4 (dự kiến).
2. Đợi UAV chạm đất và **disarm**.
3. Ctrl+C ở Tab 8. Hộp thoại Ubuntu "ros2 stopped unexpectedly" nếu hiện thì bấm Don't send, vô hại.
4. Trước lần chạy sau: kiểm tra lại Bước 3 (Hold, disarmed) và chạy lại Tab 5.

Không dùng Ctrl+C khi UAV đang bay để thay cho Land.

### Bước 10 — Vẽ đồ thị (Case I)

Vẫn ở thư mục `gazebo_logs`:

```bash
python3 -I ~/ros2_px4_ws/src/p01/tools/plot_flight_log.py --csv p01_ground_case1_02.csv --output do_thi/ground_case1_02 --motion-start 10.0 --motion-end 40.0
```

`--motion-start/--motion-end` là đoạn di chuyển theo cột `t` (Case I 0,25×: 10–40 s). Công cụ chỉ đọc CSV, tính lại sai số, ghi SHA-256 nguồn vào summary.json và xuất 3 PNG + 1 PDF. So với kết quả mẫu trong [ket_qua_mau/](./ket_qua_mau/). Dùng `python3 -I` để bỏ qua NumPy cài bằng pip trong `~/.local` (NumPy 2.x xung đột với Matplotlib của Ubuntu 22.04).
