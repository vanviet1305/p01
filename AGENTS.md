# Hướng dẫn cho trợ lý AI (Claude Code, Codex, Cursor, Copilot…)

Tệp này dành cho trợ lý AI giúp người dùng **cài đặt và chạy lại** package `p01`. Người dùng là sinh viên làm đồ án chung, có thể chưa quen ROS 2/PX4. Trả lời và comment bằng **tiếng Việt**, hướng dẫn **từng bước nhỏ**, mỗi lần một lệnh, và chờ người dùng báo kết quả rồi mới sang bước tiếp.

## Dự án là gì

- Bộ điều khiển hình học P01 (Lee, Leok, McClamroch, *Geometric tracking control of a quadrotor UAV on SE(3)*, CDC 2010), viết bằng C++, điều khiển drone X500 trong PX4 SITL + Gazebo qua ROS 2 Humble.
- Node chính là `p01_offboard_hover`. Node tự xin Offboard, arm, tăng dần lực rotor, cất cánh 2 m, rồi hover hoặc bám quỹ đạo Case I. Node gửi lệnh motor trực tiếp qua `ActuatorMotors`; PX4 không chạy vòng vị trí/tư thế của nó.
- Giải thích code: [README.md](./README.md) mục 1–8. Lệnh chạy: README mục 9. Cài đặt: [CAI_DAT.md](./CAI_DAT.md). Kết quả mẫu: [ket_qua_mau/](./ket_qua_mau/README.md).

## Thứ tự công việc khi người dùng nhờ "chạy lại"

1. Kiểm tra môi trường, chỉ đọc, chưa cài gì: `lsb_release -a`, `ls /opt/ros`, `git -C ~/PX4-Autopilot rev-parse HEAD`, `ls ~/ros2_px4_ws/src`, `git -C ~/ros2_px4_ws/src/px4_msgs rev-parse HEAD`. So với bảng phiên bản ở CAI_DAT.md mục 0, báo người dùng phần nào thiếu hoặc lệch.
2. Cài phần thiếu theo CAI_DAT.md, đúng thứ tự mục 1 → 5. Các lệnh `sudo` hoặc cài đặt hệ thống phải hỏi người dùng trước.
3. Build và chạy test: CAI_DAT.md mục 3–4.
4. Chạy mô phỏng theo README mục 9. Có 8 tab terminal **do người dùng tự mở**: Agent, PX4/Gazebo và QGC là tiến trình chạy lâu, cần người dùng nhìn màn hình. Đưa lệnh từng tab và hỏi người dùng thấy gì.
5. Phân tích log CSV bằng `tools/plot_flight_log.py`, rồi so với `ket_qua_mau/`.

## Quy tắc an toàn (bắt buộc)

- **Luôn chạy dry-run** (`-p enable_actuation:=false`) và kiểm tra dòng log đạt điều kiện (README Bước 7) **trước mọi lần** `enable_actuation:=true`.
- Trước mỗi lần chạy, PX4 phải ở **Hold + disarmed**. Nếu không, ở `pxh>` gõ `commander mode auto:loiter`, rồi `commander disarm`.
- Kết thúc bằng **Land trên QGC**, chờ chạm đất và disarm rồi mới Ctrl+C. Không dùng Ctrl+C để dừng khi drone đang bay.
- Mỗi lần chạy đặt **tên log mới**, không ghi đè log cũ.
- Code chỉ dùng cho **mô phỏng**. Không hướng dẫn nạp hoặc chạy trên drone thật.

## Quy tắc trung thực

- Không nói "đã chạy được" hay "đã tái hiện kết quả" khi chưa có log CSV/terminal làm bằng chứng. Nếu chỉ mới đọc code hay đối chiếu bài báo thì nói rõ là vậy.
- Khi báo kết quả, ghi rõ tên log, tham số chạy, và số liệu (RMS, max sai số, có bão hòa không).
- Kết quả Gazebo **không** kế thừa chứng minh ổn định của bài báo, vì có giới hạn motor, hệ số theo trục và thông số X500 khác bài báo (README mục 6).

## Không được tự ý làm

- Đổi commit PX4 hoặc px4_msgs (sẽ lệch định nghĩa bản tin).
- Sửa hệ số điều khiển, thông số X500 hoặc ngưỡng an toàn trong `include/uav_research/*.hpp` khi người dùng chỉ nhờ chạy lại. Muốn sửa phải hỏi trước và giải thích ảnh hưởng.
- Đổi tên package `p01` hoặc thư mục `include/uav_research/`. Tên `uav_research` là namespace cũ, **không** phải package ROS cần cài.
- Commit hoặc push lên Git khi người dùng chưa yêu cầu.

## Kết quả mong đợi (đối chiếu)

| Lần chạy | Tham số | Kết quả trên máy tác giả |
|---|---|---|
| dry-run | `enable_actuation:=false`, `reference_mode:=hover` | `status=1 odom=1 valid=1 rate_ok=1 land_fresh=1 ground_ready=1 enabled=0`, không phát lệnh motor |
| hover | README Bước 8, lệnh hover | Stage 1→2→5→6→7→3; sai số vị trí khi hover: RMS ≈ 1,8 cm, max ≈ 3,5 cm; không bão hòa |
| Case I 0,25× | README Bước 8, lệnh Case I | Đoạn di chuyển 10–40 s: RMS ≈ 3,8 cm, max ≈ 6,8 cm; giữ điểm cuối RMS ≈ 2,0 cm; không bão hòa |

Sai lệch vài mm so với bảng là bình thường. Nếu sai số lớn hơn khoảng 2 lần, hoặc có bão hòa (`saturated=1`), hãy dừng và kiểm tra lại phiên bản PX4/px4_msgs và tham số trước khi sửa code.

## Lệnh hay dùng

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_px4_ws/install/setup.bash
cd ~/ros2_px4_ws && colcon build --packages-select p01 --symlink-install
colcon test --packages-select p01 --ctest-args -R ground_takeoff_core && colcon test-result --verbose
python3 -I ~/ros2_px4_ws/src/p01/tools/plot_flight_log.py --csv <log.csv> --output <thu_muc> --motion-start 10 --motion-end 40
```
