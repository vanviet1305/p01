# Hướng dẫn bay mô phỏng P01/P02

Cập nhật 22/11/2026. Chỉ gồm lệnh chạy node bay để trình diễn (không ghi CSV).
Agent, PX4, RViz chạy như ghi chú terminal đã lưu.

---

## 1. Cấu trúc câu lệnh

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 \
  -p case1_speed:=0.5 \
  -p motor_k_scale:=1.1 \
  -p state_source:=gazebo
```
- `ros2 run p01`: chạy một node trong package `p01`.
- `p01_simple` / `p02_simple`: bộ điều khiển P01 / P02.
- `--ros-args`: sau đây là tham số cho node.
- `-p tên:=giá_trị`: đặt một tham số. Không ghi thì dùng mặc định.
- Dấu `\` cuối dòng: lệnh còn tiếp ở dòng dưới. Chép cả khối là chạy được.

Ý nghĩa từng tham số: xem mục 5.

Muốn lưu dữ liệu ra CSV: thêm dòng `-p log_csv:=tên_tệp.csv`
(tệp lưu vào thư mục đang đứng).

---

## 2. Bay bằng trạng thái thật (Gazebo)

Bộ điều khiển nhận vị trí, vận tốc, tư thế thật từ Gazebo.
Dùng để so sánh P01 với P02.

### Chuẩn bị

Sau mỗi lần khởi động lại PX4, gắn odometry cho X500:

```bash
bash ~/ros2_px4_ws/src/p01/tools/gan_odom_gazebo.sh
``

Chạy một lệnh, chờ dòng `Đã hạ cánh và disarm, kết thúc node`
rồi mới chạy lệnh tiếp.

### A — P01, 0,5×, mô hình đúng

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p state_source:=gazebo
```

### B — P02, 0,5×, khai sai 1,8 kg

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

### B0 — P01, 0,5×, khai sai 1,8 kg

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

### C — P01, 1×, mô hình đúng

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=1.0 \
  -p state_source:=gazebo
```

### D — P02, 1×, khai sai 1,8 kg

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=1.0 \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

### E — P01, 0,5×, motor yếu 10%

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p motor_k_scale:=1.1 \
  -p state_source:=gazebo
```

### F — P02, 0,5×, motor yếu 10%

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p motor_k_scale:=1.1 \
  -p state_source:=gazebo
```

---

## 3. Bay bằng EKF2

Bộ điều khiển nhận ước lượng của EKF2, giống khi bay máy thật.
Dùng để xem ảnh hưởng của bộ ước lượng.


### A — P01, 0,5×, mô hình đúng

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5
```

### B — P02, 0,5×, khai sai 1,8 kg

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p mass_kg:=1.8
```

### B0 — P01, 0,5×, khai sai 1,8 kg

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p mass_kg:=1.8
```

### C — P01, 1×, mô hình đúng

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=1.0
```

### D — P02, 1×, khai sai 1,8 kg

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=1.0 \
  -p mass_kg:=1.8
```

### E — P01, 0,5×, motor yếu 10%

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p motor_k_scale:=1.1
```

### F — P02, 0,5×, motor yếu 10%

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=case1 -p case1_speed:=0.5 \
  -p motor_k_scale:=1.1
```

---

## 4. Quỹ đạo số 8 và treo tại chỗ

Các lệnh dưới dùng trạng thái thật.
Muốn bay bằng EKF2: bỏ dòng `-p state_source:=gazebo`.

### Số 8

Mặc định: dài 3 m, rộng 1,5 m, mỗi vòng 10 s, 2 vòng. Bay ~50 s.
Bắt đầu và kết thúc ngay trên điểm cất cánh.
S1–S6 tương ứng Case I 0,5×; S7, S8 (chu kỳ 5 s, gia tốc ~5 m/s²) tương ứng Case I 1×.
S7, S8 chưa bay thử, có thể bão hòa motor.

#### S1 — P01, mô hình đúng

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=figure8 \
  -p state_source:=gazebo
```

#### S2 — P02, mô hình đúng

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=figure8 \
  -p state_source:=gazebo
```

#### S3 — P01, khai sai 1,8 kg

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=figure8 \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

#### S4 — P02, khai sai 1,8 kg

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=figure8 \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

#### S5 — P01, motor yếu 10%

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=figure8 \
  -p motor_k_scale:=1.1 \
  -p state_source:=gazebo
```

#### S6 — P02, motor yếu 10%

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=figure8 \
  -p motor_k_scale:=1.1 \
  -p state_source:=gazebo
```

#### S7 — P01, gắt (chu kỳ 5 s), mô hình đúng

```bash
ros2 run p01 p01_simple --ros-args \
  -p reference_mode:=figure8 \
  -p fig8_period_s:=5 \
  -p state_source:=gazebo
```

#### S8 — P02, gắt (chu kỳ 5 s), khai sai 1,8 kg

```bash
ros2 run p01 p02_simple --ros-args \
  -p reference_mode:=figure8 \
  -p fig8_period_s:=5 \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

Đổi hình số 8: thêm các dòng như
`-p fig8_a:=2.0 -p fig8_b:=1.0` (to hơn), `-p fig8_laps:=3` (3 vòng).

### Treo tại chỗ 

Bay ~35 s.

H1 — P01, khai sai 1,8 kg (treo thấp hơn đích ~36 cm):

```bash
ros2 run p01 p01_simple --ros-args \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

H2 — P02, khai sai 1,8 kg (lệch về 0 sau ~15 s):

```bash
ros2 run p01 p02_simple --ros-args \
  -p mass_kg:=1.8 \
  -p state_source:=gazebo
```

---

## 5. Bảng tham số

### Chọn bay gì

| Tham số | Giá trị | Mặc định |
|---|---|---|
| `reference_mode` | `hover`, `case1`, `figure8` | `hover` |
| `state_source` | `gazebo` (thật), `px4` (EKF2) | `px4` |

### Case I và treo

| Tham số | Ý nghĩa | Mặc định |
|---|---|---|
| `case1_speed` | Tốc độ so với bài: 0.25, 0.5, 1.0 | 0.25 |
| `return_home` | Bay về điểm cất cánh trước khi hạ | true |
| `hover_time_s` | Thời gian treo (chế độ `hover`) | 20 |

### Số 8

| Tham số | Ý nghĩa | Mặc định |
|---|---|---|
| `fig8_a` | Nửa chiều dài theo hướng mũi (m) | 1.5 |
| `fig8_b` | Biên độ ngang (m) | 0.75 |
| `fig8_period_s` | Thời gian một vòng (s) | 10 |
| `fig8_laps` | Số vòng | 2 |

### Tạo sai mô hình

| Tham số | Ý nghĩa | Mặc định |
|---|---|---|
| `mass_kg` | Khối lượng bộ điều khiển tin (thật 2,064 kg) | 2.064 |
| `motor_k_scale` | 1.1 = tin motor khỏe hơn thật 10% | 1.0 |

### Hằng số P02 (chỉ `p02_simple`)

| Tham số | Ý nghĩa | Mặc định |
|---|---|---|
| `ki` | Độ lợi tích phân vị trí | 1.0 × `mass_kg` |
| `c1` | Tốc độ tích lũy sai số vị trí | 1.0 |
| `sigma` | Ngưỡng bão hòa tích phân | 3.0 |
| `kI_xy` | Độ lợi tích phân tư thế (roll, pitch) | 0.1 |
| `kI_z` | Độ lợi tích phân tư thế (yaw) | 0.03 |
| `c2` | Tốc độ tích lũy sai số tư thế | 0.8 |
| `use_integral` | `false` = tắt tích phân | true |

---

## 6. Đọc dòng log khi bay

```
t=21.3 stage=3 ex=0.031 f=20.25 ei_z=1.402 saturated=0
```

| Mục | Ý nghĩa |
|---|---|
| `t` | Giây kể từ lúc bắt đầu cất cánh |
| `stage` | 0 chờ · 7 cất cánh · 3 bay · 5 về gốc · 4 hạ cánh |
| `ex` | Sai số vị trí (m) |
| `f` | Lực tổng (N); treo ≈ 20 N |
| `ei_z` | (P02) tích phân trục z; khai 1,8 kg → ~1,4 |
| `saturated` | 1 = có motor chạm 0 hoặc 1 |
