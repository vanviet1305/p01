# Đồ thị P01 / Gazebo X500 — Case I điều chỉnh, lần 01

Đọc các hình từ dữ liệu phản hồi PX4, không phải trạng thái thật tuyệt đối của Gazebo. Chỉ lấy stage=3, không lọc hoặc làm mượt dữ liệu.

- [PDF ba trang](./P01_Gazebo_CaseI_01_Do_thi.pdf)
- [Hình 1: quỹ đạo 3D và hình chiếu ngang](./01_quy_dao_3d.png)
- [Hình 2: vị trí phản hồi/tham chiếu theo thời gian](./02_vi_tri_theo_thoi_gian.png)
- [Hình 3: sai số theo từng trục NED và chuẩn sai số](./03_sai_so_bam.png)
- [Số liệu và thông tin nguồn](./summary.json)

Nguồn: [p01_ground_case1_01.csv](../p01_ground_case1_01.csv). SHA-256: `5997857d1efc988e67a510e88a2f6070c7aa57aa27dcba9c55c83608053d5a4e`.

Dải nền là đoạn di chuyển 10–40 s theo cấu hình lần chạy. Sau 40 s, ba tọa độ tham chiếu cố định trong dữ liệu. Cao độ hiển thị h=-z_NED; sai số trục thứ ba vẫn giữ dấu NED (dương xuống).

| Đoạn | Mẫu | Trung bình (cm) | RMS (cm) | Lớn nhất (cm) |
|---|---:|---:|---:|---:|
| Giữ điểm đầu | 300 | 1.674 | 1.701 | 2.143 |
| Di chuyển | 3000 | 3.439 | 3.797 | 6.817 |
| Giữ điểm cuối | 8048 | 1.801 | 1.990 | 4.003 |

RMS là căn trung bình bình phương chuẩn sai số theo các mẫu ghi. Số mẫu bão hòa theo bộ phân bổ: 0. Các cột lực/mômen achieved là giá trị tính lại từ phân bổ, không phải đo lực rotor.

Bản dữ liệu được vẽ có stage cuối là 4; hình không xác nhận nguyên nhân/kết quả thoát Offboard. Đây là Case I điều chỉnh cho X500, chưa phải tái hiện nguyên bản CDC.

## Tạo lại trên Ubuntu

Chạy từ thư mục gốc báo, với NumPy và Matplotlib đã có trong môi trường Python:

```bash
/usr/bin/python3 p01/tools/plot_flight_log.py --csv p01_ground_case1_01.csv --output do_thi_case1_01 --motion-start 10 --motion-end 40
```

Môi trường đã chạy: Python 3.10.12, NumPy 1.21.5, Matplotlib 3.5.1. Bộ điều khiển ROS 2/PX4 không bị sửa hoặc khởi động bởi công cụ vẽ.
