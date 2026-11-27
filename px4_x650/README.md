# Mô hình X650 cho PX4 SITL + Gazebo

Bản gốc của mô hình Gazebo `x650` dùng trong đồ án (thông số ước lượng theo X650 + MN4014 KV330 + 1555 + 6S).
Các tệp này chạy trong cây PX4, không chạy trong p01; lưu ở đây để có trong Git.

| Thông số | Giá trị |
|---|---|
| Khối lượng | 3,0 kg (thân 2,9357 + 4 rotor) |
| Quán tính J | diag(0,060; 0,060; 0,112) kg·m² |
| Cánh tay | 0,23 m |
| Hệ số lực đẩy / mômen | 6,4e-5 / 0,016 |
| Tốc độ rotor | 100–630 rad/s, trễ lên/xuống 30/50 ms |
| T/W | ≈ 3,4 |

Cài: `bash cai_dat_px4_x650.sh`, rồi `make px4_sitl gz_x650`.
Sửa mô hình: sửa bản trong thư mục này, chạy lại script cài, commit.
Nguồn gốc thông số: `06_ket_qua (log, đồ thị Gazebo)/Tham_so_X500.md` và NHAT_KY 25/11/2026.
