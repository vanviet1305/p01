# Kết quả mẫu để đối chiếu

Log gốc của các lần chạy thành công trên máy tác giả, ngày 19–20/11/2026, cùng phiên bản đã khóa ở [CAI_DAT.md](../CAI_DAT.md). Dùng để so sánh khi bạn chạy lại. Không ghi đè các tệp trong thư mục này.

| Tệp | Lần chạy | Kết quả |
|---|---|---|
| `p01_ground_dryrun_01.csv` / `_terminal.log` | Dry-run, không phát lệnh | Đủ dữ liệu, `enabled=0` |
| `p01_ground_hover_01.csv` / `_terminal.log` | Hover 2 m | RMS 1,76 cm, max 3,47 cm, không bão hòa |
| `p01_ground_case1_01.csv` / `_terminal.log` | Case I 0,25× | Di chuyển RMS 3,80 cm, max 6,82 cm; giữ cuối RMS 1,99 cm; không bão hòa |
| [do_thi_case1_01/](./do_thi_case1_01/README.md) | Đồ thị và `summary.json` của Case I | Tạo bằng `tools/plot_flight_log.py` |

Lệnh tạo lại đồ thị, chạy trong thư mục này:

```bash
python3 -I ../tools/plot_flight_log.py --csv p01_ground_case1_01.csv --output do_thi_case1_01 --motion-start 10 --motion-end 40
```

Lệnh chạy đúng như README mục 9, Bước 7–8 (tên log `_01`). Ý nghĩa các cột CSV: README mục 7.
