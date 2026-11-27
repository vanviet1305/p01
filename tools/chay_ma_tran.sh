#!/usr/bin/env bash
# CHẠY TỰ ĐỘNG MA TRẬN THÍ NGHIỆM trên x650: 4 bộ điều khiển (BO) × các kịch bản × SO_LAN lần (trạng thái thật).
# Mặc định Case I 0,5× (TOC_DO=1.0 để chạy bộ 1×).
# Cách dùng (tab node bay, đã source ~/ros2_px4_ws/install/setup.bash; Agent + PX4 gz_x650 đã chạy, thấy "Ready for takeoff!"):
#   bash ~/ros2_px4_ws/src/p01/tools/chay_ma_tran.sh            # mặc định 3 lần
#   SO_LAN=1 bash ~/ros2_px4_ws/src/p01/tools/chay_ma_tran.sh   # chạy thử 1 lần
#   KICH_BAN="c1x05_tai05 c1x05_gio" bash …/chay_ma_tran.sh     # chỉ chạy một số kịch bản
# Thứ tự xen kẽ theo vòng (lần 1 của mọi ca, rồi lần 2, ...) để trôi theo thời gian của PX4 chia đều cho các bộ.
# Ca nào đã có CSV đủ dòng thì bỏ qua → dừng giữa chừng rồi chạy lại sẽ tiếp tục từ ca còn thiếu.
# Một chuyến quá 120 s (node treo) → script DỪNG: đưa PX4 về chờ (commander mode auto:loiter, commander disarm) rồi chạy lại.
# Xong thì chạy: python3 ~/ros2_px4_ws/src/p01/tools/tong_hop_ma_tran.py
set -u
SO_LAN=${SO_LAN:-3}
# TOC_DO: tốc độ Case I. Mặc định 0,5× theo quy ước so sánh (GHI_CHU 21/11: ở 1× giới hạn vật lý lấn át khác biệt
# giữa các luật điều khiển). TOC_DO=1.0 → các ca c1_* (bộ đã chạy 25/11); 0.5 → c1x05_*.
TOC_DO=${TOC_DO:-0.5}
if [ "$TOC_DO" = "1.0" ]; then TEN_C1=c1; else TEN_C1="c1x$(echo "$TOC_DO" | tr -d '.')"; fi
OUT=~/"báo/06_ket_qua (log, đồ thị Gazebo)/gazebo_logs/ma_tran_x650"
mkdir -p "$OUT"
cd "$OUT" || exit 1

# Gắn odometry thật (bỏ qua nếu đã gắn)
bash ~/ros2_px4_ws/src/p01/tools/gan_odom_gazebo.sh x650 || { echo "LỖI: chưa có odometry Gazebo"; exit 1; }

# Kịch bản: tên → tham số
declare -A KB
KB[${TEN_C1}_dung]="-p reference_mode:=case1 -p case1_speed:=$TOC_DO"
KB[${TEN_C1}_m26]="-p reference_mode:=case1 -p case1_speed:=$TOC_DO -p mass_kg:=2.6"
KB[${TEN_C1}_k11]="-p reference_mode:=case1 -p case1_speed:=$TOC_DO -p motor_k_scale:=1.1"
KB[tron_yaw]="-p reference_mode:=circle_yaw"
# Nhiễu ngoài đổi giữa chuyến (Gazebo ApplyLinkWrench; quỹ đạo Case I chạy từ t = 10 s): bật tại t = 15 s, giữ tới lúc hạ.
KB[${TEN_C1}_tai05]="-p reference_mode:=case1 -p case1_speed:=$TOC_DO -p payload_kg:=0.5 -p dist_t_on:=15.0"
KB[${TEN_C1}_gio]="-p reference_mode:=case1 -p case1_speed:=$TOC_DO -p dist_fx:=2.0 -p dist_t_on:=15.0"
# Mômen nhiễu hằng 0,2 N.m quanh trục x (Bắc) của thế giới từ t = 15 s (≈ khối tâm lệch 6,8 mm: 0,2 / (3,0·9,81)).
# Chọn < B_theta_R 0,3 để phép chiếu chống windup không cắt ước lượng θ̄_R. Hướng mũi Case I cố định nên tương đương mômen hệ thân.
KB[${TEN_C1}_mom]="-p reference_mode:=case1 -p case1_speed:=$TOC_DO -p dist_tx:=0.2 -p dist_t_on:=15.0"
# Ca *_ekf: giống ca gốc nhưng bộ điều khiển dùng EKF2 (state_source:=px4); sai số thật ghi ở cột gt_pos_err.
for k in "${!KB[@]}"; do KB[${k}_ekf]="${KB[$k]}"; done
nguon() { case "$1" in *_ekf) echo px4;; *) echo gazebo;; esac; }
# tron_yaw không phụ thuộc TOC_DO: đã có đủ 3 lần thì được bỏ qua.
THU_TU_KB=${KICH_BAN:-"${TEN_C1}_dung ${TEN_C1}_m26 ${TEN_C1}_k11 tron_yaw ${TEN_C1}_tai05 ${TEN_C1}_gio"}
# BO: chọn bộ điều khiển, ví dụ BO="p06c" bash …/chay_ma_tran.sh
#   p06c = P06 với c2 = 1,96 (= c3 của trục yaw). c2 mặc định 5,06 tính theo trục roll (kR_x, J1) nhưng P06 dùng chung
#   cho cả yaw → vòng thích nghi yaw tuyến tính có cặp cực 0,43 Hz, ζ = 0,07 (khớp dao động Mz 0,43 Hz đo trong tron_yaw).
THU_TU_BDK=${BO:-"p01 p02 p06 p11"}
#   p11gx05 / p11gx8: P11 với gamma_x 0,5 / 8 (mặc định 2); p11gR02 / p11gR2: P11 với gamma_R 0,2 / 2 (mặc định 0,5).
#   p02ki8: P02 với ki = 8 (mặc định 1·m = 3), cùng hệ số tích phân lực với P11 gamma_x = 8 (so sánh công bằng).
ten_ctrl() { case "$1" in p06c) echo p06;; p11g*) echo p11;; p02ki*) echo p02;; *) echo "$1";; esac; }
# P06/P11: chặn ‖θ̄_R‖ ≤ 0,3 N.m (chống windup, xem NHAT_KY 25/11)
them_bdk() { case "$1" in p06|p11) echo "-p B_theta_R:=0.3";; p06c) echo "-p B_theta_R:=0.3 -p c2:=1.96";;
  p11gx05) echo "-p B_theta_R:=0.3 -p gamma_x:=0.5";;  p11gx8) echo "-p B_theta_R:=0.3 -p gamma_x:=8.0";;
  p02ki8) echo "-p ki:=8.0";;
  p11gR02) echo "-p B_theta_R:=0.3 -p gamma_R:=0.2";;  p11gR2) echo "-p B_theta_R:=0.3 -p gamma_R:=2.0";; *) echo "";; esac; }

tong=$(( SO_LAN * $(echo $THU_TU_BDK | wc -w) * $(echo $THU_TU_KB | wc -w) )); dem=0
for lan in $(seq 1 "$SO_LAN"); do
  for kb in $THU_TU_KB; do
    for bdk in $THU_TU_BDK; do
      dem=$((dem+1))
      csv="x650_${bdk}_${kb}_g${lan}.csv"
      if [ -f "$csv" ] && [ "$(wc -l < "$csv")" -gt 2500 ]; then
        echo "[$dem/$tong] Đã có $csv, bỏ qua"; continue
      fi
      echo "[$dem/$tong] $(date +%H:%M:%S) Bay $csv"
      # shellcheck disable=SC2046
      timeout --signal=INT 120 ros2 run p01 uav_simple --ros-args -p controller:=$(ten_ctrl "$bdk") -p vehicle:=x650 \
        -p state_source:=$(nguon "$kb") ${KB[$kb]} $(them_bdk "$bdk") -p log_csv:="$csv" > "${csv%.csv}.log" 2>&1
      ma=$?
      # Kiểm tra đã cất cánh thật: trong đoạn bay (stage 3) sai số vị trí phải < 1 m.
      # (25/11/2026: PX4 từ chối arm nhưng node vẫn chạy hết và ghi CSV sai, UAV nằm dưới đất, ex = 2 m.)
      if ! python3 -c "import csv,sys; r=list(csv.DictReader(open(sys.argv[1]))); e=[float(x['position_error']) for x in r if float(x['stage'])==3]; sys.exit(0 if e and max(e)<1.0 else 1)" "$csv" 2>/dev/null; then
        echo "DỪNG: $csv — UAV không cất cánh / mất bám (sai số > 1 m trong lúc bay). Xem tab PX4 (Arming denied?)."
        mv "$csv" "${csv%.csv}_LOI.csv"; mv "${csv%.csv}.log" "${csv%.csv}_LOI.log"
        exit 1
      fi
      if [ $ma -ne 0 ] || ! grep -q "Đã hạ cánh và disarm" "${csv%.csv}.log"; then
        echo "DỪNG: chuyến $csv không kết thúc bình thường (mã $ma). Xem ${csv%.csv}.log"
        echo "Đưa PX4 về chờ: commander mode auto:loiter ; commander disarm  — rồi chạy lại script (ca đã xong được bỏ qua)."
        rm -f "$csv"
        exit 1
      fi
      sleep 5   # chờ PX4 ổn định sau disarm
    done
  done
done
echo "XONG $tong chuyến. Tổng hợp: python3 ~/ros2_px4_ws/src/p01/tools/tong_hop_ma_tran.py"
