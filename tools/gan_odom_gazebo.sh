#!/usr/bin/env bash
# Gắn plugin OdometryPublisher vào x500_0 trong Gazebo đang chạy (không sửa tệp PX4).
# Phát /model/x500_0/odometry ~250 Hz: vị trí + tư thế THẬT, dùng cho p01_simple/p02_simple state_source:=gazebo.
# Chạy SAU khi PX4 + Gazebo đã lên (thấy "Ready for takeoff!"). Mỗi lần khởi động lại PX4 phải chạy lại.
set -e
TOPIC=/model/x500_0/odometry
# Kiểm tra có BÊN PHÁT (không dùng gz topic -l: tên topic vẫn hiện khi chỉ có bên nhận, ví dụ node đang chờ).
co_ben_phat() { gz topic -i -t "$TOPIC" 2>/dev/null | grep -q "^Publishers"; }
if co_ben_phat; then
  echo "Plugin đang phát $TOPIC, không gắn thêm."
  exit 0
fi
# Lấy entity ID của model (gắn theo tên không được: plugin báo "should be attached to a model entity").
ID=$(gz model -m x500_0 2>/dev/null | grep -oP 'Model: \[\K[0-9]+' || true)
if [ -z "$ID" ]; then
  echo "Không tìm thấy model x500_0. Gazebo đã chạy chưa?"
  exit 1
fi
echo "x500_0 có entity ID = $ID, đang gắn OdometryPublisher..."
gz service -s /world/default/entity/system/add --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 3000 \
  --req "entity: {id: $ID}, plugins: {name: \"gz::sim::systems::OdometryPublisher\", filename: \"gz-sim-odometry-publisher-system\", innerxml: \"<odom_frame>odom</odom_frame><robot_base_frame>x500_0</robot_base_frame><odom_publish_frequency>250</odom_publish_frequency><dimensions>3</dimensions>\"}"
sleep 1
if co_ben_phat; then echo "OK: plugin đang phát $TOPIC"; else echo "LỖI: chưa có bên phát $TOPIC"; exit 1; fi
