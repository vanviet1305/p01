#!/usr/bin/env bash
# CÀI MÔ HÌNH X650 VÀO CÂY PX4 (bản gốc lưu trong repo p01 để không mất khi cài lại PX4).
# Chép: Tools/simulation/gz/models/x650_base, x650; airframe 4022_gz_x650; thêm dòng 4022_gz_x650 vào airframes/CMakeLists.txt.
# Mesh dùng lại của x500_base (model://x500_base/...) nên cây PX4 phải có sẵn x500_base.
# Cách dùng: bash ~/ros2_px4_ws/src/p01/px4_x650/cai_dat_px4_x650.sh   (PX4 khác chỗ: PX4_DIR=/duong/dan bash ...)
# Sau đó: cd ~/PX4-Autopilot && make px4_sitl gz_x650
set -eu
NGUON="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR=${PX4_DIR:-~/PX4-Autopilot}
MODELS="$PX4_DIR/Tools/simulation/gz/models"
AF="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
[ -d "$MODELS/x500_base" ] || { echo "LỖI: không thấy $MODELS/x500_base (sai PX4_DIR?)"; exit 1; }
[ -f "$AF/CMakeLists.txt" ] || { echo "LỖI: không thấy $AF/CMakeLists.txt"; exit 1; }

# Chép (ghi đè bản cũ nếu có) và báo tệp nào khác bản trong repo trước khi ghi đè
for m in x650_base x650; do
  if [ -d "$MODELS/$m" ] && ! diff -rq "$NGUON/models/$m" "$MODELS/$m" >/dev/null; then
    echo "Chú ý: $MODELS/$m khác bản trong repo, sẽ ghi đè bằng bản repo."
  fi
  mkdir -p "$MODELS/$m"; cp "$NGUON/models/$m/"* "$MODELS/$m/"
done
cp "$NGUON/airframes/4022_gz_x650" "$AF/4022_gz_x650"

# Thêm airframe vào danh sách build (chỉ một lần)
if ! grep -q "4022_gz_x650" "$AF/CMakeLists.txt"; then
  sed -i 's/^\(\s*\)4001_gz_x500$/&\n\14022_gz_x650/' "$AF/CMakeLists.txt"
  grep -q "4022_gz_x650" "$AF/CMakeLists.txt" || { echo "LỖI: không thêm được 4022_gz_x650 vào CMakeLists (không thấy dòng 4001_gz_x500)"; exit 1; }
fi
echo "Đã cài X650 vào $PX4_DIR. Build: cd $PX4_DIR && make px4_sitl gz_x650"
