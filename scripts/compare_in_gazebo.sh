#!/bin/bash
# Gazebo で複数の制御器を順番に試し、同じ軌道での誤差を比較する.
#   前提: 別の端末で `ros2 launch ur10e_control_lab lab.launch.py` が起動していること
#
#   使い方: ./scripts/compare_in_gazebo.sh <軌道> <秒数> <制御器1> <制御器2> ...
#   例:     ./scripts/compare_in_gazebo.sh sine 8 pid pd_gravity computed_torque
#
# 各制御器について: hold で初期姿勢に戻す → 制御器を切り替え → 記録しながら軌道を実行
# 最後に全員分を重ねたグラフを表示する。
set -e
if [ $# -lt 3 ]; then
  sed -n '2,10p' "$0"; exit 1
fi
REF=$1; DUR=$2; shift 2
FILES=()
for c in "$@"; do
  echo "=== $c ==="
  ros2 param set /control_lab reference hold >/dev/null
  ros2 param set /control_lab controller pd_gravity >/dev/null; sleep 2   # 初期姿勢へ戻す
  ros2 param set /control_lab controller "$c" >/dev/null; sleep 1
  ros2 param set /control_lab record true >/dev/null
  ros2 param set /control_lab reference "$REF" >/dev/null
  sleep "$DUR"
  ros2 param set /control_lab record false >/dev/null
  FILES+=("$(ls -t ~/ur10e_lab_logs/*.csv | head -1)")
done
ros2 param set /control_lab reference hold >/dev/null
ros2 param set /control_lab controller pd_gravity >/dev/null
echo "記録したファイル:"; printf '  %s\n' "${FILES[@]}"
ros2 run ur10e_control_lab plot_log "${FILES[@]}" --out ~/ur10e_lab_logs/compare_${REF}.png
