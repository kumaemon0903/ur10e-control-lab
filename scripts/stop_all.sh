#!/bin/bash
# ラボ関連のプロセス（Gazebo, 制御ノード, ブリッジ, rqt_plot など）をすべて終了する.
#   使い方: ./scripts/stop_all.sh
# 通常は launch を起動した端末で Ctrl+C すれば終了するが、
# Gazebo が固まった・端末を閉じてしまった・前回のプロセスが残っている、といった場合に使う。

PATTERNS=(
  "control_lab.sdf"            # Gazebo (gz sim)
  "ur10e_control_lab/lab_node" # 制御ノード
  "ros2 launch ur10e_control_lab"
  "parameter_bridge"
  "robot_state_publisher"
  "controller_manager/spawner"
  "rqt_plot"
  "ur10e_control_lab/push"
)

list_pids() {
  # 自分自身と呼び出し元のシェルは除外する
  for p in "${PATTERNS[@]}"; do pgrep -f -- "$p"; done | sort -u | grep -vxE "$$|$PPID"
}

PIDS=$(list_pids)
if [ -z "$PIDS" ]; then
  echo "動作中のラボのプロセスはありません。"
  exit 0
fi
echo "終了します:"
ps -o pid=,args= -p $(echo $PIDS) | cut -c1-120
kill -INT $PIDS 2>/dev/null
for i in 1 2 3 4 5; do
  sleep 1
  [ -z "$(list_pids)" ] && { echo "すべて終了しました。"; exit 0; }
done
echo "終了しないプロセスを強制終了します。"
kill -9 $(list_pids) 2>/dev/null
sleep 1
[ -z "$(list_pids)" ] && echo "すべて終了しました。" || { echo "まだ残っています:"; ps -o pid=,args= -p $(list_pids | tr '\n' ' '); }
