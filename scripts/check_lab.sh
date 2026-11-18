#!/bin/bash
# ラボが正しく動いているかを確認する.
#   使い方: ./scripts/check_lab.sh
#   前提:   別の端末で ros2 launch ur10e_control_lab lab.launch.py が起動していること

ok()   { echo "  [OK] $*"; }
ng()   { echo "  [NG] $*"; FAIL=1; }
FAIL=0

if ! command -v ros2 >/dev/null; then
  echo "ros2 コマンドが見つかりません。先に次を実行してください:"
  echo "  source ~/sandbox_for_claude/ur10e_control_lab_ws/install/setup.bash"
  exit 1
fi
if ! ros2 pkg prefix ur10e_control_lab >/dev/null 2>&1; then
  echo "ur10e_control_lab パッケージが見つかりません。先に次を実行してください:"
  echo "  source ~/sandbox_for_claude/ur10e_control_lab_ws/install/setup.bash"
  exit 1
fi

echo "1. Gazebo"
if pgrep -f "control_lab.sdf" >/dev/null; then ok "Gazebo が動いています"; else ng "Gazebo が起動していません（ros2 launch ur10e_control_lab lab.launch.py）"; fi

echo "2. ros2_control のコントローラ"
CTRL=$(timeout 10 ros2 control list_controllers 2>/dev/null)
for c in joint_state_broadcaster effort_controller; do
  if echo "$CTRL" | grep -q "^$c .*active"; then ok "$c: active"; else ng "$c が active ではありません"; fi
done

echo "3. 制御ノード /control_lab"
if timeout 10 ros2 node list 2>/dev/null | grep -q "^/control_lab$"; then
  ok "動いています"
  for p in controller reference estimator model_payload_mass position_noise_std record; do
    v=$(timeout 10 ros2 param get /control_lab $p 2>/dev/null | sed 's/^.*value is: //')
    printf "       %-20s = %s\n" "$p" "$v"
  done
else
  ng "/control_lab が見つかりません"
fi

echo "4. 追従誤差（/control_lab/error の position, 単位 rad）"
ERR=$(timeout 5 ros2 topic echo --once /control_lab/error --field position 2>/dev/null | head -1)
if [ -n "$ERR" ]; then ok "$ERR"; else ng "/control_lab/error が届きません（シミュレーションが一時停止中かもしれません）"; fi

echo
if [ $FAIL -eq 0 ]; then echo "すべて正常です。"; else echo "問題があります。docs/START_HERE.md の「トラブルシューティング」を参照してください。"; fi
exit $FAIL
