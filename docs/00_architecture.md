# 0. 環境の仕組み — ロボットを「トルク」で動かす

> **準備**: [起動・操作マニュアル](START_HERE.md) の 3〜4 章の手順で、**端末 1** でシミュレーションを起動し（`ros2 launch ur10e_control_lab lab.launch.py`）、
> **端末 2** で `cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash` を済ませておきます。
> 特に断りがなければ、この章のコマンドは端末 2 で実行します。`offline_sim` と `plot_log` は Gazebo を起動していなくても実行できます。
> 終わったら端末 1 で `Ctrl+C`。動きがおかしくなったら `ros2 param set /control_lab reference hold` → `ros2 param set /control_lab controller pd_gravity` で初期姿勢の保持に戻せます。

各制御理論に入る前に、この環境で何が起きているかを把握しましょう。

## 0.1 全体構成

```
┌───────────── Gazebo (物理シミュレーション 1 kHz) ─────────────┐
│  UR10e + 手先負荷 5 kg                                         │
│     ▲ 関節トルク τ                    │ 関節角 q, 角速度 q̇    │
│  gz_ros2_control  (ros2_control のハードウェア役)              │
└─────▲──────────────────────────────────┼──────────────────────┘
      │ /effort_controller/commands      │ /joint_states (500 Hz)
      │ (Float64MultiArray, 6 個のトルク) ▼
┌─────┴──────────────── lab_node (Python) ──────────────────────┐
│  センサモデル → 推定器 → 制御器 → トルク飽和                  │
│  (ノイズ付加)  (none/diff/kf)  (PID, CT, LQR, ...)             │
│                               ▲                                │
│                         目標軌道 (hold/step/sine/...)          │
└───────────────────────────────────────────────────────────────┘
```

- **ros2_control の `effort_controllers/JointGroupEffortController`** は、受け取ったトルクをそのまま関節に入れるだけです。
  制御理論の中身はすべて Python 側（`lab_node`）にあります。
- 通常の産業用ロボットの使い方（位置指令）と違い、**トルク指令**なので、重力も慣性も制御器自身が面倒を見る必要があります。
  だからこそ制御理論の違いがはっきり出ます。
- 制御周期は 2 ms（500 Hz）。`/joint_states` が届くたびに 1 回計算します。

関連ファイル:
- [lab.launch.py](../src/ur10e_control_lab/launch/lab.launch.py) — 起動手順（一時停止で起動 → 制御ノード起動 → 再開）
- [ros2_controllers.yaml](../src/ur10e_control_lab/config/ros2_controllers.yaml) — 500 Hz、effort コントローラ
- [ur10e_lab.urdf.xacro](../src/ur10e_control_lab/urdf/ur10e_lab.urdf.xacro) — effort インターフェースと手先負荷
- [lab_node.py](../src/ur10e_control_lab/ur10e_control_lab/lab_node.py) — ROS ノード
- [control_loop.py](../src/ur10e_control_lab/ur10e_control_lab/control_loop.py) — 1 周期の処理（`ControlLoop.step`）

## 0.2 ロボットの運動方程式

n 関節アームの運動は次の式で表されます（ラグランジュ法で導出）:

```
M(q) q̈ + C(q, q̇) q̇ + g(q) = τ
```

| 項 | 意味 | 性質 |
|---|---|---|
| `M(q)` | 慣性行列 (6×6) | 対称・正定値。姿勢 q によって変わる |
| `C(q,q̇)q̇` | コリオリ力・遠心力 | 速度の 2 次。速く動くと大きくなる |
| `g(q)` | 重力トルク | 姿勢だけで決まる。止まっていても必要 |
| `τ` | 関節トルク | 制御入力 |

もう 1 つ重要な性質が **Ṁ − 2C が歪対称**であることです（エネルギー保存に由来）。
PD＋重力補償、スライディングモード、適応制御の安定性証明で使います。

### コードではどう計算しているか

[robot_model.py](../src/ur10e_control_lab/ur10e_control_lab/robot_model.py) の `RobotModel` クラス:

```python
model = RobotModel(payload_mass=5.0)   # URDF から作る。負荷の有無を選べる
M = model.mass_matrix(q)               # M(q)
c = model.coriolis(q, qd)              # C(q,q̇)q̇
g = model.gravity(q)                   # g(q)
p, R = model.fk(q)                     # 手先の位置・姿勢
J = model.jacobian(q)                  # ヤコビアン
```

内部では URDF を KDL（Orocos Kinematics and Dynamics Library）のチェーンに変換し、
`ChainDynParam`（動力学）、`ChainJntToJacSolver`（ヤコビアン）などを呼んでいます。

**やってみよう（Python で直接触ってみる）:**

端末 2（または新しい端末）で Python を起動します（Gazebo は起動していなくても構いません）:

```bash
cd ~/sandbox_for_claude/ur10e_control_lab_ws
source install/setup.bash
python3
```

`>>>` が表示されたら、次を 1 行ずつ入力します（終わったら `exit()` で抜けます）:
```python
import numpy as np
from ur10e_control_lab.robot_model import RobotModel
m0 = RobotModel(payload_mass=0.0)
m5 = RobotModel(payload_mass=5.0)
q = np.array([0, -1.5708, 1.5708, -1.5708, -1.5708, 0])   # 初期姿勢
print(m0.gravity(q))     # 負荷なしの重力トルク
print(m5.gravity(q))     # 負荷 5 kg → 肩・肘のトルクが約 34 Nm 増える
print(np.diag(m0.mass_matrix(q)))
print(np.diag(m5.mass_matrix(q)))   # 手首の慣性は負荷で 10〜25 倍に！
```

出力例:
```
[ -0.   -42.14 -42.14  -2.85  -0.     0.  ]
[ -0.   -76.06 -76.06  -8.72  -0.     0.  ]
[3.189 7.335 2.650 0.0402 0.00777 0.00034]
[5.739 10.52 5.283 0.3548 0.25057 0.00867]
```

手首（特に wrist_3）は慣性が非常に小さく、負荷の影響を強く受けます。
このことが後の章で「モデル誤差」の効き方の違いとして現れます。

## 0.3 1 制御周期の流れ

[control_loop.py](../src/ur10e_control_lab/ur10e_control_lab/control_loop.py) の `ControlLoop.step()`:

```python
q_meas = q_true + ノイズ                       # 1. センサモデル
q_hat, qd_hat = estimator.update(...)          # 2. 状態推定（none/diff/kf）
ref = trajectory(t)                            # 3. 目標値 (q_ref, q̇_ref, q̈_ref, x_ref, ...)
tau, info = controller.compute(t, q_hat, qd_hat, ref)   # 4. 制御則
tau = clip(tau, -limit, limit)                 # 5. トルク飽和
```

すべての制御器は同じインターフェース [controllers/base.py](../src/ur10e_control_lab/ur10e_control_lab/controllers/base.py) を持つので、
実行中に自由に切り替えられます。

## 0.4 目標軌道

[trajectories.py](../src/ur10e_control_lab/ur10e_control_lab/trajectories.py)

| 名前 | 内容 | 何を見るのに向いているか |
|---|---|---|
| `hold` | 初期姿勢で静止 | 重力に対する定常偏差、外力への応答 |
| `step` | 4 秒ごとに +0.3 rad の段差 | 過渡応答（立ち上がり・オーバーシュート）、トルク飽和 |
| `quintic` | 5 次多項式の滑らかな点間移動 | 実用的な動作での追従性 |
| `sine` | 各関節を異なる周波数で正弦波 | 連続的な追従、関節間の干渉 |
| `circle` | 手先で円を描く（IK で関節目標に変換） | タスク空間の追従 |

## 0.5 Gazebo とオフラインシミュレータ

| | Gazebo (`lab.launch.py`) | オフライン (`offline_sim`) |
|---|---|---|
| 物理 | DART 物理エンジン（接触・関節の数値特性あり） | 運動方程式を Euler 法で積分するだけ |
| 速さ | 実時間 | 実時間の約 10 倍 |
| 見た目 | 3D 表示 | グラフのみ |
| 用途 | 最終確認、外力を加える実験 | ゲイン調整、多数の比較 |

どちらも**同じ制御コード**（`ControlLoop`）を使うので、オフラインで調整したゲインがそのまま Gazebo で使えます。
この環境で測った結果は、両者でほぼ一致しています（例: 計算トルク法の正弦波追従で手先誤差 RMS 1.7 mm と 1 mm 前後）。

## 0.6 確認問題

1. `hold` で静止しているとき、運動方程式のどの項が 0 になり、どの項が残りますか？
2. 手先負荷 5 kg が肩（shoulder_lift）に与える重力トルクは約 34 Nm でした。腕の水平距離がおよそ何 m なら説明がつくでしょうか？（34 / (5 × 9.81) ≈ ?）
3. 位置指令型のロボット（一般的な産業用ロボット）では、なぜ重力をあまり気にしなくてよいのでしょうか？

→ 次は [1. PID 制御](01_pid.md)
