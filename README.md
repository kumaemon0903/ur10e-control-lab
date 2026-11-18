# UR10e 制御理論ラボ（ROS 2 Jazzy + Gazebo Harmonic）

UR10e（6 軸アーム）を Gazebo 上で**トルク指令**で動かし、制御理論を「コードを読む → 動かす → 比べる」で学ぶための環境です。

> ## 👉 はじめに: [docs/START_HERE.md（起動・操作マニュアル）](docs/START_HERE.md)
> ビルド、端末の準備、起動・動作確認、操作、グラフ表示、記録、終了、トラブル対処まで、
> ターミナルからのコマンド入力だけで完結する手順をまとめています。最初に必ず読んでください。

| # | 制御理論 | 主な学習ポイント | 教材 |
|---|---|---|---|
| 0 | 環境の仕組み | ros2_control（effort）、動力学モデル M(q), C, g(q) | [docs/00_architecture.md](docs/00_architecture.md) |
| 1 | PID 制御 | 定常偏差、I 項、ワインドアップ、関節間干渉 | [docs/01_pid.md](docs/01_pid.md) |
| 2 | PD + 重力補償 | リアプノフ安定性、モデルの一部だけを使う | [docs/02_pd_gravity.md](docs/02_pd_gravity.md) |
| 3 | 計算トルク法 | フィードバック線形化、ωn・ζ によるゲイン設計 | [docs/03_computed_torque.md](docs/03_computed_torque.md) |
| 4 | LQR | Riccati 方程式、Q/R の重み、線形化 | [docs/04_lqr.md](docs/04_lqr.md) |
| 5 | スライディングモード制御 | ロバスト制御、チャタリング、境界層 | [docs/05_sliding_mode.md](docs/05_sliding_mode.md) |
| 6 | 適応制御 | パラメータのオンライン推定、持続的励振 | [docs/06_adaptive.md](docs/06_adaptive.md) |
| 7 | インピーダンス制御 | タスク空間、ヤコビアン、外力への柔らかさ | [docs/07_impedance.md](docs/07_impedance.md) |
| 8 | モデル予測制御 (MPC) | 制約付き最適化、先読み、QP ソルバ (ADMM) | [docs/08_mpc.md](docs/08_mpc.md) |
| 9 | カルマンフィルタ | 状態推定、ノイズ、モデルとセンサの融合 | [docs/09_kalman_filter.md](docs/09_kalman_filter.md) |
| 10 | 演習: 自分の制御則 | テンプレートから実装して比較する | [docs/10_your_controller.md](docs/10_your_controller.md) |

---

## 1. セットアップ

必要なもの（この PC にはすべてインストール済み）:
ROS 2 Jazzy、Gazebo Harmonic（`ros_gz`）、`gz_ros2_control`、`ur_description`、`ros2_controllers`、PyKDL、numpy / scipy / matplotlib

```bash
cd ~/sandbox_for_claude/ur10e_control_lab_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> `--symlink-install` でビルドすると、Python コードと `config/lab_params.yaml` の変更が**再ビルドなしで**反映されます
> （Python の変更は制御ノードの再起動、YAML の変更は `reload` サービスで反映）。

動作確認（約 10 秒）:

```bash
python3 -m pytest src/ur10e_control_lab/test -q     # 16 passed になれば OK
```

## 2. クイックスタート

### Gazebo で動かす

```bash
# 端末 1: Gazebo + UR10e + 制御ノード を起動（最初は「PD + 重力補償」で初期姿勢を保持）
cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash
ros2 launch ur10e_control_lab lab.launch.py
```

```bash
# 端末 2: 実行中に制御則や目標軌道を切り替える
cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash
./scripts/check_lab.sh                               # 起動確認（すべて正常 と表示されれば OK）
ros2 param set /control_lab controller computed_torque
ros2 param set /control_lab reference sine          # hold / step / quintic / sine / circle
ros2 param set /control_lab controller pid           # 計算トルク法と動きを見比べる
```

### グラフで見る

```bash
# リアルタイム表示（関節 2 = elbow の追従誤差）
ros2 run rqt_plot rqt_plot /control_lab/error/position[2]

# CSV に記録してグラフ化
ros2 param set /control_lab record true
#   ...（しばらく動かす）...
ros2 param set /control_lab record false             # 保存先とグラフ化コマンドが表示される
ros2 run ur10e_control_lab plot_log ~/ur10e_lab_logs/<ファイル名>.csv
```

### Gazebo なしで素早く比較（オフラインシミュレータ）

同じ制御コードを Python だけのシミュレータで動かします。数秒で結果が出るので、ゲイン調整に便利です。

```bash
ros2 run ur10e_control_lab offline_sim -c pid pd_gravity computed_torque -r sine
```

```
結果:
       pid: 関節誤差RMS[deg]=[0.18 0.68 0.71 2.15 1.9  0.76]  手先誤差RMS=17.3mm ...
pd_gravity: 関節誤差RMS[deg]=[0.2  0.22 0.34 0.93 0.78 0.78]  手先誤差RMS= 9.3mm ...
computed_torque: 関節誤差RMS[deg]=[0.05 0.05 0.06 0.09 0.11 0.15] 手先誤差RMS= 1.7mm ...
```

## 3. コマンド早見表

| やりたいこと | コマンド |
|---|---|
| 新しい端末の準備（毎回） | `cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash` |
| 起動（GUI あり） | `ros2 launch ur10e_control_lab lab.launch.py` |
| 起動できたか確認 | `./scripts/check_lab.sh` |
| 終了 | 起動した端末で `Ctrl+C` → 念のため `./scripts/stop_all.sh` |
| 起動時の制御則・軌道を指定 | `ros2 launch ur10e_control_lab lab.launch.py controller:=mpc reference:=step` |
| 負荷なしの UR10e | `ros2 launch ur10e_control_lab lab.launch.py payload_mass:=0.0` |
| 制御器に負荷を知らせずに起動 | `ros2 launch ur10e_control_lab lab.launch.py model_payload_mass:=0.0` |
| 制御則を切り替え | `ros2 param set /control_lab controller <名前>` |
| 目標軌道を切り替え | `ros2 param set /control_lab reference <hold/step/quintic/sine/circle>` |
| 制御器のモデルから負荷を消す | `ros2 param set /control_lab model_payload_mass 0.0` |
| センサにノイズを加える | `ros2 param set /control_lab position_noise_std 0.002` |
| 状態推定器を切り替え | `ros2 param set /control_lab estimator <none/diff/kf>` |
| YAML を読み直す | `ros2 service call /control_lab/reload std_srvs/srv/Trigger` |
| 手先を押す（外力） | `ros2 run ur10e_control_lab push --force 0 0 -30 --duration 2` |
| 記録の開始・終了 | `ros2 param set /control_lab record true` / `false` |
| 最新の記録をグラフ化 | `ros2 run ur10e_control_lab plot_log $(ls -t ~/ur10e_lab_logs/*.csv \| head -1)` |
| リアルタイムグラフ | `ros2 run rqt_plot rqt_plot /control_lab/error/position[2]` |
| Gazebo で複数制御則を比較 | `./scripts/compare_in_gazebo.sh sine 8 pid computed_torque mpc` |
| オフラインで比較 | `ros2 run ur10e_control_lab offline_sim -c <名前...> -r <軌道>` |
| オフラインでパラメータを上書き | `... offline_sim -c pid --set controllers.pid.ki=0.0` |

制御則の名前（`config/lab_params.yaml` の `controllers:`）:
`pid`, `pid_p_only`, `pid_no_antiwindup`, `pd_gravity`, `computed_torque`, `computed_torque_no_model`,
`lqr`, `lqr_linearized`, `sliding_mode`, `sliding_mode_chattering`, `adaptive`,
`impedance`, `impedance_soft_z`, `mpc`, `mpc_no_preview`, `mpc_no_dob`, `my_controller`

## 4. ファイル構成

```
ur10e_control_lab_ws/
├── README.md                     ← このファイル
├── docs/
│   ├── START_HERE.md             ← ★ 起動・操作マニュアル（最初に読む）
│   ├── 00〜10_*.md               ← 各制御理論の教材（理論・コードの読み方・実験・課題）
│   └── images/                   ← 参考結果のグラフ
├── scripts/
│   ├── check_lab.sh              ← 起動できているかの確認
│   ├── stop_all.sh               ← ラボ関連プロセスをすべて終了
│   └── compare_in_gazebo.sh      ← Gazebo で複数制御則を順に試して比較
└── src/ur10e_control_lab/
    ├── launch/lab.launch.py      ← Gazebo + ros2_control + 制御ノードの起動
    ├── urdf/ur10e_lab.urdf.xacro ← UR10e + 手先負荷 + effort インターフェース
    ├── worlds/control_lab.sdf    ← Gazebo ワールド（外力プラグイン付き）
    ├── config/
    │   ├── lab_params.yaml       ← ★ 制御ゲイン・軌道・ノイズなど全パラメータ
    │   └── ros2_controllers.yaml ← ros2_control 設定（500 Hz, effort）
    ├── ur10e_control_lab/
    │   ├── robot_model.py        ← 動力学モデル（PyKDL）: M(q), C, g(q), J, FK/IK
    │   ├── trajectories.py       ← 目標軌道（step, quintic, sine, circle）
    │   ├── control_loop.py       ← センサ → 推定器 → 制御器 → 飽和 の 1 周期
    │   ├── controllers/          ← ★ 制御則の実装（1 ファイル 1 理論）
    │   │   ├── pid.py  pd_gravity.py  computed_torque.py  lqr.py
    │   │   ├── sliding_mode.py  adaptive.py  impedance.py  mpc.py
    │   │   └── template.py       ← 演習用テンプレート
    │   ├── estimators/kalman.py  ← カルマンフィルタ・差分推定
    │   ├── lab_node.py           ← ROS 2 ノード（Gazebo 用）
    │   ├── offline_sim.py        ← Python だけのシミュレータ
    │   ├── plot_log.py           ← CSV → グラフ
    │   └── push.py               ← Gazebo のアームに外力を加える
    └── test/test_lab.py          ← モデルの性質・各制御器の動作テスト
```

## 5. この環境の「仕掛け」

- **手先に 5 kg の負荷**（オレンジの箱）が付いています。制御器が負荷を知っているかどうか（`model_payload_mass`）を切り替えることで、
  「モデル誤差に弱い制御」と「強い制御（ロバスト・適応）」の違いを体験できます。
- **トルク飽和**: UR10e の定格トルク（330/330/150/54/54/54 Nm）で指令が切られます。`torque_limit_scale` で厳しくできます。
- **センサノイズ**: 関節角にノイズを加え、状態推定（カルマンフィルタ）の効果を確かめられます。
- **外力**: `push` コマンドで手先を押し、インピーダンス制御の柔らかさを確かめられます。

## 6. トラブルシューティング

[起動・操作マニュアル 9 章](docs/START_HERE.md#9-トラブルシューティング) にまとめています。
困ったらまず次の 2 つを試してください:

```bash
./scripts/check_lab.sh     # どこに問題があるか表示
./scripts/stop_all.sh      # すべて終了してから、ros2 launch で起動し直す
```
