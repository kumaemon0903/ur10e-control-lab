# 10. 演習: 自分の制御則を作る

> **準備**: [起動・操作マニュアル](START_HERE.md) の 3〜4 章の手順で、**端末 1** でシミュレーションを起動し（`ros2 launch ur10e_control_lab lab.launch.py`）、
> **端末 2** で `cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash` を済ませておきます。
> 特に断りがなければ、この章のコマンドは端末 2 で実行します。`offline_sim` と `plot_log` は Gazebo を起動していなくても実行できます。
> 終わったら端末 1 で `Ctrl+C`。動きがおかしくなったら `ros2 param set /control_lab reference hold` → `ros2 param set /control_lab controller pd_gravity` で初期姿勢の保持に戻せます。

ここまでの章で、8 種類の制御則と状態推定を見てきました。最後に、自分で制御則を実装して比較してみましょう。

## 10.1 手順

1. [controllers/template.py](../src/ur10e_control_lab/ur10e_control_lab/controllers/template.py) の `compute()` を書き換える
   （初期状態では PD＋重力補償と同じ動作をします）
2. [lab_params.yaml](../src/ur10e_control_lab/config/lab_params.yaml) の `controllers.my_controller` にゲインを書く
3. オフラインで確認（発散しないか、性能はどうか）:
   ```bash
   ros2 run ur10e_control_lab offline_sim -c my_controller pd_gravity computed_torque -r sine
   ```
4. Gazebo で確認:
   ```bash
   # Python を変更した場合は lab_node の再起動が必要（launch を Ctrl+C → 再起動）
   ros2 launch ur10e_control_lab lab.launch.py
   ros2 param set /control_lab controller my_controller
   ```
5. テストを追加して、壊していないことを確認:
   ```bash
   python3 -m pytest src/ur10e_control_lab/test -q
   ```

> **安全のために**: 新しい制御則は必ずオフラインで試してから Gazebo に持ち込みましょう。
> Gazebo でアームが暴れたら `ros2 param set /control_lab controller pd_gravity` で戻せます。

## 10.2 使える部品

```python
# モデル（self.model）
M = self.model.mass_matrix(q)                 # M(q)
c = self.model.coriolis(q, qd)                # C(q,q̇)q̇
cv = self.model.coriolis_times(q, qd, v)      # C(q,q̇)v
g = self.model.gravity(q)                     # g(q)
G = self.model.gravity_jacobian(q)            # ∂g/∂q
p, R = self.model.fk(q)                       # 手先の位置・姿勢
J = self.model.jacobian(q)                    # ヤコビアン (6×6)
Jdqd = self.model.jacobian_dot_qd(q, qd)      # J̇q̇
qdd = self.model.forward_dynamics(q, qd, tau) # 順動力学

# 目標（ref）
ref.q, ref.qd, ref.qdd          # 関節空間
ref.x, ref.xd, ref.xdd, ref.R   # タスク空間

# ログ: compute() の 2 つ目の戻り値の dict → /control_lab/info/<キー> に publish
return tau, {'my_signal': np.array([...])}
```

## 10.3 課題の例

難易度順に並べています。

### ★ 課題 A: PID + 重力補償 + フィードフォワード
PD＋重力補償に、I 項と「目標加速度のフィードフォワード M(q_ref) q̈_ref」を加えてください。
計算トルク法とどちらが良いでしょうか？ モデル誤差（`--set model_payload_mass=0.0`）があるとどうなりますか？

### ★ 課題 B: 固定慣性の計算トルク法
毎周期 M(q) を計算する代わりに、初期姿勢の M(q_home) を固定で使ってください。
`sine` と、大きく姿勢が変わる `quintic` で比べ、慣性の姿勢依存性がどれほど効いているか調べましょう。

### ★★ 課題 C: 受動性に基づく制御（Slotine-Li）
6 章の適応制御から適応部分を取り除くと、次の制御則になります:
```
τ = M q̈_r + C(q,q̇) q̇_r + g − K_d s
```
計算トルク法と比べて、モデル誤差がある場合の性能はどうなるでしょうか？（ヒント: 5, 6 章のコードがほぼそのまま使えます）

### ★★ 課題 D: 外乱オブザーバ付き PD
8 章の外乱オブザーバ（d̂ ≈ q̈_実測 − q̈_指令）を計算トルク法に組み込み、負荷を知らない場合の定常偏差が消えるか確かめてください。

### ★★★ 課題 E: 冗長性の活用（ゼロ空間制御）
インピーダンス制御で並進 3 自由度だけを制御すると（姿勢を自由にすると）、6 − 3 = 3 自由度が余ります。
余った自由度で「関節を初期姿勢に近づける」制御を、ゼロ空間射影 `(I − Jᵀ J^{+ᵀ})` を使って加えてください。

### ★★★ 課題 F: 適応制御の拡張
6 章の適応制御で、負荷の質量 m に加えて重心位置（offset）も推定してください。
パラメータが増えると、静止状態（hold）では推定が収束しなくなるはずです（持続的励振の問題）。本当にそうなるか確かめましょう。

## 10.4 さらに先へ

- **実機との違い**: 実機には関節摩擦、減速機のバックラッシや弾性、通信遅延、トルクセンサの有無などがあります。
  Gazebo の URDF に `<dynamics damping="..." friction="..."/>` を加えると、摩擦の影響を試せます。
- **ros2_control の C++ コントローラ**: 実機で 1 kHz 以上の制御をするには、Python ノードではなく
  ros2_control のコントローラプラグイン（C++）として実装し、リアルタイムループで動かします。
  この教材の Python コードは、そのまま C++ に移植する際の「設計書」として使えます。
- **参考書**
  - 吉川恒夫『ロボット制御基礎論』（コロナ社）— 2・3 章の内容
  - Lynch & Park, *Modern Robotics*（無料 PDF・動画あり）— 運動学・動力学・制御全般
  - Slotine & Li, *Applied Nonlinear Control* — 5・6 章（スライディングモード・適応制御）
  - Siciliano et al., *Robotics: Modelling, Planning and Control* — 7 章（インピーダンス制御）
  - Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design*（無料 PDF）— 8 章
  - Thrun, Burgard & Fox, *Probabilistic Robotics* — 9 章（カルマンフィルタ）
