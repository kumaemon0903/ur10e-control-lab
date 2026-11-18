# 起動・操作マニュアル（最初に読む）

このマニュアルの手順だけで、ターミナル（端末）から環境の起動・操作・記録・終了まですべて行えます。
各章（01〜10）の実験は、ここで説明する「端末 1〜3 の構成」が整っている前提で書かれています。

---

## 1. 最初の 1 回だけ: ビルド

端末を開いて（Ubuntu なら `Ctrl+Alt+T`）、次を実行します。

```bash
cd ~/sandbox_for_claude/ur10e_control_lab_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

最後に `Summary: 1 package finished` と表示されれば成功です。

続けて動作確認のテストを実行します（約 10 秒）:

```bash
source install/setup.bash
python3 -m pytest src/ur10e_control_lab/test -q
```

`16 passed` と表示されれば準備完了です。

> **再ビルドが必要なとき**: `setup.py`、`package.xml`、`launch/`、`urdf/`、`worlds/`、`ros2_controllers.yaml` を変更したときは、
> もう一度 `colcon build --symlink-install` を実行してください。
> Python コード（`.py`）と `lab_params.yaml` の変更は再ビルド不要です（反映方法は 7 章）。

---

## 2. 新しい端末を開いたら必ず実行すること

**端末を新しく開くたびに**、次の 2 行を実行してください。これを忘れると `ros2 ... ur10e_control_lab` のコマンドが
`Package 'ur10e_control_lab' not found` などのエラーになります。

```bash
cd ~/sandbox_for_claude/ur10e_control_lab_ws
source install/setup.bash
```

> **毎回打つのが面倒なら**（任意）: 次の 1 行を一度だけ実行すると、以後 `lab` と打つだけで上の 2 行を実行できます。
> ```bash
> echo "alias lab='cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash'" >> ~/.bashrc
> ```
> （この行を追加した後に開いた端末から有効です）

---

## 3. 端末の構成

実験では、端末を 3 つ使うと便利です（端末のタブでも構いません。`Ctrl+Shift+T` で新しいタブ）。
**どの端末でも、開いたら最初に 2 章の 2 行を実行**してください。

| 端末 | 役割 | 実行するもの |
|---|---|---|
| 端末 1 | シミュレータ本体 | `ros2 launch ...`（起動したらそのまま置いておく） |
| 端末 2 | 操作 | `ros2 param set ...`, `push`, `offline_sim` など。各章の実験コマンドは基本的にここで打つ |
| 端末 3 | グラフ表示 | `rqt_plot`（必要なときだけ） |

---

## 4. 起動

### 4.1 端末 1: シミュレーションを起動する

```bash
ros2 launch ur10e_control_lab lab.launch.py
```

起動の流れ（約 10 秒）:
1. Gazebo のウィンドウが開き、UR10e（手先にオレンジの箱 = 5 kg の負荷）が表示される
2. この時点ではシミュレーションは**一時停止**しています（アームが重力で倒れないようにするため）
3. 約 5 秒後に自動で再開し、端末 1 に `data: true` と表示される
4. アームは初期姿勢のまま静止しています（PD＋重力補償で保持）

> 端末 1 には大量のログが流れますが、そのままで問題ありません。ここでは何も入力しないでください。
> `トルク飽和中` という警告は、急な目標変更などでトルクが上限に達したことを示すもので、実験によっては正常です。

**起動時のオプション**（必要なときだけ。末尾に付け足します）:

| オプション | 意味 |
|---|---|
| `gui:=false` | Gazebo の画面を出さない（軽くなる。グラフだけで確認するとき） |
| `controller:=computed_torque` | 起動時の制御則 |
| `reference:=sine` | 起動時の目標軌道 |
| `payload_mass:=0.0` | 手先の負荷を変える（実際の重さ）[kg] |
| `model_payload_mass:=0.0` | 制御器が「知っている」負荷 [kg]（既定は `payload_mass` と同じ） |

例: `ros2 launch ur10e_control_lab lab.launch.py controller:=adaptive reference:=sine payload_mass:=10.0`

### 4.2 端末 2: 正しく起動したか確認する

```bash
./scripts/check_lab.sh
```

次のように `[OK]` が並び、最後に `すべて正常です。` と表示されれば成功です:

```
1. Gazebo
  [OK] Gazebo が動いています
2. ros2_control のコントローラ
  [OK] joint_state_broadcaster: active
  [OK] effort_controller: active
3. 制御ノード /control_lab
  [OK] 動いています
       controller           = pd_gravity
       reference            = hold
       estimator            = none
       model_payload_mass   = 5.0
       position_noise_std   = 0.0
       record               = False
4. 追従誤差（/control_lab/error の position, 単位 rad）
  [OK] array('d', [ ... 0.0001 程度の小さな値 ... ])
```

`[NG]` が出た場合は 9 章「トラブルシューティング」へ。

### 4.3 Gazebo の画面操作

| 操作 | マウス |
|---|---|
| 視点の回転 | 左ドラッグ |
| 平行移動 | 右ドラッグ（またはホイール押し込みドラッグ） |
| ズーム | ホイール |
| 一時停止 / 再開 | 左下の ⏸ / ▶ ボタン |

> 一時停止すると、制御ノードも止まります（シミュレーション時間で動いているため）。再開すれば続きから動きます。

---

## 5. 基本操作（端末 2）

### 5.1 制御則・目標軌道を切り替える

```bash
ros2 param set /control_lab controller computed_torque    # 制御則を切り替え
ros2 param set /control_lab reference sine                # 目標軌道を切り替え
```

成功すると `Set parameter successful` と表示されます。名前を間違えると
`Setting parameter failed: unknown controller ...` と表示され、使える名前の一覧が出ます（設定は元のまま）。

**制御則の名前**（`controller`）:

| 章 | 名前 |
|---|---|
| 1 | `pid`, `pid_p_only`, `pid_no_antiwindup` |
| 2 | `pd_gravity` |
| 3 | `computed_torque`, `computed_torque_no_model` |
| 4 | `lqr`, `lqr_linearized` |
| 5 | `sliding_mode`, `sliding_mode_chattering` |
| 6 | `adaptive` |
| 7 | `impedance`, `impedance_soft_z` |
| 8 | `mpc`, `mpc_no_preview`, `mpc_no_dob` |
| 10 | `my_controller` |

**目標軌道の名前**（`reference`）: `hold`（静止）, `step`, `quintic`, `sine`, `circle`

> **安全な戻し方**: アームの動きがおかしくなったら、次の 2 行で初期姿勢の保持に戻せます。
> ```bash
> ros2 param set /control_lab reference hold
> ros2 param set /control_lab controller pd_gravity
> ```
> 軌道を切り替えると、目標はその時点から最初（初期姿勢）に戻って始まります。
> 別の軌道へ移るときは、一度 `hold` にしてから切り替えると、目標が急に飛ばず安全です。

### 5.2 実験条件を変える

```bash
ros2 param set /control_lab model_payload_mass 0.0      # 制御器のモデルから負荷を消す（モデル誤差の実験）
ros2 param set /control_lab model_payload_mass 5.0      # 元に戻す
ros2 param set /control_lab position_noise_std 0.002    # 関節角センサにノイズ [rad]
ros2 param set /control_lab position_noise_std 0.0      # ノイズなしに戻す
ros2 param set /control_lab estimator kf                # 状態推定器: none / diff / kf
```

### 5.3 現在の設定を見る

```bash
ros2 param get /control_lab controller
ros2 param dump /control_lab            # すべてのパラメータ
./scripts/check_lab.sh                  # まとめて確認
```

### 5.4 手先を押す（外力）

```bash
ros2 run ur10e_control_lab push --force 0 0 -30 --duration 3     # 下向きに 30 N を 3 秒
ros2 run ur10e_control_lab push --force 30 0 0 --duration 0.2    # 横から短く叩く
```

力の向きは Gazebo の座標系（x: 赤、y: 緑、z: 青＝上）です。7 章（インピーダンス制御）で使います。

---

## 6. グラフで見る

### 6.1 リアルタイム表示（端末 3）

```bash
ros2 run rqt_plot rqt_plot /control_lab/error/position[1] /control_lab/error/position[2]
```

- 開いたウィンドウは小さいので、**ウィンドウの端をドラッグして大きくしてください**
- 縦軸が自動で合わないときは、ウィンドウ上部の歯車アイコン（またはツールバーの虫眼鏡）で範囲を調整します
- 閉じるときはウィンドウを閉じるか、端末 3 で `Ctrl+C`

**よく使うトピック**（`[i]` の i は関節番号）:

| トピック | 中身 |
|---|---|
| `/control_lab/error/position[i]` | 追従誤差 q_ref − q [rad] |
| `/control_lab/command/effort[i]` | 指令トルク [Nm] |
| `/control_lab/reference/position[i]` | 目標角 [rad] |
| `/joint_states/position[i]` | 実際の関節角 [rad] |
| `/control_lab/estimate/velocity[i]` | 推定した角速度（9 章） |
| `/control_lab/ee_pose/point/z` | 手先の z 座標 [m] |
| `/control_lab/ee_reference/point/z` | 手先の目標 z 座標 [m] |
| `/control_lab/info/m_hat/data[0]` | 推定した負荷質量（6 章 `adaptive` のときだけ） |
| `/control_lab/info/s/data[i]` | スライディング変数（5, 6 章のときだけ） |

関節番号: 0 = shoulder_pan（根元の旋回）, 1 = shoulder_lift（肩）, 2 = elbow（肘）, 3 = wrist_1, 4 = wrist_2, 5 = wrist_3（手先の回転）

> `/control_lab/info/...` は、**その制御器に切り替えた後にだけ現れる**トピックです。
> 先に `ros2 param set /control_lab controller adaptive` などを実行してから rqt_plot を起動してください。
> どのトピックがあるかは `ros2 topic list | grep control_lab` で確認できます。

### 6.2 記録してグラフにする（端末 2）

```bash
ros2 param set /control_lab record true      # 記録開始
#   ... 実験を行う（制御則・軌道の切り替えなど） ...
ros2 param set /control_lab record false     # 記録終了
```

記録終了時、端末 1 のログに保存先が表示されます:

```
[control_lab]: 記録終了: /home/user/ur10e_lab_logs/20261119_001056_adaptive_sine.csv
  ros2 run ur10e_control_lab plot_log /home/user/ur10e_lab_logs/20261119_001056_adaptive_sine.csv
```

ログが流れて見失った場合は、最新のファイルを次のように探せます:

```bash
ls -t ~/ur10e_lab_logs/*.csv | head -3
ros2 run ur10e_control_lab plot_log $(ls -t ~/ur10e_lab_logs/*.csv | head -1)    # 最新を表示
ros2 run ur10e_control_lab plot_log ファイル1.csv ファイル2.csv                    # 重ねて比較
```

グラフのウィンドウを**閉じると**コマンドが終了します。同じ名前の `.png` も自動で保存されます。

> ファイル名は「記録終了時点の制御則名_軌道名」になります。比較用には、1 回の記録で 1 つの条件にするのがおすすめです。

### 6.3 複数の制御則を自動で比較する

```bash
./scripts/compare_in_gazebo.sh sine 8 pid pd_gravity computed_torque
#                              軌道 秒数 制御則...
```

各制御則について「初期姿勢に戻す → 切り替え → 記録しながら 8 秒実行」を順に行い、最後に比較グラフを表示します。
所要時間は制御則 1 つあたり約（秒数 + 5）秒です。終わるまで端末 2 は使えません（途中でやめるときは `Ctrl+C`）。
グラフは `~/ur10e_lab_logs/compare_<軌道>.png` に保存されます。

---

## 7. ゲインやコードを変えて試す

### 7.1 ゲイン（`lab_params.yaml`）を変える

1. エディタで開いて編集・保存:
   ```bash
   gedit src/ur10e_control_lab/config/lab_params.yaml      # または code / nano / vim など
   ```
2. 端末 2 で再読み込み（Gazebo の再起動は不要）:
   ```bash
   ros2 service call /control_lab/reload std_srvs/srv/Trigger
   ```
   `success=True` と表示されれば反映済みです。YAML の書式が間違っていると `success=False` とエラー内容が表示され、元の設定のまま動き続けます。

> `model_payload_mass` は launch 引数と `ros2 param set` で決まり、reload では変わりません（5.2 節の方法で変えてください）。

### 7.2 Python のコード（制御則など）を変える

1. `src/ur10e_control_lab/ur10e_control_lab/controllers/*.py` などを編集・保存
2. まずオフラインで動作確認（端末 2。Gazebo は起動したままでも構いません）:
   ```bash
   ros2 run ur10e_control_lab offline_sim -c my_controller -r sine
   ```
3. Gazebo に反映するには、**端末 1 で `Ctrl+C` してから、もう一度 `ros2 launch ...` で起動**します
   （起動中のノードは、起動時に読み込んだコードで動き続けるため）

### 7.3 オフラインシミュレータ（Gazebo 不要）

Gazebo を起動していなくても、端末 1 つで使えます。

```bash
ros2 run ur10e_control_lab offline_sim -c pid computed_torque -r sine            # 比較
ros2 run ur10e_control_lab offline_sim -c pid -r step --duration 10              # 時間を指定
ros2 run ur10e_control_lab offline_sim -c pid -r step --set controllers.pid.ki=0.0     # YAML の値を一時的に上書き
ros2 run ur10e_control_lab offline_sim -c pd_gravity -r hold --set model_payload_mass=0.0
ros2 run ur10e_control_lab offline_sim -c impedance -r hold --push 1.0,2.0,0,0,-30     # 1 秒後から 2 秒間 -30N
ros2 run ur10e_control_lab offline_sim -c computed_torque -r sine -e kf --set sensor.position_noise_std=0.002
ros2 run ur10e_control_lab offline_sim --help                                     # すべてのオプション
```

端末に数値の結果（誤差 RMS、最大トルク、飽和率など）が表示され、続いてグラフのウィンドウが開きます（閉じると終了）。
CSV とグラフは `~/ur10e_lab_logs/offline/` に保存されます。グラフを出さないときは `--no-plot` を付けます。

---

## 8. 終了と再起動

### 8.1 終了

端末 1 で **`Ctrl+C`** を押します。Gazebo のウィンドウも閉じます（数秒かかることがあります）。

終了後、念のため残っているプロセスがないか確認・掃除するには:

```bash
./scripts/stop_all.sh
```

`動作中のラボのプロセスはありません。` と表示されればきれいに終了しています。

### 8.2 再起動

1. 端末 1 で `Ctrl+C`
2. `./scripts/stop_all.sh`（念のため）
3. 端末 1 で再び `ros2 launch ur10e_control_lab lab.launch.py`

> **前回の Gazebo が残ったまま起動すると、正しく動きません**（コントローラが有効にならない、アームが倒れるなど）。
> 起動がおかしいときは、まず `./scripts/stop_all.sh` を実行してから起動し直してください。

---

## 9. トラブルシューティング

| 症状 | 原因と対処 |
|---|---|
| `Package 'ur10e_control_lab' not found` | その端末で 2 章の `source install/setup.bash` を実行していない |
| `ros2: コマンドが見つかりません` | 同上 |
| `./scripts/check_lab.sh: そのようなファイルやディレクトリはありません` | `cd ~/sandbox_for_claude/ur10e_control_lab_ws` していない |
| `Parameter not set` / `Node not found` | 端末 1 の launch が起動していない、または起動途中。`./scripts/check_lab.sh` で確認 |
| 起動直後にアームが倒れる | 前回のプロセスが残っていた可能性。`Ctrl+C` → `./scripts/stop_all.sh` → 再起動 |
| `check_lab.sh` で effort_controller が NG | 起動から 15 秒ほど待って再確認。だめなら再起動（8.2） |
| 誤差が届かない（check_lab の 4 が NG） | Gazebo が一時停止中。画面左下の ▶ を押すか、起動から 10 秒待つ |
| アームが暴れて戻らない | 5.1 の「安全な戻し方」。それでもだめなら再起動 |
| `Setting parameter failed: unknown controller` | 名前の誤り。表示される一覧から選ぶ |
| rqt_plot で `topic ... does not exist` | `/control_lab/info/...` は制御器を切り替えてから現れる（6.1）。名前は `ros2 topic list` で確認 |
| rqt_plot のグラフが平らに見える | 縦軸の範囲が合っていない。ウィンドウを広げ、虫眼鏡アイコンで調整 |
| reload が `success=False` | YAML の書式誤り（インデント、`:` の後の空白など）。エラー文を見て修正 |
| Gazebo が重い・カクカクする | `gui:=false` で起動し、rqt_plot や記録で確認する |
| 修正した Python コードが反映されない | 端末 1 で launch を再起動（7.2） |
| `offline_sim` で `発散しました` | ゲインが不安定。値を戻すか小さくする（教材の課題で意図的に起こす場合もあります） |

---

## 10. 1 回分の実験の流れ（例: 3 章 計算トルク法）

```bash
# --- 端末 1 ---
cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash
ros2 launch ur10e_control_lab lab.launch.py

# --- 端末 2 ---
cd ~/sandbox_for_claude/ur10e_control_lab_ws && source install/setup.bash
./scripts/check_lab.sh                                   # すべて正常 を確認
ros2 param set /control_lab controller computed_torque
ros2 param set /control_lab reference sine               # Gazebo でアームが動き出す
ros2 param set /control_lab record true
#   （10 秒ほど待つ）
ros2 param set /control_lab record false
ros2 run ur10e_control_lab plot_log $(ls -t ~/ur10e_lab_logs/*.csv | head -1)    # グラフを確認して閉じる

ros2 param set /control_lab model_payload_mass 0.0        # モデル誤差を入れると…
#   （Gazebo で手首の動きが乱れるのを観察）
ros2 param set /control_lab model_payload_mass 5.0        # 戻す
ros2 param set /control_lab reference hold

# --- 終了: 端末 1 で Ctrl+C、その後 端末 2 で ---
./scripts/stop_all.sh
```

準備ができたら [0. 環境の仕組み](00_architecture.md) から始めましょう。
