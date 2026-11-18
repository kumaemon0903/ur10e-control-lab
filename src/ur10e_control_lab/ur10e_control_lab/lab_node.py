"""制御ノード: Gazebo の UR10e を、選んだ制御則でトルク制御する.

  /joint_states ──▶ [ControlLoop: 推定器 → 制御器 → 飽和] ──▶ /effort_controller/commands

/joint_states が届くたびに（500 Hz）1 回制御計算を行う。

実行中の切り替え（Gazebo は起動したまま）:
  ros2 param set /control_lab controller computed_torque
  ros2 param set /control_lab reference sine
  ros2 param set /control_lab estimator kf
  ros2 param set /control_lab record true         # CSV 記録 開始 / false で終了
  ros2 param set /control_lab model_payload_mass 0.0   # 制御器のモデルから負荷を消す（モデル誤差）
  ros2 param set /control_lab position_noise_std 0.002 # 関節角センサにノイズを加える [rad]
  ros2 service call /control_lab/reload std_srvs/srv/Trigger   # YAML を読み直す

可視化用トピック（rqt_plot / PlotJuggler で見る）:
  /control_lab/reference   JointState  position=q_ref, velocity=q̇_ref
  /control_lab/error       JointState  position=q_ref-q [rad], velocity=q̇_ref-q̇
  /control_lab/command     JointState  effort=τ（飽和後）
  /control_lab/estimate    JointState  position=q̂, velocity=q̂̇（推定器の出力）
  /control_lab/ee_pose     PointStamped 手先位置
  /control_lab/ee_reference PointStamped 手先目標位置
  /control_lab/info/<名前> Float64MultiArray 制御器固有の信号（s, m_hat, F など）
"""
import copy
import datetime
import os
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .control_loop import ControlLoop, default_params_file, load_params, log_header, log_row
from .robot_model import JOINT_NAMES, NJ


class LabNode(Node):

    def __init__(self):
        super().__init__('control_lab')
        self.declare_parameter('params_file', '')
        self.declare_parameter('controller', 'pd_gravity')
        self.declare_parameter('reference', 'hold')
        self.declare_parameter('estimator', '')
        self.declare_parameter('record', False)
        # 以下 2 つは負の値なら YAML の値を使う
        self.declare_parameter('model_payload_mass', -1.0)
        self.declare_parameter('position_noise_std', -1.0)

        self.params_path = self.get_parameter('params_file').value or default_params_file()
        self.params = self._apply_overrides(load_params(self.params_path))
        self.get_logger().info(f'パラメータ: {self.params_path}')
        self.loop = ControlLoop(
            self.params,
            controller=self.get_parameter('controller').value,
            reference=self.get_parameter('reference').value,
            estimator=self.get_parameter('estimator').value or None)
        self._log_status('開始')
        # 「YAML の値を使う」を意味する空文字・負の値を、実際に使っている値で置き換えておく
        # （ros2 param get で現在の設定が分かるように。コールバック登録前なので再構築は起きない）
        self.set_parameters([
            Parameter('estimator', value=self.loop.estimator_name),
            Parameter('model_payload_mass', value=float(self.params.get('model_payload_mass', 5.0))),
            Parameter('position_noise_std',
                      value=float(self.params.get('sensor', {}).get('position_noise_std', 0.0))),
        ])

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.ref_pub = self.create_publisher(JointState, '~/reference', 10)
        self.err_pub = self.create_publisher(JointState, '~/error', 10)
        self.tau_pub = self.create_publisher(JointState, '~/command', 10)
        self.est_pub = self.create_publisher(JointState, '~/estimate', 10)
        self.ee_pub = self.create_publisher(PointStamped, '~/ee_pose', 10)
        self.ee_ref_pub = self.create_publisher(PointStamped, '~/ee_reference', 10)
        self.info_pubs = {}

        self.create_subscription(JointState, '/joint_states', self.on_joint_state,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '~/reload', self.on_reload)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # 最初の /joint_states が届くまで（シミュレーション一時停止中）は、初期姿勢の重力トルクを
        # 送り続ける。sim time は停止中進まないので、壁時計のタイマを使う。
        self.got_state = False
        self.hold_tau = self.loop.model.gravity(self.loop.home)
        self.hold_timer = self.create_timer(0.01, self.publish_hold,
                                            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.csv = None
        self.n_cb = 0
        self.t_last = None

    # ------------------------------------------------------------------
    def _apply_overrides(self, params, overrides=None):
        """ROS パラメータ model_payload_mass / position_noise_std で YAML の値を上書きする."""
        overrides = overrides or {}
        m = overrides.get('model_payload_mass', self.get_parameter('model_payload_mass').value)
        n = overrides.get('position_noise_std', self.get_parameter('position_noise_std').value)
        if m >= 0:
            params['model_payload_mass'] = float(m)
        if n >= 0:
            params.setdefault('sensor', {})['position_noise_std'] = float(n)
        return params

    def _rebuild(self, params):
        """設定を変えて ControlLoop を作り直す（制御器・軌道・推定器の選択は引き継ぐ）."""
        loop = ControlLoop(params, self.loop.controller_name, self.loop.reference_name,
                           self.loop.estimator_name)
        loop.set_reference(self.loop.reference_name, t_start=self.loop.ref_t0)
        self.params, self.loop = params, loop

    def _log_status(self, prefix):
        self.get_logger().info(
            f'{prefix}: controller={self.loop.controller_name}  reference={self.loop.reference_name}'
            f'  estimator={self.loop.estimator_name}'
            f'  model_payload_mass={self.params.get("model_payload_mass")}'
            f'  noise={self.params.get("sensor", {}).get("position_noise_std")}')

    def publish_hold(self):
        if self.got_state:
            self.hold_timer.cancel()
            return
        self.cmd_pub.publish(Float64MultiArray(data=self.hold_tau.tolist()))

    def on_joint_state(self, msg: JointState):
        try:
            idx = [msg.name.index(n) for n in JOINT_NAMES]
        except ValueError:
            return
        q = np.array([msg.position[i] for i in idx])
        qd = np.array([msg.velocity[i] for i in idx]) if msg.velocity else np.zeros(NJ)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_last is not None and t <= self.t_last:
            return  # 同じ時刻の重複メッセージは無視
        self.t_last = t
        self.got_state = True

        try:
            tau, log = self.loop.step(t, q, qd)
        except Exception:  # 制御器の例外でノードが落ちると、アームが脱力してしまう
            self.get_logger().error(traceback.format_exc(), throttle_duration_sec=2.0)
            tau = self.loop.model.gravity(q)
            self.cmd_pub.publish(Float64MultiArray(data=tau.tolist()))
            return
        self.cmd_pub.publish(Float64MultiArray(data=tau.tolist()))
        self.publish_debug(msg.header.stamp, q, qd, tau, log)
        if self.csv is not None:
            row = log_row(self.loop.model, t, q, qd, tau, log)
            self.csv.write(','.join(f'{v:.6g}' for v in row) + '\n')
        self.n_cb += 1
        if log['saturated']:
            self.get_logger().warn('トルク飽和中', throttle_duration_sec=2.0)

    def publish_debug(self, stamp, q, qd, tau, log):
        ref = log['ref']

        def js(**kw):
            m = JointState()
            m.header.stamp = stamp
            m.name = JOINT_NAMES
            for k, v in kw.items():
                setattr(m, k, np.asarray(v, dtype=float).tolist())
            return m

        self.ref_pub.publish(js(position=ref.q, velocity=ref.qd))
        self.err_pub.publish(js(position=ref.q - q, velocity=ref.qd - qd))
        self.tau_pub.publish(js(effort=tau))
        self.est_pub.publish(js(position=log['q_hat'], velocity=log['qd_hat']))
        x, _ = self.loop.model.fk(q)
        for pub, p in ((self.ee_pub, x), (self.ee_ref_pub, ref.x)):
            m = PointStamped()
            m.header.stamp = stamp
            m.header.frame_id = 'base_link'
            m.point.x, m.point.y, m.point.z = (float(v) for v in p)
            pub.publish(m)
        for key, val in log['info'].items():
            if key not in self.info_pubs:
                self.info_pubs[key] = self.create_publisher(Float64MultiArray, f'~/info/{key}', 10)
            self.info_pubs[key].publish(
                Float64MultiArray(data=np.atleast_1d(np.asarray(val, dtype=float)).tolist()))

    # ------------------------------------------------------------------
    def on_set_parameters(self, params):
        if getattr(self, '_syncing', False):
            return SetParametersResult(successful=True)
        for p in params:
            try:
                if p.name == 'controller':
                    self.loop.set_controller(p.value)
                elif p.name == 'reference':
                    t = self.t_last
                    self.loop.set_reference(p.value, t_start=t)
                elif p.name == 'estimator':
                    self.loop.set_estimator(p.value or 'none')
                elif p.name == 'record':
                    self.set_record(bool(p.value))
                    continue
                elif p.name in ('model_payload_mass', 'position_noise_std'):
                    params = self._apply_overrides(copy.deepcopy(self.params), {p.name: p.value})
                    self._rebuild(params)
                else:
                    continue
            except Exception as e:  # 不正な名前など: 切り替えを拒否して元のまま続ける
                return SetParametersResult(successful=False, reason=str(e))
            self._log_status('切り替え')
        return SetParametersResult(successful=True)

    def set_record(self, on):
        if on and self.csv is None:
            d = os.path.expanduser(self.params.get('log', {}).get('csv_dir', '~/ur10e_lab_logs'))
            os.makedirs(d, exist_ok=True)
            self.csv_stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(d, f'{self.csv_stamp}_recording.csv')
            self.csv = open(path, 'w')
            self.csv.write(','.join(log_header()) + '\n')
            self.csv_path = path
            self.get_logger().info(f'記録開始: {path}')
        elif not on and self.csv is not None:
            self.csv.close()
            self.csv = None
            # ファイル名は記録終了時点の制御器・軌道名にする（記録中に切り替えることが多いため）
            final = os.path.join(os.path.dirname(self.csv_path),
                                 f'{self.csv_stamp}_{self.loop.controller_name}_{self.loop.reference_name}.csv')
            os.rename(self.csv_path, final)
            self.get_logger().info(f'記録終了: {final}\n'
                                   f'  ros2 run ur10e_control_lab plot_log {final}')

    def on_reload(self, request, response):
        try:
            params = load_params(self.params_path)
            # センサノイズは YAML の値を採用。モデルの負荷質量は launch / ros2 param の値を引き継ぐ
            noise = float(params.get('sensor', {}).get('position_noise_std', 0.0))
            self._rebuild(self._apply_overrides(params, {'position_noise_std': noise}))
        except Exception as e:
            response.success = False
            response.message = f'読み込み失敗（元の設定のまま）: {e}'
            return response
        self._syncing = True  # 表示用にパラメータを更新するだけ（再構築しない）
        self.set_parameters([Parameter('position_noise_std', value=noise)])
        self._syncing = False
        self._log_status('再読み込み')
        response.success = True
        response.message = f'reloaded {self.params_path}'
        return response

    def destroy_node(self):
        if self.csv is not None:
            self.csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LabNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
