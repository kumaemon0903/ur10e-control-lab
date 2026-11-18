"""制御ループ本体（ROS / Gazebo に依存しない部分）.

    真の状態 (q, q̇) ──▶ [センサモデル] ──▶ [推定器] ──▶ [制御器] ──▶ [トルク飽和] ──▶ τ
                         ノイズ付加          none/diff/kf       ▲
                                                               │
                                              [目標軌道] ──────┘

lab_node（Gazebo 用）と offline_sim（Python だけのシミュレータ）の両方がこのクラスを使うので、
同じ制御コードを両方の環境で試せる。
"""
import os

import numpy as np
import yaml

from .controllers import make_controller
from .estimators import make_estimator
from .robot_model import NJ, RobotModel
from .trajectories import make_trajectory


def default_params_file():
    """lab_params.yaml の場所.

    colcon build --symlink-install の場合、このファイルはソースツリーへのシンボリックリンク
    経由で読み込まれているので、ソース側の config/lab_params.yaml を優先する
    （編集 → 再ビルドなしで reload できるようにするため）。
    """
    src = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'lab_params.yaml')
    if os.path.exists(src):
        return os.path.normpath(src)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ur10e_control_lab'),
                        'config', 'lab_params.yaml')


def load_params(path=None):
    with open(path or default_params_file()) as f:
        return yaml.safe_load(f)


class ControlLoop:

    def __init__(self, params: dict, controller='pd_gravity', reference='hold', estimator=None):
        self.params = params
        self.home = np.asarray(params['home'], dtype=float)
        # 制御器が使うモデル。model_payload_mass=0 なら「負荷を知らない」制御器になる
        # （未指定なら Gazebo の既定負荷 5 kg。launch / offline_sim は実際の負荷を渡す）
        self.model = RobotModel(payload_mass=float(params.get('model_payload_mass', 5.0)))
        self.tau_limit = self.model.effort_limit * float(params.get('torque_limit_scale', 1.0))
        sensor = params.get('sensor', {}) or {}
        self.noise_q = float(sensor.get('position_noise_std', 0.0))
        self.noise_qd = float(sensor.get('velocity_noise_std', 0.0))
        self.rng = np.random.default_rng(sensor.get('seed', 0))
        self.tau_prev = None
        self.t0 = None
        self.controller_name = self.reference_name = self.estimator_name = None
        self.set_reference(reference)
        self.set_controller(controller)
        self.set_estimator(estimator or params.get('estimator', {}).get('type', 'none'))

    # ---------- 実行中の切り替え ----------
    def set_controller(self, name):
        self.controller = make_controller(name, self.model, self.params['controllers'])
        if hasattr(self.controller, 'tau_limit'):
            self.controller.tau_limit = self.tau_limit  # PID のアンチワインドアップ用
        self.controller.reset()
        if hasattr(self.controller, 'trajectory'):
            self.controller.trajectory = _ShiftedTrajectory(self.trajectory, self)  # MPC の先読み用
        self.controller_name = name

    def set_reference(self, name, t_start=None):
        self.trajectory = make_trajectory(name, self.model, self.home,
                                          self.params.get('references', {}))
        self.reference_name = name
        self.ref_t0 = t_start  # 軌道は切り替えた時刻を t=0 として始まる
        if getattr(self, 'controller', None) is not None and hasattr(self.controller, 'trajectory'):
            self.controller.trajectory = _ShiftedTrajectory(self.trajectory, self)

    def set_estimator(self, name):
        self.estimator = make_estimator(name, self.model, self.params.get('estimator', {}))
        self.estimator_name = name

    # ---------- 1 制御周期 ----------
    def step(self, t, q_true, qd_true):
        """真の状態から指令トルクを計算する. 戻り値: (tau, log_dict)."""
        if self.ref_t0 is None:
            self.ref_t0 = t
        q_meas = q_true + self.rng.normal(0.0, self.noise_q, NJ) if self.noise_q > 0 else q_true
        qd_meas = qd_true + self.rng.normal(0.0, self.noise_qd, NJ) if self.noise_qd > 0 else qd_true
        q_hat, qd_hat = self.estimator.update(t, q_meas, qd_meas, self.tau_prev)

        ref = self.trajectory(t - self.ref_t0)
        tau_raw, info = self.controller.compute(t, q_hat, qd_hat, ref)
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)
        self.tau_prev = tau
        return tau, {
            'ref': ref, 'q_hat': q_hat, 'qd_hat': qd_hat, 'q_meas': q_meas,
            'tau_raw': tau_raw, 'saturated': np.any(tau != tau_raw), 'info': info,
        }


class _ShiftedTrajectory:
    """MPC の先読み用: 軌道の開始時刻のずれを吸収するラッパ."""

    def __init__(self, traj, loop):
        self.traj, self.loop = traj, loop

    def preview(self, t, dt, n):
        t0 = self.loop.ref_t0 if self.loop.ref_t0 is not None else t
        return self.traj.preview(t - t0, dt, n)


def log_header():
    cols = ['t']
    for prefix in ('q', 'qd', 'q_ref', 'qd_ref', 'q_hat', 'qd_hat', 'tau'):
        cols += [f'{prefix}{i}' for i in range(NJ)]
    cols += ['x', 'y', 'z', 'x_ref', 'y_ref', 'z_ref', 'm_hat', 'saturated']
    return cols


def log_row(model, t, q, qd, tau, log):
    ref = log['ref']
    x, _ = model.fk(q)
    m_hat = log['info'].get('m_hat', [np.nan])[0]
    return np.concatenate([[t], q, qd, ref.q, ref.qd, log['q_hat'], log['qd_hat'], tau,
                           x, ref.x, [m_hat, float(log['saturated'])]])
