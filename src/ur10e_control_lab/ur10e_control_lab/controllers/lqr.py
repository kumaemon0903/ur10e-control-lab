"""[4] LQR（線形二次レギュレータ）.

線形システム ẋ = A x + B u に対して、評価関数

    J = ∫ (xᵀ Q x + uᵀ R u) dt

を最小にする状態フィードバック u = -K x を求める。K は Riccati 方程式

    Aᵀ P + P A - P B R⁻¹ Bᵀ P + Q = 0,   K = R⁻¹ Bᵀ P

の解 P から得られる（scipy.linalg.solve_continuous_are）。

ロボットは非線形なので、2 つの使い方を用意した:

mode: feedback_linearized（既定）
    計算トルク法で q̈ = v に線形化してから、各関節の二重積分器に LQR を適用。
    状態 x = [∫e, e, ė]（積分項付き）, 入力 u = v - q̈_ref.
    → 「計算トルク法の Kp, Kd を Q, R から最適に決める」方法と言える。
      (積分を入れると負荷によるモデル誤差の定常偏差も消える)

mode: linearized_model
    目標姿勢 q_ref のまわりで運動方程式を線形化:
        M(q₀) δq̈ + G₀ δq = δτ,   G₀ = ∂g/∂q |q₀
    状態 x = [δq, δq̇] (12 次元) で LQR を解き、τ = τ_ff(q_ref) - K x。
    関節間の結合（M の非対角成分・重力の結合）を考慮した 12x6 のゲイン行列になる。
    姿勢が変わると線形化をやり直す（ゲインスケジューリング）。

学習ポイント:
  - Q を大きく → 誤差に厳しい（速い・強い応答、トルク大）
  - R を大きく → 入力に厳しい（ゆっくり・省エネ）
  - 実は Q, R の比だけが応答を決める（両方 10 倍しても K は同じ）
"""
import numpy as np
from scipy.linalg import solve_continuous_are

from ..robot_model import NJ
from .base import Controller, per_joint


def lqr_gain(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


class LQR(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.mode = cfg.get('mode', 'feedback_linearized')
        self.q_int = per_joint(cfg.get('q_integral', 0.0))
        self.q_pos = per_joint(cfg.get('q_position', 100.0))
        self.q_vel = per_joint(cfg.get('q_velocity', 1.0))
        self.r = per_joint(cfg.get('r', 1.0))
        self.relinearize_period = cfg.get('relinearize_period', 0.1)
        self.relinearize_threshold = cfg.get('relinearize_threshold', 0.05)

        if self.mode == 'feedback_linearized':
            self.K = self._design_double_integrator()
        elif self.mode != 'linearized_model':
            raise ValueError(f'unknown LQR mode: {self.mode}')
        self.reset()

    def _design_double_integrator(self):
        """各関節: 状態 [∫e, e, ė], 入力 u（= 追加の加速度）について LQR を解く."""
        gains = []
        use_int = np.any(self.q_int > 0)
        for i in range(NJ):
            if use_int:
                A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]], dtype=float)
                B = np.array([[0], [0], [1]], dtype=float)
                Q = np.diag([self.q_int[i], self.q_pos[i], self.q_vel[i]])
            else:
                A = np.array([[0, 1], [0, 0]], dtype=float)
                B = np.array([[0], [1]], dtype=float)
                Q = np.diag([self.q_pos[i], self.q_vel[i]])
            K = lqr_gain(A, B, Q, np.array([[self.r[i]]]))
            gains.append(K.ravel() if use_int else np.concatenate([[0.0], K.ravel()]))
        return np.array(gains)  # (NJ, 3): [k_int, k_pos, k_vel]

    def _design_linearized(self, q0):
        M = self.model.mass_matrix(q0)
        G = self.model.gravity_jacobian(q0)
        Minv = np.linalg.inv(M)
        A = np.block([[np.zeros((NJ, NJ)), np.eye(NJ)], [-Minv @ G, np.zeros((NJ, NJ))]])
        B = np.vstack([np.zeros((NJ, NJ)), Minv])
        Q = np.diag(np.concatenate([self.q_pos, self.q_vel]))
        R = np.diag(self.r)
        return lqr_gain(A, B, Q, R)  # (6, 12)

    def reset(self):
        self.integral = np.zeros(NJ)
        self.t_prev = None
        self.q_lin = None
        self.t_lin = -np.inf

    def compute(self, t, q, qd, ref):
        e = ref.q - q
        ed = ref.qd - qd
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        self.t_prev = t

        if self.mode == 'feedback_linearized':
            self.integral += e * dt
            # u = -K x, x = -[∫e, e, ė]（誤差の定義が「目標 - 現在」なので符号に注意）
            v = ref.qdd + self.K[:, 0] * self.integral + self.K[:, 1] * e + self.K[:, 2] * ed
            tau = (self.model.mass_matrix(q) @ v + self.model.coriolis(q, qd)
                   + self.model.gravity(q))
            return tau, {'v': v}

        # linearized_model: 目標が一定以上動いたら、または一定時間ごとに再線形化
        if (self.q_lin is None or
                (t - self.t_lin > self.relinearize_period and
                 np.max(np.abs(ref.q - self.q_lin)) > self.relinearize_threshold)):
            self.K_full = self._design_linearized(ref.q)
            self.q_lin, self.t_lin = ref.q.copy(), t
        tau_ff = self.model.inverse_dynamics(ref.q, ref.qd, ref.qdd)
        x = np.concatenate([q - ref.q, qd - ref.qd])
        tau = tau_ff - self.K_full @ x
        return tau, {}
