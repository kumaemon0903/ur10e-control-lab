"""[8] モデル予測制御（MPC）.

計算トルク法で各関節を二重積分器（q̈ = a）に線形化したうえで、各制御周期に

    min_{a_0..a_{N-1}}  Σ_k  wq (q_k - q_ref,k)² + wv (q̇_k - q̇_ref,k)² + r a_k²
    s.t.  x_{k+1} = A x_k + B a_k          （予測モデル, x = [q, q̇]）
          |a_k| ≤ a_max                     （加速度制約 = 実質的なトルク制約）
          |q̇_k| ≤ v_max                     （速度制約）

という最適化問題（二次計画問題 QP）を解き、最初の a_0 だけを使う（Receding Horizon）。
得られた a_0 から  τ = M(q) a_0 + C q̇ + g  でトルクに変換する。

PID / LQR との違い:
  - 「未来の目標軌道」を先読み (preview) して、早めに動き始められる
  - 制約を直接扱える: 速度・加速度の上限を守りながら最適な動きを計画する
    （PID や LQR では、大きな誤差でトルクが飽和しても制御器はそれを知らない）
  - 代償として毎周期の最適化計算が必要（ここでは小さな QP なので 1 ms 程度）

QP ソルバ: OSQP と同じ ADMM（交互方向乗数法）を numpy で実装（_BatchADMM）。
6 関節は同じ予測モデルを持つので、行列分解を共有して 6 個の QP を同時に解いている。

学習ポイント:
  - reference:=step で、PID（いきなり大トルク→飽和）と MPC（制約内で滑らかに加速）を比較
  - horizon を短くすると先読みが効かず、反応が遅れる
  - モデル誤差（未知負荷）があると「a を指令したのに実際の加速度は a + d」になる。
    disturbance_observer: true で外乱加速度 d を推定し（d̂ ≈ q̈_実測 - a_指令 をローパス）、
    指令から差し引く（オフセットフリー MPC）。false にして model_payload_mass=0 で試すと、
    重力の誤差が加速度制約を超えてしまい、アームを支えきれなくなる様子が観察できる。
"""
import numpy as np

from ..robot_model import NJ
from .base import Controller, per_joint


class _BatchADMM:
    """min ½ uᵀ H u + fᵀ u  s.t.  l ≤ A u ≤ u_  を列ごと（関節ごと）に同時に解く ADMM ソルバ."""

    def __init__(self, H, A, rho=1.0, sigma=1e-6, alpha=1.6, iters=40):
        self.A, self.rho, self.sigma, self.alpha, self.iters = A, rho, sigma, alpha, iters
        n = H.shape[0]
        # x 更新の線形方程式 (H + σI + ρAᵀA) x = ... の係数行列の逆行列を事前計算
        self.Kinv = np.linalg.inv(H + sigma * np.eye(n) + rho * A.T @ A)
        self.x = self.z = self.y = None

    def solve(self, f, lo, hi):
        A, rho, sigma, alpha = self.A, self.rho, self.sigma, self.alpha
        if self.x is None or self.x.shape[1] != f.shape[1]:
            self.x = np.zeros((A.shape[1], f.shape[1]))
            self.z = np.zeros((A.shape[0], f.shape[1]))
            self.y = np.zeros_like(self.z)
        x, z, y = self.x, self.z, self.y  # 前回の解からウォームスタート
        for _ in range(self.iters):
            x_tilde = self.Kinv @ (sigma * x - f + A.T @ (rho * z - y))
            z_tilde = A @ x_tilde
            x = alpha * x_tilde + (1 - alpha) * x
            z_relax = alpha * z_tilde + (1 - alpha) * z
            z = np.clip(z_relax + y / rho, lo, hi)
            y = y + rho * (z_relax - z)
        self.x, self.z, self.y = x, z, y
        return x


class MPC(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.N = int(cfg.get('horizon', 25))
        self.dt = float(cfg.get('dt', 0.02))
        wq = float(cfg.get('w_position', 1000.0))
        wv = float(cfg.get('w_velocity', 10.0))
        r = float(cfg.get('r', 0.01))
        self.a_max = per_joint(cfg.get('accel_limit', 5.0))
        self.v_max = per_joint(cfg.get('velocity_limit', 1.0))
        self.preview = bool(cfg.get('preview', True))
        self.decimation = int(cfg.get('decimation', 1))
        self.use_dob = bool(cfg.get('disturbance_observer', True))
        self.dob_cutoff = float(cfg.get('dob_cutoff_hz', 5.0))

        N, dt = self.N, self.dt
        # 予測行列: 位置 P = Φp x0 + Γp U,  速度 V = Φv x0 + Γv U
        self.Phi_p = np.array([[1.0, (k + 1) * dt] for k in range(N)])
        self.Phi_v = np.array([[0.0, 1.0] for _ in range(N)])
        self.Gam_p = np.zeros((N, N))
        self.Gam_v = np.zeros((N, N))
        for k in range(N):
            for j in range(k + 1):
                self.Gam_v[k, j] = dt
                self.Gam_p[k, j] = (k - j + 0.5) * dt**2
        H = wq * self.Gam_p.T @ self.Gam_p + wv * self.Gam_v.T @ self.Gam_v + r * np.eye(N)
        self.wq, self.wv = wq, wv
        # 制約行列: [I; Γv]（加速度の上下限, 速度の上下限）
        A = np.vstack([np.eye(N), self.Gam_v])
        # スケーリング: H の大きさに合わせて ADMM のペナルティ ρ を決める
        rho = float(cfg.get('admm_rho', np.trace(H) / N))
        self.solver = _BatchADMM(H, A, rho=rho, iters=int(cfg.get('admm_iterations', 40)))
        self.trajectory = None  # 先読み用（ControlLoop が設定する）
        self.reset()

    def reset(self):
        self.count = 0
        self.a_cmd = np.zeros(NJ)      # 計画した加速度（QP の解 U[0]）
        self.a_applied = np.zeros(NJ)  # 実際にトルクへ変換した加速度 = a_cmd - d̂
        self.d_hat = np.zeros(NJ)
        self.qd_prev = self.t_prev = None
        self.solver.x = None

    def _update_disturbance(self, t, qd):
        """外乱オブザーバ: d̂ ← LPF(q̈_実測 - a_applied).  q̈_実測は速度の差分."""
        if self.use_dob and self.t_prev is not None and t > self.t_prev:
            dt = t - self.t_prev
            qdd_meas = (qd - self.qd_prev) / dt
            alpha = dt / (dt + 1.0 / (2 * np.pi * self.dob_cutoff))
            self.d_hat += alpha * (qdd_meas - self.a_applied - self.d_hat)
        self.qd_prev, self.t_prev = qd.copy(), t

    def compute(self, t, q, qd, ref):
        self._update_disturbance(t, qd)
        if self.count % self.decimation == 0:
            if self.preview and self.trajectory is not None:
                q_ref, qd_ref = self.trajectory.preview(t, self.dt, self.N)
            else:  # 先読みなし: 現在の目標がずっと続くと仮定
                q_ref = np.tile(ref.q, (self.N, 1))
                qd_ref = np.tile(ref.qd, (self.N, 1))
            x0 = np.vstack([q, qd])  # (2, NJ)
            P_free = self.Phi_p @ x0  # (N, NJ) 入力 0 のときの予測
            V_free = self.Phi_v @ x0
            f = (self.wq * self.Gam_p.T @ (P_free - q_ref)
                 + self.wv * self.Gam_v.T @ (V_free - qd_ref))
            lo = np.vstack([np.tile(-self.a_max, (self.N, 1)), -self.v_max - V_free])
            hi = np.vstack([np.tile(self.a_max, (self.N, 1)), self.v_max - V_free])
            U = self.solver.solve(f, lo, hi)
            self.a_cmd = U[0]
            self.U = U
        self.count += 1
        # 計画した加速度 a_cmd を実現するため、推定外乱 d̂ を差し引いてトルクに変換
        self.a_applied = self.a_cmd - self.d_hat
        tau = (self.model.mass_matrix(q) @ self.a_applied + self.model.coriolis(q, qd)
               + self.model.gravity(q))
        return tau, {'a_cmd': self.a_cmd.copy(), 'd_hat': self.d_hat.copy()}
