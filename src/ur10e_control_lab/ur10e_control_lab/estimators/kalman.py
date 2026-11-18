"""[9] 状態推定: カルマンフィルタ と 比較用の差分推定器.

実機の関節センサ（エンコーダ）は角度しか測れず、ノイズ・量子化誤差を含む。
制御に必要な角速度 q̇ をどう得るかは重要な問題。

Diff（差分 + ローパス）:
    q̇ ≈ (q_k - q_{k-1}) / dt を一次ローパスフィルタで平滑化
    → 簡単だが、ノイズを抑えるほど遅れ（位相遅れ）が大きくなる

KalmanFilter（各関節独立、状態 x = [q, q̇]）:
    予測:  x⁻ = F x + G a            a = M⁻¹(τ - C q̇ - g)  （モデルから求めた加速度）
           P⁻ = F P Fᵀ + Q
    更新:  K  = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
           x  = x⁻ + K (y - H x⁻)      y = 測定した q
           P  = (I - K H) P⁻
    F = [[1, dt], [0, 1]], G = [dt²/2, dt], H = [1, 0]

    Q（プロセスノイズ）: 「モデル（加速度）がどれだけ信用できないか」
    R（観測ノイズ）    : 「センサがどれだけ信用できないか」
    → Q/R の比が「モデルとセンサのどちらを信じるか」を決める

学習ポイント:
  - sensor.position_noise_std を 0.001 (≈0.06°) 程度に上げ、estimator を none/diff/kf で切り替え
  - diff だと速度が非常にノイジー → D 項がノイズを増幅しトルクが暴れる
  - kf はモデルの加速度予測を使うので、遅れが小さく滑らか
"""
import numpy as np

from ..robot_model import NJ


class Ideal:
    """シミュレータの真値をそのまま使う（ノイズのない理想センサ）."""

    def reset(self):
        pass

    def update(self, t, q_meas, qd_meas, tau_prev):
        return q_meas, qd_meas


class Diff:

    def __init__(self, model, cfg):
        self.fc = float(cfg.get('cutoff_hz', 30.0))
        self.reset()

    def reset(self):
        self.q_prev = None
        self.t_prev = None
        self.qd = np.zeros(NJ)

    def update(self, t, q_meas, qd_meas, tau_prev):
        if self.q_prev is not None and t > self.t_prev:
            dt = t - self.t_prev
            raw = (q_meas - self.q_prev) / dt
            alpha = dt / (dt + 1.0 / (2 * np.pi * self.fc))  # 一次ローパス
            self.qd = self.qd + alpha * (raw - self.qd)
        self.q_prev, self.t_prev = q_meas.copy(), t
        return q_meas, self.qd.copy()


class KalmanFilter:

    def __init__(self, model, cfg):
        self.model = model
        self.sigma_a = float(cfg.get('process_noise_accel', 5.0))      # [rad/s²]
        self.sigma_q = float(cfg.get('measurement_noise_position', 1e-3))  # [rad]
        self.use_model = bool(cfg.get('use_model_acceleration', True))
        self.reset()

    def reset(self):
        self.x = None  # (2, NJ): [q; q̇]
        self.P = None  # (NJ, 2, 2)
        self.t_prev = None

    def update(self, t, q_meas, qd_meas, tau_prev):
        if self.x is None:
            self.x = np.vstack([q_meas, np.zeros(NJ)])
            self.P = np.tile(np.diag([self.sigma_q**2, 1.0]), (NJ, 1, 1))
            self.t_prev = t
            return q_meas, np.zeros(NJ)

        dt = max(t - self.t_prev, 0.0)
        self.t_prev = t
        q, qd = self.x
        # --- 予測 ---
        if self.use_model and tau_prev is not None:
            a = self.model.forward_dynamics(q, qd, tau_prev)
        else:
            a = np.zeros(NJ)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Gv = np.array([0.5 * dt**2, dt])
        self.x = F @ self.x + np.outer(Gv, a)
        Q = self.sigma_a**2 * np.outer(Gv, Gv)
        self.P = F @ self.P @ F.T + Q
        # --- 更新（観測は位置のみ: H = [1, 0]）---
        S = self.P[:, 0, 0] + self.sigma_q**2             # (NJ,)
        K = self.P[:, :, 0] / S[:, None]                   # (NJ, 2)
        innov = q_meas - self.x[0]
        self.x = self.x + (K * innov[:, None]).T
        # P = (I - K H) P
        KH = np.zeros((NJ, 2, 2))
        KH[:, :, 0] = K
        self.P = self.P - KH @ self.P
        return self.x[0].copy(), self.x[1].copy()


def make_estimator(name, model, cfg_all):
    cfg = (cfg_all or {}).get(name, {}) or {}
    if name in ('none', 'ideal'):
        return Ideal()
    if name == 'diff':
        return Diff(model, cfg)
    if name == 'kf':
        return KalmanFilter(model, cfg)
    raise ValueError(f'unknown estimator "{name}". choose from none/diff/kf')
