"""[1] 独立関節 PID 制御.

各関節を「独立した 1 入力 1 出力系」とみなし、関節ごとに PID をかける。
ロボットの動力学モデルを一切使わない、最も基本的な制御。

    e  = q_ref - q
    τ  = Kp e + Ki ∫e dt + Kd (q̇_ref - q̇)

学習ポイント:
  - 重力トルク g(q) は「一定の外乱」として働く → P だけだと定常偏差が残る（Ki=0 で試す）
  - I 項が定常偏差を消すが、トルクが飽和している間も積分が溜まり続けると、
    目標に着いた後も溜まった積分が抜けるまで行き過ぎる（ワインドアップ）
    anti_windup: conditional … 飽和中かつ誤差が飽和方向なら積分を止める（条件付き積分）
                 clamp       … 積分項の大きさを integral_limit で制限するだけ
                 none        … 対策なし
  - 関節間の干渉（M(q) の非対角項・コリオリ力）は無視している → 速い軌道で追従が悪化
  - gravity_compensation: true にすると g(q) を足す → 「PD + 重力補償」に近づく
"""
import numpy as np

from .base import Controller, per_joint


class PID(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.kp = per_joint(cfg['kp'])
        self.ki = per_joint(cfg.get('ki', 0.0))
        self.kd = per_joint(cfg['kd'])
        self.anti_windup = cfg.get('anti_windup', 'conditional')
        # 積分項の出力 |Ki ∫e| をこの値 [Nm] 以下に制限（anti_windup: none 以外）
        self.i_limit = per_joint(cfg.get('integral_limit', 1e9))
        self.gravity_comp = bool(cfg.get('gravity_compensation', False))
        self.tau_limit = model.effort_limit  # ControlLoop が実際の上限で上書きする
        self.reset()

    def reset(self):
        self.integral = np.zeros_like(self.kp)
        self.t_prev = None

    def compute(self, t, q, qd, ref):
        e = ref.q - q
        ed = ref.qd - qd
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        self.t_prev = t

        tau_p = self.kp * e
        tau_d = self.kd * ed
        tau_g = self.model.gravity(q) if self.gravity_comp else 0.0

        # 積分（単純な矩形則）
        new_integral = self.integral + e * dt
        if self.anti_windup == 'conditional':
            # 積分を進めた場合の出力が飽和し、かつ誤差が飽和を強める向きなら、その関節は積分しない
            tau_try = tau_p + self.ki * new_integral + tau_d + tau_g
            winding = (np.abs(tau_try) > self.tau_limit) & (np.sign(e) == np.sign(tau_try))
            new_integral = np.where(winding, self.integral, new_integral)
        if self.anti_windup != 'none':
            with np.errstate(divide='ignore', invalid='ignore'):
                lim = np.where(self.ki > 0, self.i_limit / self.ki, np.inf)
            new_integral = np.clip(new_integral, -lim, lim)
        self.integral = new_integral

        tau_i = self.ki * self.integral
        tau = tau_p + tau_i + tau_d + tau_g
        return tau, {'tau_p': tau_p, 'tau_i': tau_i, 'tau_d': tau_d}
