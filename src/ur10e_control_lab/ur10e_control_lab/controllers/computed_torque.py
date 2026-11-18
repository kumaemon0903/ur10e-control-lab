"""[3] 計算トルク法（逆動力学制御・フィードバック線形化）.

運動方程式  M(q) q̈ + C(q,q̇) q̇ + g(q) = τ  に対して

    τ = M(q) v + C(q,q̇) q̇ + g(q)

とすると、モデルが正確なら  q̈ = v  （各関節が独立な二重積分器）になる。
これを「フィードバック線形化」と呼ぶ。新しい入力 v を

    v = q̈_ref + Kd (q̇_ref - q̇) + Kp (q_ref - q)

と選ぶと、誤差 e = q_ref - q は

    ë + Kd ė + Kp e = 0

に従う。Kp = ωn², Kd = 2ζωn と選べば、全関節が「固有角振動数 ωn, 減衰比 ζ」の
2 次系として同じように振る舞う（姿勢によらない！）。

学習ポイント:
  - ゲインを「物理的な意味（ωn, ζ）」で設計できる
  - 軌道追従性能が PID / PD+重力補償 より格段に良い（q̈_ref のフィードフォワード）
  - モデル誤差（未知の手先負荷）があると完全には線形化できず誤差が残る
    → model_payload_mass を変えて比較してみよう
"""
import numpy as np

from .base import Controller, per_joint


class ComputedTorque(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        wn = per_joint(cfg.get('wn', 10.0))
        zeta = per_joint(cfg.get('zeta', 1.0))
        self.kp = wn**2
        self.kd = 2.0 * zeta * wn
        self.feedforward = bool(cfg.get('feedforward', True))

    def compute(self, t, q, qd, ref):
        e = ref.q - q
        ed = ref.qd - qd
        qdd_ff = ref.qdd if self.feedforward else np.zeros_like(ref.qdd)
        v = qdd_ff + self.kd * ed + self.kp * e
        M = self.model.mass_matrix(q)
        tau = M @ v + self.model.coriolis(q, qd) + self.model.gravity(q)
        return tau, {'v': v}
