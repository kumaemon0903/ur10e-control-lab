"""[2] PD + 重力補償 制御（有本・高木の制御則）.

    τ = Kp (q_ref - q) + Kd (q̇_ref - q̇) + g(q)

モデルのうち重力項 g(q) だけを使う。目標が静止点 (q̇_ref = 0) のとき、
リアプノフ関数

    V = ½ q̇ᵀ M(q) q̇ + ½ eᵀ Kp e

を使うと、M(q) の性質 (Ṁ - 2C が歪対称) から  V̇ = -q̇ᵀ Kd q̇ ≤ 0  となり、
LaSalle の不変性原理により 大域的漸近安定 が言える（ゲインが正なら必ず安定）。

学習ポイント:
  - PID と違い I 項なしで定常偏差が出ない（重力を打ち消しているので）
  - ただし「モデルの g(q)」と「実際の重力」がずれていると定常偏差が残る
    → 手先負荷 (payload_mass) を知らないモデルでは、負荷の重さ分だけ垂れ下がる
  - 軌道追従（q̇_ref ≠ 0）では慣性力を補償しないので遅れが出る
"""
from .base import Controller, per_joint


class PDGravity(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.kp = per_joint(cfg['kp'])
        self.kd = per_joint(cfg['kd'])

    def compute(self, t, q, qd, ref):
        g = self.model.gravity(q)
        tau = self.kp * (ref.q - q) + self.kd * (ref.qd - qd) + g
        return tau, {'tau_gravity': g}
