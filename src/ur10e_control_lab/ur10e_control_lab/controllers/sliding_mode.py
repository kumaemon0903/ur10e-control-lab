"""[5] スライディングモード制御（ロバスト制御）.

誤差 ẽ = q - q_ref に対して「滑り面」

    s = ė̃ + Λ ẽ    （s = 0 上では ẽ が指数的に 0 へ収束）

を定義し、状態を s = 0 に押し付ける。基準速度・加速度を

    q̇_r = q̇_ref - Λ ẽ,   q̈_r = q̈_ref - Λ ė̃      （s = q̇ - q̇_r）

として、制御則

    τ = M̂ q̈_r + Ĉ(q,q̇) q̇_r + ĝ  -  Kd s  -  η · sat(s / φ)
        └──── モデルベース項 ────┘   └ 減衰 ┘  └ 不連続(ロバスト)項 ┘

を用いる（M̂, Ĉ, ĝ はモデル。負荷を知らないので真値とずれている）。
η をモデル誤差の大きさより大きく取れば、誤差があっても s → 0 が保証される。

学習ポイント:
  - boundary_layer φ = 0 だと sign 関数 → 高周波で入力が切り替わる「チャタリング」が起きる
    （/control_lab/command のトルクを見てみよう。実機ではモーターや減速機を傷める）
  - φ > 0 の飽和関数にするとチャタリングは消えるが、|s| < φ の範囲に誤差が残る
  - η を大きくするほどロバストだが、入力は荒くなる → 「ロバスト性 vs 滑らかさ」のトレードオフ
"""
import numpy as np

from .base import Controller, per_joint


class SlidingMode(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.lam = per_joint(cfg.get('lambda', 10.0))
        self.kd = per_joint(cfg.get('kd', 20.0))
        self.eta = per_joint(cfg.get('eta', 20.0))
        self.phi = per_joint(cfg.get('boundary_layer', 0.05))

    def compute(self, t, q, qd, ref):
        e = q - ref.q
        ed = qd - ref.qd
        s = ed + self.lam * e
        qd_r = ref.qd - self.lam * e
        qdd_r = ref.qdd - self.lam * ed

        tau_model = (self.model.mass_matrix(q) @ qdd_r
                     + self.model.coriolis_times(q, qd, qd_r)
                     + self.model.gravity(q))
        with np.errstate(divide='ignore', invalid='ignore'):
            sat = np.where(self.phi > 0, np.clip(s / self.phi, -1.0, 1.0), np.sign(s))
        tau_robust = -self.eta * sat
        tau = tau_model - self.kd * s + tau_robust
        return tau, {'s': s, 'tau_robust': tau_robust}
