"""[6] 適応制御（Slotine-Li 型）: 未知の手先負荷の質量をオンライン推定する.

ロボットの運動方程式は「物理パラメータ θ について線形」に書ける:

    M(q) q̈ + C(q,q̇) q̇ + g(q) = Y(q, q̇, q̇, q̈) θ

ここでは未知パラメータを「手先負荷の質量 m」1 つに絞る。負荷を tool0 から
offset の位置にある質点とみなすと、その点のヤコビアン Jp (3x6) を使って

    τ_payload = m · Jpᵀ ( Jp q̈_r + J̇p q̇_r + [0, 0, g]ᵀ ) = Y m

と書ける（Y が 6x1 の回帰ベクトル）。スライディングモードと同じ s, q̇_r, q̈_r を使い

    τ   = τ_model(負荷なし) + Y m̂ - Kd s           … 制御則
    dm̂/dt = -Γ Yᵀ s                                 … 適応則（パラメータ更新）

とする。リアプノフ関数  V = ½ sᵀ M s + ½ Γ⁻¹ (m̂ - m)²  を微分すると
（Ṁ - 2C の歪対称性を使って） V̇ = -sᵀ Kd s ≤ 0 となり、追従誤差 → 0 が保証される。

学習ポイント:
  - /control_lab/info の m_hat が、Gazebo の負荷質量（既定 5 kg）に近づいていく様子を観察
  - Γ（適応ゲイン）が大きいと速く収束するが振動的になる
  - 「誤差 → 0」は保証されるが「m̂ → 真値」は一般には保証されない！
    パラメータ収束には回帰ベクトル Y が十分に「励振的 (persistently exciting)」である必要がある。
    ここでは未知数が質量 1 つで、重力項 Jpᵀ[0,0,g] が静止中も 0 でないので、hold でも収束する。
    m_hat_init を大きく外した値にしたり、gamma を変えたりして収束の様子を観察しよう
"""
import numpy as np

from .base import Controller, per_joint


class Adaptive(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.lam = per_joint(cfg.get('lambda', 5.0))
        self.kd = per_joint(cfg.get('kd', 30.0))
        self.gamma = float(cfg.get('gamma', 0.5))
        self.offset = np.asarray(cfg.get('payload_offset', [0.0, 0.0, 0.1]))
        self.m_init = float(cfg.get('m_hat_init', 0.0))
        self.m_min, self.m_max = cfg.get('m_hat_bounds', [0.0, 20.0])
        self.reset()

    def reset(self):
        self.m_hat = self.m_init
        self.t_prev = None

    def regressor(self, q, qd, qd_r, qdd_r, h=1e-6):
        Jp = self.model.point_jacobian(q, self.offset)
        Jp_dot = (self.model.point_jacobian(q + qd * h, self.offset) - Jp) / h
        a = Jp @ qdd_r + Jp_dot @ qd_r + np.array([0.0, 0.0, 9.81])
        return Jp.T @ a  # (6,)

    def compute(self, t, q, qd, ref):
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        self.t_prev = t

        e = q - ref.q
        ed = qd - ref.qd
        s = ed + self.lam * e
        qd_r = ref.qd - self.lam * e
        qdd_r = ref.qdd - self.lam * ed

        Y = self.regressor(q, qd, qd_r, qdd_r)
        tau_model = (self.model.mass_matrix(q) @ qdd_r
                     + self.model.coriolis_times(q, qd, qd_r)
                     + self.model.gravity(q))
        tau = tau_model + Y * self.m_hat - self.kd * s

        # 適応則（射影付き: 推定値が物理的にありえない範囲に出ないようにする）
        self.m_hat += -self.gamma * float(Y @ s) * dt
        self.m_hat = float(np.clip(self.m_hat, self.m_min, self.m_max))
        return tau, {'s': s, 'm_hat': np.array([self.m_hat])}
