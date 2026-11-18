"""[7] タスク空間インピーダンス制御.

手先と目標の間に「仮想的なばね・ダンパ」をつなぐイメージの制御。
位置を厳密に合わせるのではなく、外力に対して「どれだけ柔らかく応じるか」を設計する。
人との協働、組立・研磨などの接触作業の基本。

手先の誤差（位置 e_p, 姿勢 e_o）から仮想的な力 F を作り、ヤコビアン転置でトルクに変換:

    F = K (x_ref - x) + D (ẋ_ref - ẋ)          （6 次元: 並進 3 + 回転 3）
    τ = Jᵀ F + g(q)                             … use_dynamics: false（剛性制御・コンプライアンス制御）

use_dynamics: true では、慣性・コリオリも補償して目標加速度を追従させる:

    Λ(q) = (J M⁻¹ Jᵀ)⁻¹                         … 手先から見た慣性（タスク空間慣性行列）
    τ = Jᵀ [ Λ (ẍ_ref - J̇ q̇) + F ] + C q̇ + g

このとき外力 F_ext に対する手先の挙動は  Λ ë + D ė + K e = F_ext  （質量-ばね-ダンパ系）。

学習ポイント:
  - stiffness_translational を [x, y, z] で別々に設定 → 「z だけ柔らかい」などが可能
  - `ros2 run ur10e_control_lab push` で手先を押すと、ばねのように変位して戻る
    → K が小さいほど大きく変位する（変位 ≈ F / K）
  - 位置制御（計算トルク法）で同じことをすると、硬く抵抗する
"""
import numpy as np

from ..robot_model import orientation_error
from .base import Controller, per_joint


class Impedance(Controller):
    task_space = True

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        kt = per_joint(cfg.get('stiffness_translational', 1000.0), 3)
        kr = per_joint(cfg.get('stiffness_rotational', 50.0), 3)
        self.zeta = float(cfg.get('damping_ratio', 1.0))
        self.K = np.diag(np.concatenate([kt, kr]))
        self.K_sqrt = np.sqrt(self.K)
        self.use_dynamics = bool(cfg.get('use_dynamics', False))
        self.null_damping = float(cfg.get('joint_damping', 0.5))

    def damping(self, Lam):
        """減衰行列 D.

        1 自由度の質量-ばね-ダンパ m ë + d ė + k e = 0 の臨界減衰は d = 2ζ√(mk)。
        多自由度では手先慣性 Λ(q) が姿勢で変わり、方向によって桁違いに異なる
        （手首まわりの回転は非常に軽い）。そこで D も毎周期 Λ から作る:
            D = ζ (Λ^½ K^½ + K^½ Λ^½)
        固定の D にすると、軽い方向で減衰が強すぎ、離散時間制御が不安定になる。
        """
        w, V = np.linalg.eigh(Lam)
        Lam_sqrt = V @ np.diag(np.sqrt(np.maximum(w, 0.0))) @ V.T
        return self.zeta * (Lam_sqrt @ self.K_sqrt + self.K_sqrt @ Lam_sqrt)

    def compute(self, t, q, qd, ref):
        x, R = self.model.fk(q)
        J = self.model.jacobian(q)
        xdot = J @ qd  # [v; ω]

        M = self.model.mass_matrix(q)
        Lam = np.linalg.inv(J @ np.linalg.solve(M, J.T))  # 手先から見た慣性 Λ = (J M⁻¹ Jᵀ)⁻¹

        e = np.concatenate([ref.x - x, orientation_error(ref.R, R)])
        ed = np.concatenate([ref.xd, np.zeros(3)]) - xdot
        F = self.K @ e + self.damping(Lam) @ ed

        g = self.model.gravity(q)
        if self.use_dynamics:
            a_ref = np.concatenate([ref.xdd, np.zeros(3)]) - self.model.jacobian_dot_qd(q, qd)
            tau = J.T @ (Lam @ a_ref + F) + self.model.coriolis(q, qd) + g
        else:
            tau = J.T @ F + g
        tau -= self.null_damping * qd  # 関節の小さな粘性（数値的な安定化）
        return tau, {'F': F, 'e_task': e}
