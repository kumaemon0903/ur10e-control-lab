"""[演習] 自分の制御則を書くためのテンプレート.

このファイルを編集して、自分で考えた制御則を試そう。初期状態では「PD + 重力補償」と同じ動作をする。

  1. compute() の中身を書き換える
  2. lab_params.yaml の controllers.my_controller にゲインを書く
  3. オフラインで確認:  ros2 run ur10e_control_lab offline_sim -c my_controller pd_gravity -r sine
  4. Gazebo で確認:     ros2 param set /control_lab controller my_controller
     （--symlink-install でビルドしていれば、Python の変更は lab_node の再起動で反映）

使えるモデル関数（self.model, robot_model.RobotModel）:
  M = self.model.mass_matrix(q)          慣性行列 (6x6)
  c = self.model.coriolis(q, qd)         C(q,q̇)q̇ (6,)
  g = self.model.gravity(q)              重力トルク (6,)
  p, R = self.model.fk(q)                手先位置 (3,), 姿勢 (3x3)
  J = self.model.jacobian(q)             ヤコビアン (6x6)
目標値（ref）:
  ref.q, ref.qd, ref.qdd                 関節空間の目標
  ref.x, ref.xd, ref.xdd, ref.R          タスク空間の目標
"""
import numpy as np

from .base import Controller, per_joint


class Template(Controller):

    def __init__(self, model, cfg):
        super().__init__(model, cfg)
        self.kp = per_joint(cfg.get('kp', 100.0))
        self.kd = per_joint(cfg.get('kd', 10.0))

    def reset(self):
        pass  # 積分器などの内部状態があればここで初期化

    def compute(self, t, q, qd, ref):
        e = ref.q - q
        ed = ref.qd - qd
        # ===== ここを書き換える =====
        tau = self.kp * e + self.kd * ed + self.model.gravity(q)
        # ============================
        # 2 つ目の戻り値の dict は /control_lab/info/<キー> に publish される（可視化用）
        return tau, {'e_norm': np.array([np.linalg.norm(e)])}
