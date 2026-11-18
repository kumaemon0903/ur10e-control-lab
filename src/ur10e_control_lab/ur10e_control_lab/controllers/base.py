"""制御器の共通インターフェース.

すべての制御器は
    tau, info = controller.compute(t, q, qd, ref)
の形で呼ばれる。

    t   : 時刻 [s]
    q   : 関節角 (6,)   … 推定器を通した値（ノイズ・推定誤差を含みうる）
    qd  : 関節角速度 (6,)
    ref : trajectories.Reference（目標の q, q̇, q̈ / x, ẋ, ẍ, R）
    tau : 関節トルク指令 (6,)
    info: ログ・可視化用の追加信号 dict（例: スライディング変数 s、推定パラメータ）
"""
import numpy as np

from ..robot_model import NJ, RobotModel


def per_joint(value, n=NJ):
    """スカラーなら全関節に同じ値、リストならそのまま numpy 配列に."""
    arr = np.asarray(value, dtype=float)
    return np.full(n, float(arr)) if arr.ndim == 0 else arr


class Controller:
    #: タスク空間の制御器なら True（ログで手先誤差を強調する程度の意味）
    task_space = False

    def __init__(self, model: RobotModel, cfg: dict):
        self.model = model
        self.cfg = cfg or {}

    def reset(self):
        """制御器を切り替えた瞬間に呼ばれる（積分値・推定値の初期化など）."""

    def compute(self, t, q, qd, ref):
        raise NotImplementedError
