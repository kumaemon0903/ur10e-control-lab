"""目標軌道（リファレンス）生成.

すべての軌道は時刻 t に対して Reference（関節空間 q, q̇, q̈ とタスク空間 x, ẋ, ẍ, R）を返す。
- 関節空間の制御器は q_ref, qd_ref, qdd_ref を使う
- タスク空間の制御器（インピーダンス制御）は x_ref, xd_ref, xdd_ref, R_ref を使う
"""
from dataclasses import dataclass

import numpy as np

from .robot_model import NJ, RobotModel


@dataclass
class Reference:
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    x: np.ndarray        # 手先位置 (3,)
    xd: np.ndarray       # 手先速度 (3,)
    xdd: np.ndarray      # 手先加速度 (3,)
    R: np.ndarray        # 手先姿勢 (3x3)


class Trajectory:
    """関節空間で定義された軌道の基底クラス。タスク空間の値は順運動学で求める."""

    def __init__(self, model: RobotModel, home: np.ndarray, cfg: dict):
        self.model = model
        self.home = np.asarray(home, dtype=float)
        self.cfg = cfg

    def joint(self, t):
        raise NotImplementedError

    def preview(self, t, dt, n):
        """未来 n ステップ分の関節目標（MPC 用）: 形状 (n, NJ) の q, qd."""
        qs, qds = [], []
        for k in range(1, n + 1):
            q, qd, _ = self.joint(t + k * dt)
            qs.append(q)
            qds.append(qd)
        return np.array(qs), np.array(qds)

    def __call__(self, t) -> Reference:
        q, qd, qdd = self.joint(t)
        x, R = self.model.fk(q)
        J = self.model.jacobian(q)
        xd = J[:3] @ qd
        xdd = J[:3] @ qdd + self.model.jacobian_dot_qd(q, qd)[:3]
        return Reference(q, qd, qdd, x, xd, xdd, R)


class Hold(Trajectory):
    """初期姿勢で静止（レギュレーション問題）."""

    def joint(self, t):
        z = np.zeros(NJ)
        return self.home.copy(), z, z.copy()


class Step(Trajectory):
    """ステップ応答: period 秒ごとに home と home+amplitude を往復する.

    速度・加速度の目標は 0（不連続な目標 → オーバーシュートや飽和が見やすい）。
    """

    def joint(self, t):
        amp = np.asarray(self.cfg.get('amplitude', [0.3] * NJ))
        period = self.cfg.get('period', 4.0)
        on = int(t // period) % 2 == 1
        z = np.zeros(NJ)
        return self.home + (amp if on else 0.0), z, z.copy()


class Quintic(Trajectory):
    """5 次多項式（最小躍度）による点間移動: home ↔ home+amplitude を move_time 秒で移動し、dwell 秒停止."""

    def joint(self, t):
        amp = np.asarray(self.cfg.get('amplitude', [0.5] * NJ))
        T = self.cfg.get('move_time', 2.0)
        dwell = self.cfg.get('dwell', 1.0)
        cycle = T + dwell
        k = int(t // cycle)
        tau = min((t - k * cycle) / T, 1.0)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        sd = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T if tau < 1.0 else 0.0
        sdd = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2 if tau < 1.0 else 0.0
        start, goal = (self.home, self.home + amp) if k % 2 == 0 else (self.home + amp, self.home)
        d = goal - start
        return start + s * d, sd * d, sdd * d


class Sine(Trajectory):
    """各関節の正弦波: q = home + A sin(2π f t)."""

    def joint(self, t):
        A = np.asarray(self.cfg.get('amplitude', [0.3] * NJ))
        f = np.asarray(self.cfg.get('frequency', [0.25] * NJ))
        w = 2 * np.pi * f
        return (self.home + A * np.sin(w * t), A * w * np.cos(w * t), -A * w**2 * np.sin(w * t))


class Circle(Trajectory):
    """手先で円を描く（姿勢は一定）。関節目標は数値 IK と微分運動学で求める.

        q̇_ref  = J⁻¹ [ẋ; 0]
        q̈_ref  = J⁻¹ ([ẍ; 0] - J̇ q̇_ref)
    """

    def __init__(self, model, home, cfg):
        super().__init__(model, home, cfg)
        self.center, self.R0 = model.fk(self.home)
        self._q_prev = self.home.copy()
        self._t_prev = None

    def task(self, t):
        r = self.cfg.get('radius', 0.15)
        w = 2 * np.pi * self.cfg.get('frequency', 0.2)
        plane = self.cfg.get('plane', 'yz')
        # ゆっくり立ち上げる（t=0 で速度が不連続にならないよう、半径を ramp 秒かけて広げる）
        ramp = self.cfg.get('ramp', 2.0)
        a = min(t / ramp, 1.0) if ramp > 0 else 1.0
        u, v = r * np.cos(w * t) - r, r * np.sin(w * t)   # t=0 で中心(=home)を通る
        ud, vd = -r * w * np.sin(w * t), r * w * np.cos(w * t)
        udd, vdd = -r * w**2 * np.cos(w * t), -r * w**2 * np.sin(w * t)
        ad = 1.0 / ramp if (ramp > 0 and t < ramp) else 0.0
        # x = a(t) * (u, v) の積の微分
        pu, pv = a * u, a * v
        pud, pvd = a * ud + ad * u, a * vd + ad * v
        pudd, pvdd = a * udd + 2 * ad * ud, a * vdd + 2 * ad * vd
        idx = {'xy': (0, 1), 'yz': (1, 2), 'xz': (0, 2)}[plane]
        x, xd, xdd = self.center.copy(), np.zeros(3), np.zeros(3)
        x[idx[0]] += pu
        x[idx[1]] += pv
        xd[idx[0]], xd[idx[1]] = pud, pvd
        xdd[idx[0]], xdd[idx[1]] = pudd, pvdd
        return x, xd, xdd

    def joint(self, t):
        x, xd, xdd = self.task(t)
        # 時刻が戻った（preview 等）場合も破綻しないよう、IK の初期値は直前の解
        q = self.model.ik(x, self.R0, self._q_prev)
        if self._t_prev is None or t >= self._t_prev:
            self._q_prev, self._t_prev = q, t
        J = self.model.jacobian(q)
        qd = np.linalg.solve(J, np.concatenate([xd, np.zeros(3)]))
        qdd = np.linalg.solve(J, np.concatenate([xdd, np.zeros(3)]) - self.model.jacobian_dot_qd(q, qd))
        return q, qd, qdd

    def __call__(self, t):
        ref = super().__call__(t)
        ref.x, ref.xd, ref.xdd = self.task(t)
        ref.R = self.R0
        return ref


TRAJECTORIES = {
    'hold': Hold,
    'step': Step,
    'quintic': Quintic,
    'sine': Sine,
    'circle': Circle,
}


def make_trajectory(name, model, home, cfg_all):
    if name not in TRAJECTORIES:
        raise ValueError(f'unknown reference "{name}". choose from {list(TRAJECTORIES)}')
    return TRAJECTORIES[name](model, home, cfg_all.get(name, {}) or {})
