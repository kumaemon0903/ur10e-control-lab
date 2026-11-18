"""UR10e の運動学・動力学モデル（PyKDL を使用）.

ロボットアームの運動方程式:

    M(q) q̈ + C(q, q̇) q̇ + g(q) = τ

    M(q)      : 慣性行列 (6x6, 対称・正定値)
    C(q, q̇)q̇ : コリオリ力・遠心力 (6,)
    g(q)      : 重力トルク (6,)
    τ         : 関節トルク (6,)

制御器はこのクラスを通して「モデル」を参照する。
モデルは URDF から作るため、payload_mass を変えれば「負荷を知っている／知らない」制御器を作れる。
"""
import os
import subprocess
from functools import lru_cache

import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
NJ = len(JOINT_NAMES)
GRAVITY = 9.81


@lru_cache(maxsize=8)
def generate_urdf(payload_mass: float = 0.0) -> str:
    """xacro から URDF 文字列を生成する."""
    from ament_index_python.packages import get_package_share_directory
    xacro_file = os.path.join(get_package_share_directory('ur10e_control_lab'),
                              'urdf', 'ur10e_lab.urdf.xacro')
    return subprocess.check_output(
        ['xacro', xacro_file, f'payload_mass:={payload_mass}'], text=True)


# --------------------------------------------------------------------------
# URDF → KDL Chain 変換（kdl_parser_py と同じ処理を最小限で実装）
# --------------------------------------------------------------------------
def _kdl_frame(origin):
    xyz = origin.xyz if origin is not None and origin.xyz is not None else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin is not None and origin.rpy is not None else [0.0, 0.0, 0.0]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _kdl_inertia(inertial):
    if inertial is None:
        return kdl.RigidBodyInertia()
    origin = _kdl_frame(inertial.origin)
    i = inertial.inertia
    rbi = kdl.RigidBodyInertia(
        inertial.mass, origin.p,
        kdl.RotationalInertia(i.ixx, i.iyy, i.izz, i.ixy, i.ixz, i.iyz))
    return origin.M * rbi


def _kdl_joint(joint):
    origin = _kdl_frame(joint.origin)
    if joint.type == 'fixed':
        return kdl.Joint(joint.name, kdl.Joint.Fixed)
    axis = kdl.Vector(*joint.axis)
    if joint.type in ('revolute', 'continuous'):
        return kdl.Joint(joint.name, origin.p, origin.M * axis, kdl.Joint.RotAxis)
    if joint.type == 'prismatic':
        return kdl.Joint(joint.name, origin.p, origin.M * axis, kdl.Joint.TransAxis)
    raise ValueError(f'unsupported joint type: {joint.type}')


def build_chain(urdf_xml: str, root: str, tip: str) -> kdl.Chain:
    robot = URDF.from_xml_string(urdf_xml)
    # tip から root へ親をたどり、root → tip の順に並べる
    joints = []
    link = tip
    while link != root:
        if link not in robot.parent_map:
            raise ValueError(f'{root} から {tip} への経路が見つかりません')
        joint_name, parent = robot.parent_map[link]
        joints.append(robot.joint_map[joint_name])
        link = parent
    joints.reverse()

    # tip 以下に付いている固定リンク（payload 等）の慣性は tip に合成する
    def subtree_inertia(link_name, frame):
        total = frame * _kdl_inertia(robot.link_map[link_name].inertial)
        for joint_name, child in robot.child_map.get(link_name, []):
            j = robot.joint_map[joint_name]
            if j.type == 'fixed':
                total = total + subtree_inertia(child, frame * _kdl_frame(j.origin))
        return total

    chain = kdl.Chain()
    for j in joints:
        child = j.child
        if child == tip:
            inertia = subtree_inertia(child, kdl.Frame.Identity())
        else:
            inertia = _kdl_inertia(robot.link_map[child].inertial)
        chain.addSegment(kdl.Segment(child, _kdl_joint(j), _kdl_frame(j.origin), inertia))
    return chain


# --------------------------------------------------------------------------
# numpy <-> KDL 変換ヘルパ
# --------------------------------------------------------------------------
def _jnt(v):
    a = kdl.JntArray(NJ)
    for i in range(NJ):
        a[i] = float(v[i])
    return a


def _np_vec(a):
    return np.array([a[i] for i in range(a.rows())])


def _np_mat(m, rows, cols):
    return np.array([[m[i, j] for j in range(cols)] for i in range(rows)])


class RobotModel:
    """UR10e の動力学モデル.

    Args:
        payload_mass: モデルに含める手先負荷 [kg]。制御器にとっての「既知の負荷」。
        root, tip:    運動学チェーンの根元・先端リンク
    """

    def __init__(self, payload_mass: float = 0.0, root: str = 'base_link', tip: str = 'tool0',
                 urdf_xml: str | None = None):
        self.payload_mass = payload_mass
        urdf_xml = urdf_xml or generate_urdf(float(payload_mass))
        self.chain = build_chain(urdf_xml, root, tip)
        assert self.chain.getNrOfJoints() == NJ
        self._dyn = kdl.ChainDynParam(self.chain, kdl.Vector(0.0, 0.0, -GRAVITY))
        self._fk = kdl.ChainFkSolverPos_recursive(self.chain)
        self._jac = kdl.ChainJntToJacSolver(self.chain)
        self._jacdot = kdl.ChainJntToJacDotSolver(self.chain)
        self._ik = kdl.ChainIkSolverPos_LMA(self.chain)

        robot = URDF.from_xml_string(urdf_xml)
        self.effort_limit = np.array([robot.joint_map[n].limit.effort for n in JOINT_NAMES])
        self.velocity_limit = np.array([robot.joint_map[n].limit.velocity for n in JOINT_NAMES])

    # ---------------- 動力学 ----------------
    def mass_matrix(self, q):
        """慣性行列 M(q)."""
        m = kdl.JntSpaceInertiaMatrix(NJ)
        self._dyn.JntToMass(_jnt(q), m)
        return _np_mat(m, NJ, NJ)

    def coriolis(self, q, qd):
        """コリオリ・遠心力ベクトル C(q, q̇) q̇."""
        c = kdl.JntArray(NJ)
        self._dyn.JntToCoriolis(_jnt(q), _jnt(qd), c)
        return _np_vec(c)

    def coriolis_times(self, q, qd, v):
        """C(q, q̇) v を返す（v ≠ q̇ のとき用。スライディングモード・適応制御で使う）.

        C(q,q̇)q̇ は q̇ について 2 次形式 h(q̇,q̇)（h は対称双線形）であり、
        Christoffel 記号で定義した C については C(q,q̇)v = h(q̇, v)。
        よって偏極恒等式 h(a,b) = [h(a+b,a+b) - h(a-b,a-b)] / 4 で計算できる。
        """
        return (self.coriolis(q, qd + v) - self.coriolis(q, qd - v)) / 4.0

    def gravity(self, q):
        """重力トルク g(q)."""
        g = kdl.JntArray(NJ)
        self._dyn.JntToGravity(_jnt(q), g)
        return _np_vec(g)

    def inverse_dynamics(self, q, qd, qdd):
        """逆動力学 τ = M(q) q̈ + C(q,q̇)q̇ + g(q)."""
        return self.mass_matrix(q) @ qdd + self.coriolis(q, qd) + self.gravity(q)

    def forward_dynamics(self, q, qd, tau):
        """順動力学 q̈ = M(q)^-1 (τ - C(q,q̇)q̇ - g(q))（オフラインシミュレータで使う）."""
        return np.linalg.solve(self.mass_matrix(q), tau - self.coriolis(q, qd) - self.gravity(q))

    def gravity_jacobian(self, q, eps=1e-6):
        """∂g/∂q（数値微分）。LQR で平衡点まわりの線形化に使う."""
        g0 = self.gravity(q)
        G = np.zeros((NJ, NJ))
        for i in range(NJ):
            dq = np.zeros(NJ)
            dq[i] = eps
            G[:, i] = (self.gravity(q + dq) - g0) / eps
        return G

    # ---------------- 運動学 ----------------
    def fk(self, q):
        """順運動学: 手先 (tool0) の位置 p (3,) と回転行列 R (3x3)."""
        f = kdl.Frame()
        self._fk.JntToCart(_jnt(q), f)
        p = np.array([f.p[0], f.p[1], f.p[2]])
        R = np.array([[f.M[i, j] for j in range(3)] for i in range(3)])
        return p, R

    def jacobian(self, q):
        """幾何ヤコビアン J (6x6)。上 3 行が並進速度、下 3 行が角速度（base_link 座標系）."""
        j = kdl.Jacobian(NJ)
        self._jac.JntToJac(_jnt(q), j)
        return _np_mat(j, 6, NJ)

    def jacobian_dot_qd(self, q, qd):
        """J̇(q, q̇) q̇ (6,)。タスク空間の加速度 ẍ = J q̈ + J̇ q̇ の第 2 項."""
        t = kdl.Twist()
        self._jacdot.JntToJacDot(kdl.JntArrayVel(_jnt(q), _jnt(qd)), t)
        return np.array([t.vel[0], t.vel[1], t.vel[2], t.rot[0], t.rot[1], t.rot[2]])

    def point_jacobian(self, q, offset_tool):
        """tool0 座標系で offset_tool だけ離れた点の並進ヤコビアン (3x6).

        v_p = v_tool + ω × r  （r は base 座標系で表した tool0 → 点 のベクトル）
        """
        J = self.jacobian(q)
        _, R = self.fk(q)
        r = R @ np.asarray(offset_tool)
        r_skew = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
        return J[:3] - r_skew @ J[3:]

    def ik(self, p, R, q_init):
        """逆運動学（数値解, Levenberg-Marquardt）。q_init の近くの解を返す."""
        target = kdl.Frame(kdl.Rotation(*R.flatten().tolist()), kdl.Vector(*p))
        q_out = kdl.JntArray(NJ)
        ret = self._ik.CartToJnt(_jnt(q_init), target, q_out)
        if ret < 0:
            raise RuntimeError(f'IK failed (code {ret})')
        return _np_vec(q_out)


def orientation_error(R_ref, R):
    """姿勢誤差 e_o (3,): R を R_ref に合わせる回転の回転ベクトル（base 座標系）."""
    R_err = R_ref @ R.T
    cos_theta = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    v = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
    if theta < 1e-9:
        return 0.5 * v
    return theta / (2.0 * np.sin(theta)) * v
