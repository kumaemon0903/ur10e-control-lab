"""動作確認テスト:  colcon test --packages-select ur10e_control_lab --event-handlers console_direct+

モデルの性質（教科書に出てくる性質が本当に成り立つか）と、各制御器がオフラインで安定に動くかを確認する。
"""
import numpy as np
import PyKDL as kdl
import pytest

from ur10e_control_lab.control_loop import load_params, log_header
from ur10e_control_lab.offline_sim import simulate
from ur10e_control_lab.robot_model import RobotModel, _jnt, _np_vec

COL = {name: i for i, name in enumerate(log_header())}
Q = np.array([0.3, -1.2, 1.4, -1.0, -1.5, 0.4])
QD = np.array([0.3, -0.2, 0.5, 0.1, -0.4, 0.2])


@pytest.fixture(scope='module')
def model():
    return RobotModel(payload_mass=0.0)


def test_inverse_dynamics_matches_kdl_rne(model):
    """M q̈ + C q̇ + g が KDL の逆動力学 (Newton-Euler) と一致する."""
    qdd = np.array([1.0, 2.0, -1.0, 0.5, 0.3, -0.2])
    rne = kdl.ChainIdSolver_RNE(model.chain, kdl.Vector(0, 0, -9.81))
    tau = kdl.JntArray(6)
    wrenches = [kdl.Wrench() for _ in range(model.chain.getNrOfSegments())]
    rne.CartToJnt(_jnt(Q), _jnt(QD), _jnt(qdd), wrenches, tau)
    assert np.allclose(_np_vec(tau), model.inverse_dynamics(Q, QD, qdd), atol=1e-9)


def test_mass_matrix_symmetric_positive_definite(model):
    M = model.mass_matrix(Q)
    assert np.allclose(M, M.T)
    assert np.all(np.linalg.eigvalsh(M) > 0)


def test_skew_symmetry(model):
    """Ṁ - 2C が歪対称（受動性に基づく制御の安定性証明の鍵）."""
    h = 1e-6
    Mdot = (model.mass_matrix(Q + QD * h) - model.mass_matrix(Q - QD * h)) / (2 * h)
    C = np.column_stack([model.coriolis_times(Q, QD, e) for e in np.eye(6)])
    N = Mdot - 2 * C
    assert np.allclose(N, -N.T, atol=1e-6)


def test_payload_regressor(model):
    """負荷の重力トルクが Jpᵀ [0, 0, m g] と一致する（適応制御の回帰ベクトル）."""
    m5 = RobotModel(payload_mass=5.0)
    Jp = model.point_jacobian(Q, [0, 0, 0.1])
    assert np.allclose(m5.gravity(Q) - model.gravity(Q), Jp.T @ [0, 0, 5 * 9.81], atol=1e-9)


@pytest.mark.parametrize('controller', [
    'pid', 'pd_gravity', 'computed_torque', 'lqr', 'lqr_linearized', 'sliding_mode',
    'adaptive', 'impedance', 'mpc', 'my_controller'])
def test_controllers_track_sine(controller):
    params = load_params()
    params['model_payload_mass'] = 5.0
    data = simulate(params, controller, 'sine', duration=4.0, plant_payload=5.0, verbose=False)
    assert data.shape[0] > 1900, '途中で発散した'
    if controller == 'impedance':  # タスク空間の制御器は手先位置の誤差で評価する
        x = data[:, COL['x']:COL['x'] + 3]
        x_ref = data[:, COL['x_ref']:COL['x_ref'] + 3]
        rms_mm = 1000 * np.sqrt(np.mean(np.sum((x_ref - x) ** 2, axis=1)))
        assert rms_mm < 20.0, rms_mm
        return
    q = data[:, COL['q0']:COL['q0'] + 6]
    q_ref = data[:, COL['q_ref0']:COL['q_ref0'] + 6]
    rms = np.degrees(np.sqrt(np.mean((q_ref - q) ** 2, axis=0)))
    assert np.all(rms < 10.0), rms


def test_adaptive_estimates_payload():
    params = load_params()
    data = simulate(params, 'adaptive', 'sine', duration=5.0, plant_payload=5.0, verbose=False)
    m_hat = data[-1, COL['m_hat']]
    assert abs(m_hat - 5.0) < 0.5, m_hat


def test_impedance_stiffness():
    """静的な外力 F に対して変位 ≈ F / K（インピーダンス制御の定義そのもの）."""
    params = load_params()
    k = params['controllers']['impedance']['stiffness_translational'][2]
    data = simulate(params, 'impedance', 'hold', duration=3.0, plant_payload=5.0,
                    push=[0.5, 2.5, 0.0, 0.0, -30.0], verbose=False)
    t = data[:, 0]
    i = np.searchsorted(t, 2.9)
    dz = data[i, COL['z']] - data[i, COL['z_ref']]
    assert dz == pytest.approx(-30.0 / k, rel=0.05)
