"""制御器の一覧。lab_params.yaml の controllers: のキーはここの名前に対応する."""
from .adaptive import Adaptive
from .computed_torque import ComputedTorque
from .impedance import Impedance
from .lqr import LQR
from .mpc import MPC
from .pd_gravity import PDGravity
from .pid import PID
from .sliding_mode import SlidingMode
from .template import Template

CONTROLLERS = {
    'template': Template,
    'pid': PID,
    'pd_gravity': PDGravity,
    'computed_torque': ComputedTorque,
    'lqr': LQR,
    'sliding_mode': SlidingMode,
    'adaptive': Adaptive,
    'impedance': Impedance,
    'mpc': MPC,
}


def make_controller(name, model, cfg_all):
    """name に対応する制御器を作る。cfg_all[name] の 'type' で実装クラスを指定することもできる
    （例: 同じ PID を別ゲインで 'pid_soft' として登録）."""
    if name not in cfg_all:
        raise ValueError(f'unknown controller "{name}". choose from {list(cfg_all)}')
    cfg = dict(cfg_all[name] or {})
    cls_name = cfg.pop('type', name)
    # 制御器ごとにモデルの負荷質量を上書きできる（ロバスト・適応制御は「負荷を知らない」モデルを使う）
    if 'model_payload_mass' in cfg:
        model = _model_with_payload(float(cfg.pop('model_payload_mass')))
    return CONTROLLERS[cls_name](model, cfg)


_models = {}


def _model_with_payload(mass):
    from ..robot_model import RobotModel
    if mass not in _models:
        _models[mass] = RobotModel(payload_mass=mass)
    return _models[mass]
