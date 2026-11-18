"""オフラインシミュレータ: Gazebo なしで、同じ制御コードを Python だけで高速に試す.

プラント（制御対象）は  M(q) q̈ + C(q,q̇) q̇ + g(q) = τ + Jᵀ F_ext  を数値積分するだけの
単純なモデル（接触・摩擦なし）。Gazebo の起動を待たずにゲイン調整や比較ができる。

例:
  ros2 run ur10e_control_lab offline_sim -c pid -r step
  ros2 run ur10e_control_lab offline_sim -c computed_torque -r sine --duration 10
  ros2 run ur10e_control_lab offline_sim -c pid computed_torque mpc -r step   # 比較
  ros2 run ur10e_control_lab offline_sim -c impedance -r hold --push 1.0,0.5,0,0,-100
  ros2 run ur10e_control_lab offline_sim -c pd_gravity --set model_payload_mass=0.0
"""
import argparse
import os
import time

import numpy as np
import yaml

from .control_loop import ControlLoop, load_params, log_header, log_row
from .robot_model import RobotModel


def set_nested(params, assignment):
    """'controllers.pid.kp=[1,2,3,4,5,6]' のような指定で params を書き換える."""
    key, value = assignment.split('=', 1)
    d = params
    parts = key.split('.')
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = yaml.safe_load(value)


def simulate(params, controller, reference, duration, plant_payload, estimator=None,
             dt_sim=0.001, control_period=0.002, push=None, verbose=True):
    loop = ControlLoop(params, controller, reference, estimator)
    plant = RobotModel(payload_mass=plant_payload)
    q = loop.home.copy()
    qd = np.zeros_like(q)
    decim = max(1, int(round(control_period / dt_sim)))
    rows = []
    tau = np.zeros_like(q)
    n_steps = int(round(duration / dt_sim))
    wall0 = time.time()
    for k in range(n_steps + 1):
        t = k * dt_sim
        if k % decim == 0:
            tau, log = loop.step(t, q, qd)
            rows.append(log_row(loop.model, t, q, qd, tau, log))
        # 外力（手先を押す）: τ_ext = Jᵀ F
        tau_ext = np.zeros_like(q)
        if push is not None:
            t0, T, fx, fy, fz = push
            if t0 <= t < t0 + T:
                tau_ext = plant.jacobian(q)[:3].T @ np.array([fx, fy, fz])
        qdd = plant.forward_dynamics(q, qd, tau + tau_ext)
        qd = qd + qdd * dt_sim          # 半陰的オイラー法
        q = q + qd * dt_sim
        if not np.all(np.isfinite(q)) or np.max(np.abs(qd)) > 100:
            if verbose:
                print(f'  !! t={t:.3f}s で発散しました')
            break
    if verbose:
        print(f'  {controller:>24s} / {reference:<8s}  sim {duration:.1f}s  '
              f'(計算 {time.time() - wall0:.1f}s)')
    return np.array(rows)


def summarize(name, data):
    h = log_header()
    idx = {c: i for i, c in enumerate(h)}
    q = data[:, idx['q0']:idx['q0'] + 6]
    qr = data[:, idx['q_ref0']:idx['q_ref0'] + 6]
    tau = data[:, idx['tau0']:idx['tau0'] + 6]
    e = qr - q
    rms = np.sqrt(np.mean(e**2, axis=0))
    final = np.abs(e[-1])
    x = data[:, idx['x']:idx['x'] + 3]
    xr = data[:, idx['x_ref']:idx['x_ref'] + 3]
    ee = np.linalg.norm(xr - x, axis=1)
    sat = data[:, idx['saturated']].mean() * 100
    rough = np.mean(np.abs(np.diff(tau, axis=0)), axis=0)  # トルクのガタつき（1 周期あたりの変化量）
    print(f'  {name:>24s}: 関節誤差RMS[deg]={np.array2string(np.degrees(rms), precision=2)}'
          f'  最終誤差max={np.degrees(final.max()):.3f}deg'
          f'  手先誤差RMS={np.sqrt(np.mean(ee**2)) * 1000:.1f}mm'
          f'  |τ|max={np.array2string(np.abs(tau).max(axis=0), precision=0)}'
          f'  |Δτ|平均={np.array2string(rough, precision=2)}  飽和={sat:.1f}%')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-c', '--controller', nargs='+', default=['pd_gravity'])
    ap.add_argument('-r', '--reference', default='hold')
    ap.add_argument('-e', '--estimator', default=None, help='none / diff / kf')
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--payload', type=float, default=5.0, help='プラント（実機役）の負荷質量 [kg]')
    ap.add_argument('--params', default=None, help='lab_params.yaml のパス')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='パラメータ上書き (例: controllers.pid.ki=0.0)')
    ap.add_argument('--push', default=None,
                    help='手先に外力: "開始時刻,継続時間,Fx,Fy,Fz" [s,s,N,N,N]')
    ap.add_argument('--out', default=os.path.expanduser('~/ur10e_lab_logs/offline'))
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    params = load_params(args.params)
    for s in args.set:
        set_nested(params, s)
    # 制御器のモデルの負荷は、指定がなければプラントと同じ（= モデルが正確）
    params.setdefault('model_payload_mass', args.payload)
    push = [float(v) for v in args.push.split(',')] if args.push else None

    os.makedirs(args.out, exist_ok=True)
    files = []
    results = {}
    print('シミュレーション中...')
    for c in args.controller:
        data = simulate(params, c, args.reference, args.duration, args.payload,
                        args.estimator, push=push)
        path = os.path.join(args.out, f'{c}_{args.reference}.csv')
        np.savetxt(path, data, delimiter=',', header=','.join(log_header()), comments='')
        files.append(path)
        results[c] = data
    print('結果:')
    for c, data in results.items():
        summarize(c, data)
    print('CSV:', *files, sep='\n  ')
    if not args.no_plot:
        from .plot_log import plot_files
        png = os.path.join(args.out, f'{"_vs_".join(args.controller)}_{args.reference}.png')
        plot_files(files, png, show=True)


if __name__ == '__main__':
    main()
