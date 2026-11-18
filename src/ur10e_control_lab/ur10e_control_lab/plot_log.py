"""ログ CSV（lab_node の record / offline_sim の出力）をグラフにする.

  ros2 run ur10e_control_lab plot_log ~/ur10e_lab_logs/offline/pid_step.csv
  ros2 run ur10e_control_lab plot_log a.csv b.csv c.csv      # 重ねて比較
  ros2 run ur10e_control_lab plot_log a.csv --out fig.png --no-show
"""
import argparse
import os

import numpy as np

JOINT_LABELS = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    return {name: data[:, i] for i, name in enumerate(header)}


def plot_files(paths, out=None, show=True):
    import matplotlib
    if not show or not os.environ.get('DISPLAY'):
        matplotlib.use('Agg')
        show = False
    import matplotlib.pyplot as plt

    logs = [(os.path.splitext(os.path.basename(p))[0], load(p)) for p in paths]
    fig = plt.figure(figsize=(20, 12), constrained_layout=True)
    gs = fig.add_gridspec(4, 6)
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

    for j in range(6):
        ax_q = fig.add_subplot(gs[0, j])
        ax_e = fig.add_subplot(gs[1, j])
        ax_t = fig.add_subplot(gs[2, j])
        for k, (name, d) in enumerate(logs):
            c = colors[k % len(colors)]
            t = d['t'] - d['t'][0]
            if k == 0:
                ax_q.plot(t, np.degrees(d[f'q_ref{j}']), 'k--', lw=1, label='ref')
            ax_q.plot(t, np.degrees(d[f'q{j}']), color=c, lw=1.2, label=name)
            ax_e.plot(t, np.degrees(d[f'q_ref{j}'] - d[f'q{j}']), color=c, lw=1.2)
            ax_t.plot(t, d[f'tau{j}'], color=c, lw=1.0)
        ax_q.set_title(f'J{j} {JOINT_LABELS[j]}', fontsize=10)
        ax_q.set_ylabel('q [deg]' if j == 0 else '')
        ax_e.set_ylabel('error [deg]' if j == 0 else '')
        ax_t.set_ylabel('torque [Nm]' if j == 0 else '')
        ax_t.set_xlabel('t [s]')
        for ax in (ax_q, ax_e, ax_t):
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=8)
        if j == 0:
            ax_q.legend(fontsize=8)

    # 手先誤差・手先軌跡・推定パラメータ
    ax_ee = fig.add_subplot(gs[3, 0:2])
    ax_path = fig.add_subplot(gs[3, 2:4])
    ax_m = fig.add_subplot(gs[3, 4:6])
    for k, (name, d) in enumerate(logs):
        c = colors[k % len(colors)]
        t = d['t'] - d['t'][0]
        err = np.sqrt((d['x_ref'] - d['x'])**2 + (d['y_ref'] - d['y'])**2 + (d['z_ref'] - d['z'])**2)
        ax_ee.plot(t, err * 1000, color=c, label=name)
        if k == 0:
            ax_path.plot(d['y_ref'], d['z_ref'], 'k--', lw=1, label='ref')
        ax_path.plot(d['y'], d['z'], color=c, lw=1.2, label=name)
        if np.any(np.isfinite(d['m_hat'])):
            ax_m.plot(t, d['m_hat'], color=c, label=name)
        if np.any(d['saturated'] > 0):
            ax_ee.fill_between(t, 0, 1, where=d['saturated'] > 0, color=c, alpha=0.08,
                               transform=ax_ee.get_xaxis_transform())
    ax_ee.set_title('end-effector position error (shaded: torque saturated)', fontsize=10)
    ax_ee.set_ylabel('|x_ref - x| [mm]')
    ax_ee.set_xlabel('t [s]')
    ax_path.set_title('end-effector path (y-z plane)', fontsize=10)
    ax_path.set_xlabel('y [m]')
    ax_path.set_ylabel('z [m]')
    ax_path.axis('equal')
    ax_m.set_title('estimated payload m_hat (adaptive)', fontsize=10)
    ax_m.set_ylabel('[kg]')
    ax_m.set_xlabel('t [s]')
    for ax in (ax_ee, ax_path, ax_m):
        ax.grid(alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=8)

    if out:
        fig.savefig(out, dpi=90)
        print('図を保存:', out)
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--out', default=None)
    ap.add_argument('--no-show', action='store_true')
    args = ap.parse_args()
    out = args.out or os.path.splitext(args.csv[0])[0] + '.png'
    plot_files(args.csv, out, show=not args.no_show)


if __name__ == '__main__':
    main()
