"""Gazebo 上のアームの手先を「押す」: 外力に対する応答（インピーダンス制御など）を観察する.

  ros2 run ur10e_control_lab push --force 0 0 -50 --duration 2
  ros2 run ur10e_control_lab push --force 30 0 0 --duration 0.2     # 短い衝撃

力はワールド座標系 [N] で、wrist_3_link（手先・負荷を含む）に加わる。
Gazebo の ApplyLinkWrench システム（worlds/control_lab.sdf）を使っている。
"""
import argparse
import time

import rclpy
from ros_gz_interfaces.msg import Entity, EntityWrench

WORLD = 'control_lab'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--force', nargs=3, type=float, default=[0.0, 0.0, -50.0], metavar=('FX', 'FY', 'FZ'))
    ap.add_argument('--torque', nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=('TX', 'TY', 'TZ'))
    ap.add_argument('--duration', type=float, default=2.0, help='押す時間 [s]（壁時計）')
    ap.add_argument('--link', default='ur10e::wrist_3_link')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('push')
    pub = node.create_publisher(EntityWrench, f'/world/{WORLD}/wrench', 10)
    msg = EntityWrench()
    msg.entity.name = args.link
    msg.entity.type = Entity.LINK
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = args.force
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = args.torque

    # wrench トピックは「1 ステップだけ」力を加えるので、押している間 publish し続ける
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        time.sleep(0.05)
    print(f'{args.link} に F={args.force} N を {args.duration} s 加えます')
    t_end = time.time() + args.duration
    while time.time() < t_end:
        pub.publish(msg)
        time.sleep(0.001)
    print('終了')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
