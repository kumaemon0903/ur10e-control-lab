"""UR10e 制御学習ラボ: Gazebo + ros2_control(effort) + 制御ノード を一括起動する.

使い方:
  ros2 launch ur10e_control_lab lab.launch.py
  ros2 launch ur10e_control_lab lab.launch.py controller:=computed_torque reference:=sine
  ros2 launch ur10e_control_lab lab.launch.py gui:=false payload_mass:=0.0
  ros2 launch ur10e_control_lab lab.launch.py model_payload_mass:=0.0   # 制御器は負荷を知らない

起動の流れ:
  1. Gazebo を「一時停止」状態で起動し、UR10e をスポーン
  2. lab_node（制御則を計算するノード）を起動。状態が届くまでは初期姿勢の保持トルクを送り続ける
  3. spawner が joint_state_broadcaster / effort_controller を load・configure して有効化待ちになる
  4. 数秒後にシミュレーションを再開 → 最初の更新でコントローラが有効化され、即座に保持トルクが入る
     （一時停止せずに起動すると、トルク 0 の間にアームが重力で倒れてしまうため）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'control_lab'


def generate_launch_description():
    pkg = get_package_share_directory('ur10e_control_lab')
    gui = LaunchConfiguration('gui')
    payload_mass = LaunchConfiguration('payload_mass')
    controller = LaunchConfiguration('controller')
    reference = LaunchConfiguration('reference')
    lab_params = LaunchConfiguration('lab_params')
    start_lab_node = LaunchConfiguration('start_lab_node')

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI を表示するか'),
        DeclareLaunchArgument('payload_mass', default_value='5.0',
                              description='手先負荷の質量 [kg]（制御器のモデルは知らない）'),
        DeclareLaunchArgument('model_payload_mass', default_value=payload_mass,
                              description='制御器のモデルが「知っている」負荷質量 [kg]（既定: 実際の負荷と同じ）'),
        DeclareLaunchArgument('controller', default_value='pd_gravity',
                              description='起動時の制御則 (lab_params.yaml の controllers のキー)'),
        DeclareLaunchArgument('reference', default_value='hold',
                              description='起動時の目標軌道 (hold/step/quintic/sine/circle)'),
        # 空なら lab_node が src/.../config/lab_params.yaml を直接読む（編集 → reload で反映できるように）
        DeclareLaunchArgument('lab_params', default_value='',
                              description='制御ゲイン等の YAML（空 = ソースツリーの config/lab_params.yaml）'),
        DeclareLaunchArgument('start_lab_node', default_value='true',
                              description='false にすると制御ノードを起動しない（自作ノード用）'),
    ]

    controllers_file = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(pkg, 'urdf', 'ur10e_lab.urdf.xacro'),
        ' sim_gazebo:=true force_abs_paths:=true',
        ' controllers_file:=', controllers_file,
        ' payload_mass:=', payload_mass,
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # 一時停止で起動（-r を付けない）。GUI なしの場合は -s（サーバのみ）
    gz_args = PythonExpression([
        "'", os.path.join(pkg, 'worlds', 'control_lab.sdf'), "' + ('' if '", gui,
        "' == 'true' else ' -s') + ' -v 2'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'ur10e', '-world', WORLD_NAME])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{WORLD_NAME}/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
        ])

    # 2 つのコントローラを同時に load → configure → activate する。
    # activate は Gazebo の更新ループ内で処理されるため、一時停止中は「待ち」状態になる。
    spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'effort_controller',
                   '--activate-as-group', '--switch-timeout', '60'])

    lab_node = Node(
        package='ur10e_control_lab', executable='lab_node', name='control_lab', output='screen',
        condition=IfCondition(start_lab_node),
        parameters=[{
            'use_sim_time': True,
            'params_file': lab_params,
            'controller': controller,
            'reference': reference,
            'model_payload_mass': ParameterValue(LaunchConfiguration('model_payload_mass'),
                                                 value_type=float),
        }])

    unpause = ExecuteProcess(
        cmd=['gz', 'service', '-s', f'/world/{WORLD_NAME}/control',
             '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', 'pause: false'],
        output='screen')

    return LaunchDescription(args + [
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        # ロボット生成後: 制御ノードとスポナーを同時に起動し、
        # 両者の準備が整う頃（数秒後）にシミュレーションを再開する
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[lab_node, spawner, TimerAction(period=5.0, actions=[unpause])])),
    ])
