"""Formation experiment: virtual leader, four controllers and the panel.

Brings up only this experiment. MAVROS, odom_relay and command_bridge run as they always do
(docs/GAZEBO_RUNBOOK.md or docs/FIELD_RUNBOOK.md):

    ros2 launch swarm_formation formation.launch.py                     # field
    ros2 launch swarm_formation formation.launch.py use_sim_time:=true  # SITL + Gazebo
    ros2 launch swarm_formation formation.launch.py record:=true        # also rosbag

In SITL use use_sim_time:=true: the sim runs slower than real time, so the leader and the
controllers must run on Gazebo's clock (relayed by gz_clock) or the drones trail the leader.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NAMESPACES = ['drone1', 'drone2', 'drone3', 'drone4']

# Desired, actual, error, command (ours and the bridge's), state, and the leader.
RECORD = ['/formation/leader', '/clock'] + [
    f'/{ns}/{topic}' for ns in NAMESPACES
    for topic in ('odom', 'formation_target', 'formation_error', 'cmd_vel',
                  'mavros/setpoint_velocity/cmd_vel_unstamped', 'mavros/state',
                  'formation_controller/active')]


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    controllers = [
        Node(package='swarm_formation', executable='formation_controller',
             name='formation_controller', namespace=ns, output='screen',
             parameters=[params, sim_time])
        for ns in NAMESPACES]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('panel', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('swarm_formation'), 'config', 'formation.yaml'])),

        Node(package='swarm_formation', executable='gz_clock', name='gz_clock',
             condition=IfCondition(LaunchConfiguration('use_sim_time'))),
        Node(package='swarm_formation', executable='formation_leader',
             name='formation_leader', output='screen', parameters=[params, sim_time]),
        *controllers,
        # The panel stays on wall time: it only streams button presses.
        Node(package='swarm_formation', executable='formation_panel', name='formation_panel',
             parameters=[params], condition=IfCondition(LaunchConfiguration('panel'))),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', *RECORD], output='screen',
                       condition=IfCondition(LaunchConfiguration('record'))),
    ])
