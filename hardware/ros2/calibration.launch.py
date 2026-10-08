"""Run only the state broadcaster; no position/trajectory controller is activated.

The upstream hardware initialization still sets operation mode and speed override.
Launch on the physical robot only after checking its model and operating state.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('rbpodo_description'), 'robots',
                              [LaunchConfiguration('model_id'), '.urdf.xacro']]),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' use_fake_hardware:=false cb_simulation:=false fake_sensor_commands:=false',
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Physical RB controller IP'),
        DeclareLaunchArgument('model_id', default_value='rb5_850e', description='Must match the physical robot'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'update_rate': 30, 'joint_state_broadcaster': {
                 'type': 'joint_state_broadcaster/JointStateBroadcaster'}}],
             remappings=[('~/robot_description', '/robot_description'),
                         ('joint_states', 'rbpodo/joint_states')],
             output='screen', on_exit=Shutdown()),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], output='screen'),
    ])
