"""LiDAR localization only: no planners, controllers, or velocity publishers."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_amcl(context):
    # The standalone launch also works without an installed chassis package.
    import sys
    repo = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo))
    from s1_can.home_pose import home_parameters
    home = LaunchConfiguration('home_pose').perform(context)
    params = (home_parameters(home, LaunchConfiguration('map_path').perform(context))
              if LaunchConfiguration('use_home').perform(context).lower() == 'true' else {})
    if params:
        print('Starting at saved home pose. Place the robot on its marked position and heading.')
    return [Node(package='nav2_amcl', executable='amcl', name='amcl',
                 namespace='localization', output='screen',
                 parameters=[str(repo / 'config/hardware_localization.yaml'), params])]


def generate_launch_description():
    config_dir = Path(__file__).resolve().parent
    params = str(config_dir / 'hardware_localization.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_path'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('home_pose', default_value=str(
            config_dir.parent / 'maps/hardware/home-pose.yaml')),
        DeclareLaunchArgument('use_home', default_value='true'),
        # Any node exit stops the session instead of leaving a frozen viewer.
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='A localization component exited'))
        ])),
        Node(package='rtabmap_odom', executable='icp_odometry',
             name='icp_odometry', namespace='localization', output='screen',
             parameters=[params],
             remappings=[('scan', '/scan'), ('odom', '/localization/odom')]),
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', namespace='localization', output='screen',
             parameters=[params, {'yaml_filename': LaunchConfiguration('map_path')}]),
        OpaqueFunction(function=create_amcl),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', namespace='localization',
             output='screen', parameters=[params]),
        Node(package='rviz2', executable='rviz2', name='localization_rviz',
             output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(config_dir / 'hardware_localization.rviz')]),
    ])
