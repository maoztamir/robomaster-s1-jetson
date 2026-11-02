"""Nav2 preview only: all velocity output stays on disconnected preview topics."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    import math
    length = float(LaunchConfiguration('length').perform(context))
    width = float(LaunchConfiguration('width').perform(context))
    front = float(LaunchConfiguration('laser_to_front').perform(context))
    if not all(math.isfinite(v) for v in (length, width, front)) or not (
            .1 < length < 2 and .1 < width < 2 and 0 < front < length):
        raise ValueError('Provide measured length, width and laser-to-front distance in metres')
    # The laser is level and centred laterally. No unmeasured chassis TF is added.
    footprint = str([[front, width/2], [front, -width/2],
                     [front-length, -width/2], [front-length, width/2]])
    params = str(Path(__file__).resolve().parent / 'hardware_nav2_preview.yaml')
    namespace = 'hardware_navigation'
    # Costmaps are child nodes: their footprint must be set through a parameter
    # file with their fully qualified names, rather than the parent's override.
    from tempfile import NamedTemporaryFile
    import yaml
    with NamedTemporaryFile(mode='w', prefix='hardware-footprint-', suffix='.yaml',
                            delete=False) as out:
        yaml.safe_dump({f'/hardware_navigation/{name}/{name}': {
            'ros__parameters': {'footprint': footprint}}
            for name in ('local_costmap', 'global_costmap')}, out)
        footprint_file = out.name
    # Apply the footprint to child costmaps through each process's parameters.
    nodes = [Node(package=package, executable=executable, name=executable,
                  namespace=namespace, output='screen', parameters=[params, footprint_file],
                  remappings=[('cmd_vel', '/hardware_navigation/cmd_vel_nav'),
                              ('cmd_vel_smoothed', '/hardware_navigation/cmd_vel_preview')])
             for package, executable in (
                 ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
                 ('nav2_smoother', 'smoother_server'), ('nav2_behaviors', 'behavior_server'),
                 ('nav2_bt_navigator', 'bt_navigator'), ('nav2_velocity_smoother', 'velocity_smoother'))]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_preview', namespace=namespace, output='screen',
                      parameters=[{'use_sim_time': False, 'autostart': True,
                                   'node_names': ['controller_server', 'planner_server',
                                                  'smoother_server', 'behavior_server',
                                                  'velocity_smoother', 'bt_navigator']}]))
    from launch.event_handlers import OnShutdown
    nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda context: Path(footprint_file).unlink(missing_ok=True))])))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('length', description='Measured overall length in metres'),
        DeclareLaunchArgument('width', description='Measured overall width in metres'),
        DeclareLaunchArgument('laser_to_front', description='Laser centre to front edge in metres'),
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='A Nav2 preview component exited'))])),
        OpaqueFunction(function=create_nodes),
    ])
