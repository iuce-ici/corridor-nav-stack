import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_corridor = get_package_share_directory('corridor_sim')
    pkg_vehicle = get_package_share_directory('tunnel_vehicle')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_corridor, 'worlds', 'corridor.sdf')
    xacro_path = os.path.join(pkg_vehicle, 'urdf', 'tunnel_vehicle.urdf.xacro')

    robot_description = xacro.process_file(xacro_path).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'corridor',
            '-topic', 'robot_description',
            '-name', 'vehicle',
            '-x', '10.0', '-y', '0.0', '-z', '0.05',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Gazebo needs a moment to come up before anything spawns into it.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn])

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_spawn,
    ])