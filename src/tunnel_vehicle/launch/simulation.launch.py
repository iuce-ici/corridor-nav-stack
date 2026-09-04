import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit



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
        launch_arguments={'gz_args': f'-r --seed 42 -v 4 {world_path}'}.items(),
    )

    spawn_y = LaunchConfiguration('y')
    declare_y = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='Lateral spawn offset from the corridor centreline, metres')

    target_x = LaunchConfiguration('target_x')
    declare_target_x = DeclareLaunchArgument(
        'target_x', default_value='110.0',
        description='Run terminates at this base_link x. 110 keeps 5.8 m clear of end wall degradation onset')

    speed = LaunchConfiguration('speed')
    declare_speed = DeclareLaunchArgument(
        'speed', default_value='2.0',
        description='Commanded forward speed, m/s')

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
            '-x', '10.0', '-y', spawn_y, '-z', '0.05',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_vehicle, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    dead_reckoning = Node(
        package='corridor_perception',
        executable='dead_reckoning',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    run_controller = Node(
        package='corridor_experiments',
        executable='run_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_x': target_x,
            'speed': speed,
        }],
    )

    # Gazebo needs a moment to come up before anything spawns into it.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn])

    # The create process exits cleanly once the entity exists, so both consumers
    # key on that event rather than on a launch timer. Launch timers count from
    # launch start and have no relationship to how long Gazebo takes to load.
    on_spawn_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                dead_reckoning,
                # Motion begins after spawn settling has decayed. Driving through
                # the settling transient would put a startup artefact in the first
                # metre of every run.
                TimerAction(period=3.0, actions=[run_controller]),
            ],
        )
    )

    return LaunchDescription([
        declare_y,
        declare_target_x,
        declare_speed,
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_spawn,
        on_spawn_complete,
    ])
