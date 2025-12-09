# robot_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('disaster_robot_description')
    world_path = os.path.join(pkg_dir, 'worlds', 'disaster_scene.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'disaster_robot.urdf')  # if xacro, change extension accordingly

    # Launch args for spawn pose
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='spawn x')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='spawn y')
    declare_z = DeclareLaunchArgument('z', default_value='0.0', description='spawn z')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='spawn yaw (radians)')

    x_arg = LaunchConfiguration('x')
    y_arg = LaunchConfiguration('y')
    z_arg = LaunchConfiguration('z')
    yaw_arg = LaunchConfiguration('yaw')

    # Use the gazebo_ros package's provided launch to start Gazebo more robustly
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        # pass the world file
        launch_arguments={'world': world_path}.items()
    )

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn entity node -- delay with TimerAction so Gazebo services are up
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_path,
            '-entity', 'disaster_robot',
            '-x', x_arg, '-y', y_arg, '-z', z_arg,
            '-Y', yaw_arg   # spawn_entity expects -Y for yaw; we kept LaunchConfiguration name 'yaw'
        ],
        output='screen'
    )

    # Wrap spawn in a short timer to avoid racing with Gazebo init (1.5s is usually enough)
    delayed_spawn = TimerAction(
        period=1.5,
        actions=[spawn_node]
    )

    ld = LaunchDescription()

    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)

    # start gazebo through the included launch (this handles plugin loading)
    ld.add_action(gazebo_launch)

    # start robot_state_publisher (it can run before spawn)
    ld.add_action(rsp_node)

    # spawn after a short delay
    ld.add_action(delayed_spawn)

    return ld
