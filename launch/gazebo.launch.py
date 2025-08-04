"""
ROS 2 (Humble) launch file that spawns an Intel® RealSense™ D435 depth camera
URDF/Xacro into an empty Gazebo world.

This is a one‑to‑one conversion of the original ROS 1 XML launch snippet the
user provided.  All original launch arguments are preserved with the same
semantics, but implemented using ROS 2 Launch API best‑practices.

Usage (from an overlay workspace that contains realsense2_description):

    ros2 launch <your_pkg> d435_camera_gazebo_launch.py \
        paused:=false use_sim_time:=true gui:=true headless:=false \
        debug:=false \
        model:=$(ros2 pkg prefix realsense2_description)/share/realsense2_description/urdf/test_d435_camera.urdf.xacro
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    # ───────────────────────────  Launch arguments  ────────────────────────────
    paused_arg    = DeclareLaunchArgument('paused',   default_value='false',
                                          description='Start Gazebo in a paused state')
    sim_time_arg  = DeclareLaunchArgument('use_sim_time', default_value='true',
                                          description='Use /clock from simulation')
    gui_arg       = DeclareLaunchArgument('gui',      default_value='true',
                                          description='Whether to launch gzclient')
    headless_arg  = DeclareLaunchArgument('headless', default_value='false',
                                          description='Run Gazebo head‑less (no rendering)')
    debug_arg     = DeclareLaunchArgument('debug',    default_value='false',
                                          description='Enable Gazebo verbose/debug')
    model_arg     = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('realsense2_description'),
            'urdf',
            'test_d435_camera.urdf.xacro'
        ]),
        description='Path to the camera Xacro/URDF model to spawn'
    )

    # ─────────────────────────  Robot description  ────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        LaunchConfiguration('model')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher (optional but useful for TF tree)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ─────────────────────────────  Gazebo  ───────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'verbose': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('paused'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # ────────────────────────  Spawn the camera  ──────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_d435',
        arguments=[
            '-entity', 'd435_camera',
            '-topic', 'robot_description',
            '-z', '1.0'
        ],
        output='screen'
    )

    # ───────────────────────────── Description  ──────────────────────────────
    return LaunchDescription([
        # Declare all user‑configurable launch arguments first
        paused_arg,
        sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        model_arg,

        # Launch Gazebo, RSP, and spawn our camera entity
        gazebo_launch,
        rsp_node,
        spawn_entity,
    ])
