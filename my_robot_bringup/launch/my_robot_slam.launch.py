from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys
import subprocess


def generate_launch_description():
    # Launch arg for sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package directories
    bringup_dir = get_package_share_directory('my_robot_bringup')
    description_dir = get_package_share_directory('my_robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # -------------------- File Paths --------------------
    urdf_file = os.path.join(description_dir, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(bringup_dir, 'world', 'my_world.world')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'urdf_config.rviz')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    # -------------------- File Existence Checks --------------------
    missing_files = []
    for f in [urdf_file, world_file, rviz_config_file, nav2_launch_file, nav2_params_file]:
        if not os.path.exists(f):
            missing_files.append(f)

    if missing_files:
        print("\n[ERROR] Missing required file(s):")
        for f in missing_files:
            print(f" - {f}")
        print("\nFix the missing file(s) before launching.\n")
        sys.exit(1)

    # -------------------- Robot Description --------------------
    try:
        robot_description_config = subprocess.check_output(['xacro', urdf_file]).decode()
    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] Failed to process xacro file: {urdf_file}")
        print(e.output.decode())
        sys.exit(1)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    # -------------------- Gazebo Launch --------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # -------------------- SLAM Toolbox --------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'localization',
        }]
    )

    # -------------------- Nav2 Bringup --------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )

    # -------------------- RViz (Launch after robot_state_publisher starts) --------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    delayed_rviz = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[rviz_node]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated clock'
        ),
        LogInfo(msg=f"Launching with URDF: {urdf_file}"),
        LogInfo(msg=f"Launching with World: {world_file}"),
        LogInfo(msg=f"Launching with RViz config: {rviz_config_file}"),
        LogInfo(msg=f"Launching with Nav2 params: {nav2_params_file}"),
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        slam_toolbox_node,
        nav2_launch,
        delayed_rviz  # <-- ensures RViz launches only after robot_state_publisher is running
    ])
