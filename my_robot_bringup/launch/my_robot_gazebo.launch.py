from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import sys

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_dir = get_package_share_directory('my_robot_bringup')
    description_dir = get_package_share_directory('my_robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(description_dir, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(bringup_dir, 'world', 'my_world.world')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'urdf_config.rviz')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    # Check files exist
    for f in [urdf_file, world_file, rviz_config_file, nav2_launch_file, nav2_params_file]:
        if not os.path.exists(f):
            print(f"[ERROR] Missing file: {f}")
            sys.exit(1)

    # Robot description
    try:
        robot_description_config = subprocess.check_output(['xacro', urdf_file]).decode()
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to process xacro: {urdf_file}")
        print(e.output.decode())
        sys.exit(1)

    # -------------------- Nodes --------------------

    # Robot State Publisher
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

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # STATIC TF: odom -> base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0','0','0','0','0','0','odom','base_link']
    )

    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',  # change to 'localization' if map already exists
        }]
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )

    # RViz (launch after robot_state_publisher)
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

    # -------------------- Launch Description --------------------
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        LogInfo(msg=f"Launching Gazebo with world: {world_file}"),
        LogInfo(msg=f"Launching robot URDF: {urdf_file}"),
        LogInfo(msg=f"Launching Nav2 params: {nav2_params_file}"),
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        static_tf_node,       # <-- Static TF
        slam_toolbox_node,    # <-- SLAM
        nav2_launch,          # <-- Nav2
        delayed_rviz
    ])
