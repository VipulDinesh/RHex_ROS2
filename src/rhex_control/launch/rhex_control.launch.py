from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # --- Package paths ---
    pkg_rhex_desc = get_package_share_directory('rhex_description')
    pkg_rhex_gazebo = get_package_share_directory('rhex_gazebo')
    pkg_rhex_control = get_package_share_directory('rhex_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Files ---
    xacro_file = os.path.join(pkg_rhex_desc, 'urdf', 'rhex.xacro')
    world_file = os.path.join(pkg_rhex_gazebo, 'worlds', 'empty.world')
    controller_yaml = os.path.join(pkg_rhex_control, 'config', 'rhex_controller.yaml')

    # --- Process Xacro to URDF ---
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    # --- Start Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r {world_file}"}.items()
    )

    # --- Spawn RHex in Gazebo ---
    spawn_rhex = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'rhex', '-string', robot_description_config],
        output='screen'
    )

    # --- Robot State Publisher (publishes /robot_description) ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # --- Joint State Broadcaster ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controllers', '--controller-manager', '/controller_manager'],
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        spawn_rhex,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner
    ])
