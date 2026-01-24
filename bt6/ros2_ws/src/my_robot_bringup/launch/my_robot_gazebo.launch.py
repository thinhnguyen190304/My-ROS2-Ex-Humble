import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # PATHS
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_bringup, 'worlds', 'final_project.world'),
        description='Full path to world file'
    )

    # Robot State Publisher
    xacro_file = os.path.join(pkg_description, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Spawn Robot (Original Position)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Spawn Practice Box (Near Robot)
    spawn_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'practice_box',
            '-file', os.path.join(pkg_description, 'urdf', 'practice_box.sdf'),
            '-x', '0.7', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}]
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        parameters=[{'use_sim_time': True}]
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
