from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_my_robot = FindPackageShare('my_robot_description').find('my_robot_description')
    urdf_path = PathJoinSubstitution([pkg_my_robot, 'urdf', 'my_robot.urdf'])

    return LaunchDescription([
        # Gazebo server (gzserver)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
            ),
            launch_arguments={'world': 'empty.world'}.items(),  # Bỏ dòng này nếu không có world file
        ),

        # Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
            ),
        ),

        # Robot state publisher (publish /robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            output='screen'
        ),

        # Spawn entity in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-x', '0', '-y', '0', '-z', '0.8'],
            output='screen'
        ),
    ])
