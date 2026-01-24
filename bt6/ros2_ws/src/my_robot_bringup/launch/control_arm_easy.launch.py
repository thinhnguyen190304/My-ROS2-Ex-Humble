import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Joint trajectory controller GUI
    rqt_jtc = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_config}
        ]
    )

    return LaunchDescription([
        rqt_jtc
    ])
