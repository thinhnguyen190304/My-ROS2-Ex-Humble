import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # PATHS
    description_pkg = get_package_share_directory("my_robot_description")
    moveit_pkg = get_package_share_directory("my_robot_moveit_config")
    
    urdf_file = os.path.join(description_pkg, "urdf", "my_robot.urdf.xacro")
    srdf_file = os.path.join(moveit_pkg, "config", "my_robot.srdf")

    # Build MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    # MoveGroup node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "planning_pipelines.pipeline_names": ["ompl"],
                "default_planning_pipeline": "ompl",
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            }
        ],
    )

    # RViz
    rviz_config_file = os.path.join(moveit_pkg, "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    # Static TF for world (Using modern arguments)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "world", "--child-frame-id", "base_footprint"],
    )

    return LaunchDescription([
        static_tf,
        run_move_group_node,
        rviz_node,
    ])
