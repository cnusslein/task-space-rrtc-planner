import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    # parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    # parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines("ompl", ["ompl"])
        .to_moveit_configs()
    )

    tsrrtc_example = Node(
        name="tsrrtc_example",
        package="task_space_rrtc_planner",
        executable="tsrrtc_example",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )


    # Command-line arguments
    # tutorial_arg = DeclareLaunchArgument(
    #     "rviz_tutorial", default_value="False", description="Tutorial flag"
    # )

    # kinematics_yaml = load_yaml(
    #     "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    # )

    # RViz
    # tutorial_mode = LaunchConfiguration("rviz_tutorial")
    # rviz_base = os.path.join(get_package_share_directory("moveit2_tutorials"), "launch")
    # rviz_full_config = os.path.join(rviz_base, "panda_moveit_config_demo.rviz")
    # rviz_empty_config = os.path.join(rviz_base, "panda_moveit_config_demo_empty.rviz")
    # rviz_node_tutorial = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_empty_config],
    #     parameters=[kinematics_yaml],
    #     condition=IfCondition(tutorial_mode),
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_full_config],
    #     parameters=[kinematics_yaml],
    #     condition=UnlessCondition(tutorial_mode),
    # )

    return LaunchDescription([
        tsrrtc_example,
    ])
