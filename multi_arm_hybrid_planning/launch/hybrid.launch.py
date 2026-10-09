"""Robot-independent Hybrid Planning components. Supply a complete robot config YAML."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def generate_launch_description():
    config = LaunchConfiguration("robot_config")
    local = yaml.safe_load((Path(get_package_share_directory("multi_arm_hybrid_planning")) /
                            "config" / "local_planner.yaml").read_text())
    return LaunchDescription([
        DeclareLaunchArgument("robot_config", description="Absolute YAML path with robot descriptions, aggregate group and STOMP pipeline"),
        DeclareLaunchArgument("legacy_runtime", default_value="false", description="Use the upstream single-goal HybridPlanner manager instead of concurrent ExecuteMotion"),
        Node(package="multi_arm_hybrid_planning", executable="coordinator", output="screen",
             condition=UnlessCondition(LaunchConfiguration("legacy_runtime")), parameters=[config]),
        Node(package="multi_arm_hybrid_planning", executable="hybrid_runtime", output="screen",
             condition=IfCondition(LaunchConfiguration("legacy_runtime")),
             parameters=[local, {
                 "global_planner_name": "multi_arm_hybrid_planning/StompGlobal",
                 "planner_logic_plugin_name": "moveit_hybrid_planning/ReplanInvalidatedTrajectory",
             }, config]),
    ])
