from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    model = LaunchConfiguration("model").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = False
    if mode == "gazebo":
        use_sim_time = True

    # generate moveit configs
    moveit_configs = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )

    # launch demo node
    ld.add_action(
        Node(
            package="lbr_moveit_py",
            executable="hello_moveit_action",
            parameters=[
                moveit_configs.to_dict(),
                {"use_sim_time": use_sim_time},
                {
                    "planning_pipeline_id": LaunchConfiguration("planning_pipeline_id"),
                    "planner_id": LaunchConfiguration("planner_id"),
                },
                LBRDescriptionMixin.param_robot_name(),
            ],
        )
    )
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(
        DeclareLaunchArgument(
            name="planning_pipeline_id",
            default_value="ompl",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="planner_id",
            default_value="RRTConnectkConfigDefault",
        )
    )

    ld.add_action(OpaqueFunction(function=hidden_setup))

    return ld
