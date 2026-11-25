from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class DualLBRROS2ControlMixin:
    @staticmethod
    def arg_ctrl_cfg_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg_pkg",
            default_value="lbr_description",
            description="Controller configuration package. The package containing the ctrl_cfg.",
        )

    @staticmethod
    def arg_ctrl_cfg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg",
            default_value="ros2_control/lbr_controllers.yaml",
            description="Relative path from ctrl_cfg_pkg to the controllers.",
        )

    @staticmethod
    def arg_lbr_common_ctrl() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="lbr_ctrl",
            default_value="joint_trajectory_controller",
            description="Desired default controller for the right arm. One of specified in ctrl_cfg.",
            choices=[
                "right_admittance_controller",
                "right_joint_trajectory_controller",
                "right_forward_position_controller",
                "lbr_right_joint_position_command_controller",
                "lbr_right_torque_command_controller",
                "lbr_right_wrench_command_controller",
                "right_twist_controller",
            ],
        )

    @staticmethod
    def arg_lbr_right_ctrl() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="lbr_right_ctrl",
            default_value="right_joint_trajectory_controller",
            description="Desired default controller for the right arm. One of specified in ctrl_cfg.",
            choices=[
                "right_admittance_controller",
                "right_joint_trajectory_controller",
                "right_forward_position_controller",
                "lbr_right_joint_position_command_controller",
                "lbr_right_torque_command_controller",
                "lbr_right_wrench_command_controller",
                "right_twist_controller",
            ],
        )

    @staticmethod
    def arg_lbr_left_ctrl() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="lbr_left_ctrl",
            default_value="left_joint_trajectory_controller",
            description="Desired default controller for the left arm. One of specified in ctrl_cfg.",
            choices=[
                "left_admittance_controller",
                "left_joint_trajectory_controller",
                "left_forward_position_controller",
                "lbr_left_joint_position_command_controller",
                "lbr_left_torque_command_controller",
                "lbr_left_wrench_command_controller",
                "left_twist_controller",
            ],
        )

    @staticmethod
    def arg_sys_cfg_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sys_cfg_pkg",
            default_value="lbr_description",
            description="Package containing the lbr_system_config.yaml file for FRI configurations.",
        )

    @staticmethod
    def arg_lbr_right_sys_cfg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="lbr_right_sys_cfg",
            default_value="ros2_control/lbr_right_system_config.yaml",
            description="The relative path from sys_cfg_pkg to the lbr_right_system_config.yaml file.",
        )

    @staticmethod
    def arg_lbr_left_sys_cfg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="lbr_left_sys_cfg",
            default_value="ros2_control/lbr_left_system_config.yaml",
            description="The relative path from sys_cfg_pkg to the lbr_right_system_config.yaml file.",
        )

    @staticmethod
    def arg_use_sim_time() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )

    @staticmethod
    def node_ros2_control(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        use_sim_time: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "use_sim_time", default="false"
        ),
        ctrl_cfg_name: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "ctrl_cfg_name", default="ctrl_cfg"
        ),
        ctrl_cfg_path: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "ctrl_cfg_path", default="ros2_control/lbr_controllers.yaml"
        ),
        robot_description: Optional[
            Dict[str, str]
        ] = {},  # required for certain ROS 2 controllers in Humble
        **kwargs,
    ) -> Node:
        return Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"use_sim_time": use_sim_time},
                PathJoinSubstitution(
                    [
                        FindPackageShare(
                            LaunchConfiguration(
                                "ctrl_cfg_pkg", default="lbr_description"
                            )
                        ),
                        LaunchConfiguration(
                            ctrl_cfg_name, default=ctrl_cfg_path
                        ),
                    ]
                ),
                robot_description,
            ],
            namespace=robot_name,
            # remappings=[
            #     ("~/robot_description", "robot_description"),
            # ],
            **kwargs,
        )

    @staticmethod
    def node_controller_spawner(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        controller: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "ctrl"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller,
                "--controller-manager",
                "controller_manager",
            ],
            namespace=robot_name,
            **kwargs,
        )

    @staticmethod
    def node_robot_state_publisher(
        robot_description: Dict[str, str],
        robot_name: Optional[LaunchConfiguration] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        use_sim_time: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "use_sim_time", default="false"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
            ],
            namespace=robot_name,
            **kwargs,
        )
