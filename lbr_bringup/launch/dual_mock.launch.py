from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.dual_description import DualLBRDescriptionMixin
from lbr_bringup.dual_ros2_control import DualLBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(DualLBRDescriptionMixin.arg_model())
    ld.add_action(DualLBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(DualLBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(DualLBRROS2ControlMixin.arg_lbr_right_ctrl())
    ld.add_action(DualLBRROS2ControlMixin.arg_lbr_left_ctrl())

    # static transform world -> <robot_name>_floating_link
    ld.add_action(
        DualLBRDescriptionMixin.node_static_tf(
            tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            parent="world",
            child="lbr_right_floating_link",
        )
    )
    ld.add_action(
        DualLBRDescriptionMixin.node_static_tf(
            tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            parent="world",
            child="lbr_left_floating_link"
        )
    )

    # robot description
    robot_description = DualLBRDescriptionMixin.param_robot_description(mode="mock")

    # robot state publisher
    robot_state_publisher = DualLBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description,
        use_sim_time=False,
        robot_name="",
        remappings=[
            ("~/robot_description", "robot_description"),
            # ("/joint_states", "/lbr/joint_states")
        ],
    )
    ld.add_action(robot_state_publisher)

    # ros2 control node
    common_ros2_control_node = DualLBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False,
        robot_description=robot_description,
        robot_name="",
        ctrl_cfg_name="lbr_ctrl_cfg",
        ctrl_cfg_path="ros2_control/lbr_common_controllers.yaml",
        remappings=[
            ("~/robot_description", "robot_description"),
            # ("/joint_states", "/lbr/joint_states")
        ],
    )
    ld.add_action(common_ros2_control_node)
    right_ros2_control_node = DualLBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False,
        robot_description=robot_description,
        robot_name="lbr_right",
        ctrl_cfg_name="lbr_right_ctrl_cfg",
        ctrl_cfg_path="ros2_control/lbr_right_controllers.yaml",
        remappings=[
            ("~/robot_description", "robot_description")],
    )
    ld.add_action(right_ros2_control_node)
    left_ros2_control_node = DualLBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False,
        robot_description=robot_description,
        robot_name="lbr_left",
        ctrl_cfg_name="lbr_left_ctrl_cfg",
        ctrl_cfg_path="ros2_control/lbr_left_controllers.yaml",
        remappings=[
            ("~/robot_description", "robot_description")],
    )
    ld.add_action(left_ros2_control_node)

    # joint state broad caster and controller on ros2 control node start
    joint_state_broadcaster = DualLBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster",
        robot_name=""
    )

    right_controller = DualLBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("lbr_right_ctrl"),
        robot_name="lbr_right"
    )
    left_controller = DualLBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("lbr_left_ctrl"),
        robot_name="lbr_left"
    )

    common_controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=common_ros2_control_node,
            on_start=[
                joint_state_broadcaster,
            ],
        )
    )
    ld.add_action(common_controller_event_handler)
    right_controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=right_ros2_control_node,
            on_start=[
                right_controller,
            ],
        )
    )
    ld.add_action(right_controller_event_handler)
    left_controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=left_ros2_control_node,
            on_start=[
                left_controller,
            ],
        )
    )
    ld.add_action(left_controller_event_handler)

    return ld
