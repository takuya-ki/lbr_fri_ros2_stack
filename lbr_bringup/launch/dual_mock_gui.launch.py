from launch import LaunchDescription
from launch_ros.actions import Node
from lbr_bringup.dual_description import DualLBRDescriptionMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(DualLBRDescriptionMixin.arg_model())

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

    # robot_state_publisher (/lbr/joint_states)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        remappings=[
            ("/joint_states", "/lbr/joint_states")
        ]
    )
    ld.add_action(robot_state_publisher)

    # joint_state_publisher_gui (/lbr/joint_states)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        remappings=[
            ("/joint_states", "/lbr/joint_states")
        ]
    )
    ld.add_action(joint_state_publisher_gui)

    return ld