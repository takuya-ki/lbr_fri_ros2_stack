
#!/usr/bin/python3
from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class MoveGroupActionClientNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.action_server = "/move_action"
        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )

        self.get_logger().info(f"Waiting for action server {self.action_server}...")
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )
        self.get_logger().info(f"Done.")

    def send_goal_async(self, target: Pose, move_group_name, base, end_effector):

        goal = MoveGroup.Goal()
        goal.request.allowed_planning_time = 1.0
        goal.request.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=base),
                        link_name=end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=base),
                        link_name=end_effector,
                        orientation=target.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        goal.request.group_name = move_group_name
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.num_planning_attempts = 1

        return self.move_group_action_client.send_goal_async(goal)


def main(args: List = None) -> None:
    rclpy.init(args=args)
    node = MoveGroupActionClientNode("hello_moveit")

    right_move_group_name = "lbr_right"
    right_base = "lbr_right_link_0"
    right_end_effector = "lbr_right_link_ee"

    left_move_group_name = "lbr_left"
    left_base = "lbr_left_link_0"
    left_end_effector = "lbr_left_link_ee"

    # Note that this is in the robot coordinate system
    pose = Pose(
        position=Point(x=0.0, y=0.0, z=1.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    future_right = node.send_goal_async(
        pose,
        right_move_group_name,
        right_base,
        right_end_effector)
    future_left = node.send_goal_async(
        pose,
        left_move_group_name,
        left_base,
        left_end_effector)

    done_flags = {"right_goal": False, "left_goal": False}

    def done_cb_right(future):
        print("Right goal result received")
        done_flags["right_goal"] = True

    def done_cb_left(future):
        print("Left goal result received")
        done_flags["left_goal"] = True

    future_right.add_done_callback(done_cb_right)
    future_left.add_done_callback(done_cb_left)

    while not all(done_flags.values()):
        rclpy.spin_once(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()