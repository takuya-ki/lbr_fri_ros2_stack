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
    PLANNING_PIPELINE_ID = "ompl"
    # Supported planner_id values registered in ompl_planning.yaml:
    PLANNER_ID_OPTIONS = (
        "ESTkConfigDefault",
        "RRTkConfigDefault",
        "RRTConnectkConfigDefault",
        "RRTstarkConfigDefault",
        "TRRTkConfigDefault",
        "PRMkConfigDefault",
        "PRMstarkConfigDefault",
        "BiTRRTkConfigDefault",
        "LBTRRTkConfigDefault",
        "BiESTkConfigDefault",
        "LazyPRMstarkConfigDefault",
        "SPARSkConfigDefault",
    )
    # Default planner_id. Set this to one of PLANNER_ID_OPTIONS above.
    PLANNER_ID = "RRTConnectkConfigDefault"

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("planning_pipeline_id", self.PLANNING_PIPELINE_ID)
        self.declare_parameter("planner_id", self.PLANNER_ID)
        self.planning_pipeline_id = (
            self.get_parameter("planning_pipeline_id").get_parameter_value().string_value
        )
        self.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        if self.planner_id not in self.PLANNER_ID_OPTIONS:
            raise ValueError(
                f"Unsupported planner_id '{self.planner_id}'. "
                f"Available planner_ids: {', '.join(self.PLANNER_ID_OPTIONS)}"
            )

        self.action_server = "/lbr/move_action"
        self.move_group_name = "arm"
        self.base = "lbr_link_0"
        self.end_effector = "lbr_link_ee"

        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )

        self.get_logger().info(f"Waiting for action server {self.action_server}...")
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )
        self.get_logger().info("Done.")
        self.get_logger().info(
            f"Available OMPL planner_ids: {', '.join(self.PLANNER_ID_OPTIONS)}"
        )
        self.get_logger().info(
            "Configured MoveGroup request defaults with "
            f"pipeline_id='{self.planning_pipeline_id}', "
            f"planner_id='{self.planner_id}'"
        )

    def send_goal_async(self, target: Pose):
        goal = MoveGroup.Goal()
        goal.request.allowed_planning_time = 1.0
        goal.request.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[
                                SolidPrimitive(
                                    type=SolidPrimitive.SPHERE,
                                    dimensions=[0.0001],
                                )
                            ],
                            primitive_poses=[Pose(position=target.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        orientation=target.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        goal.request.group_name = self.move_group_name
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.num_planning_attempts = 1
        goal.request.pipeline_id = self.planning_pipeline_id
        goal.request.planner_id = self.planner_id
        self.get_logger().info(
            "Sending MoveGroup goal with "
            f"pipeline_id='{goal.request.pipeline_id}', "
            f"planner_id='{goal.request.planner_id}', "
            f"group_name='{goal.request.group_name}'"
        )

        return self.move_group_action_client.send_goal_async(goal)


def main(args: List = None) -> None:
    rclpy.init(args=args)
    node = MoveGroupActionClientNode("hello_moveit_action")

    # Note that this is in the robot coordinate system
    pose = Pose(
        position=Point(x=0.0, y=0.0, z=1.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    future = node.send_goal_async(pose)
    rclpy.spin_until_future_complete(
        node, future
    )  # gets stuck for invalid goals

    rclpy.shutdown()


if __name__ == "__main__":
    main()
