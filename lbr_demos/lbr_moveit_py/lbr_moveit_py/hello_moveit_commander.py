#!/usr/bin/env python3
import threading
import time
from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException
from tf2_ros import LookupException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class CartesianPathExecutor(Node):
    WAIT_TIMEOUT_SEC = 10.0
    JOINT_STATE_TIMEOUT_SEC = 10.0
    TF_TIMEOUT_SEC = 5.0
    CARTESIAN_FRACTION_THRESHOLD = 0.95
    DURATION_PER_CARTESIAN_POINT_SEC = 0.5
    INITIAL_JOINT_POSITIONS = [
        -0.087,
        -0.366,
        -0.524,
        1.239,
        0.192,
        -1.571,
        1.745,
    ]

    def __init__(self) -> None:
        super().__init__("cartesian_path_executor")

        self.group_name = "arm"
        self.base_frame = "lbr_link_0"
        self.ee_link = "lbr_link_ee"
        self.joint_state_topic = "/lbr/joint_states"
        self.cartesian_srv_ns = "/lbr/compute_cartesian_path"
        self.action_ns = "/lbr/joint_trajectory_controller/follow_joint_trajectory"

        self.current_joint_state = None
        self.joint_state_lock = threading.Lock()

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.cartesian_client = self.create_client(GetCartesianPath, self.cartesian_srv_ns)
        self.get_logger().info(f"Waiting for {self.cartesian_srv_ns}...")
        if not self.cartesian_client.wait_for_service(
            timeout_sec=self.WAIT_TIMEOUT_SEC
        ):
            raise RuntimeError(f"Service not available: {self.cartesian_srv_ns}")
        self.get_logger().info("Service is available.")

        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_ns,
        )
        self.get_logger().info(f"Waiting for action server {self.action_ns}...")
        if not self.trajectory_action_client.wait_for_server(
            timeout_sec=self.WAIT_TIMEOUT_SEC
        ):
            raise RuntimeError(f"Action server not available: {self.action_ns}")
        self.get_logger().info("Action server available.")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self) -> Pose:
        deadline = time.monotonic() + self.TF_TIMEOUT_SEC
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_link,
                    rclpy.time.Time(),
                )
                return Pose(
                    position=Point(
                        x=transform.transform.translation.x,
                        y=transform.transform.translation.y,
                        z=transform.transform.translation.z,
                    ),
                    orientation=transform.transform.rotation,
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"TF lookup failed: {last_error}")

    def joint_state_callback(self, msg: JointState) -> None:
        with self.joint_state_lock:
            self.current_joint_state = msg

    def wait_for_joint_state(self) -> JointState:
        self.get_logger().info(f"Waiting for {self.joint_state_topic}...")
        deadline = time.monotonic() + self.JOINT_STATE_TIMEOUT_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            with self.joint_state_lock:
                if self.current_joint_state:
                    return self.current_joint_state
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"No joint state received on {self.joint_state_topic}")

    def send_trajectory(self, positions: List[float], duration_sec: int = 5) -> None:
        if len(positions) != 7:
            raise ValueError(f"Expected 7 joint values, got {len(positions)}.")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7",
        ]
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=positions,
                velocities=[0.0] * 7,
                accelerations=[0.0] * 7,
                time_from_start=Duration(sec=duration_sec),
            )
        )
        goal.goal_time_tolerance.sec = 1

        self.get_logger().info("Sending trajectory...")
        future = self.trajectory_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Trajectory execution failed with code {result.result.error_code}."
            )
        self.get_logger().info("Trajectory execution succeeded.")

    def plan_and_execute_cartesian_path(self, relx: float, rely: float, relz: float) -> None:
        joint_state = self.wait_for_joint_state()
        current_pose = self.get_current_pose()

        request = GetCartesianPath.Request()
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.header = Header(frame_id=self.base_frame)
        request.max_step = 0.02
        request.jump_threshold = 10.0
        request.avoid_collisions = False
        request.start_state = RobotState(joint_state=joint_state, is_diff=True)
        request.waypoints.append(
            Pose(
                position=Point(
                    x=current_pose.position.x + relx,
                    y=current_pose.position.y + rely,
                    z=current_pose.position.z + relz,
                ),
                orientation=current_pose.orientation,
            )
        )

        self.get_logger().info("Calling compute_cartesian_path...")
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            raise RuntimeError("compute_cartesian_path returned no result.")
        if result.fraction < self.CARTESIAN_FRACTION_THRESHOLD:
            raise RuntimeError(
                "Cartesian path fraction "
                f"{result.fraction:.3f} is below {self.CARTESIAN_FRACTION_THRESHOLD:.3f}."
            )

        trajectory = result.solution.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("compute_cartesian_path returned an empty trajectory.")

        joint_count = len(trajectory.joint_names)
        for index, point in enumerate(trajectory.points, start=1):
            total_sec = self.DURATION_PER_CARTESIAN_POINT_SEC * index
            point.time_from_start = Duration(
                sec=int(total_sec),
                nanosec=int((total_sec % 1) * 1e9),
            )
            point.velocities = [0.0] * joint_count
            point.accelerations = [0.0] * joint_count

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = trajectory.joint_names
        goal.trajectory.points = trajectory.points
        goal.goal_time_tolerance.sec = 1

        self.get_logger().info("Sending Cartesian trajectory...")
        send_future = self.trajectory_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if handle is None or not handle.accepted:
            raise RuntimeError("Cartesian trajectory goal rejected.")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                "Cartesian trajectory execution failed with code "
                f"{result.result.error_code}."
            )
        self.get_logger().info("Cartesian trajectory execution succeeded.")

    def run_sequence(self) -> None:
        self.get_logger().info("Step 1: moving to the initial position.")
        self.send_trajectory(self.INITIAL_JOINT_POSITIONS, duration_sec=8)

        self.get_logger().info("Step 2: executing the Cartesian path.")
        self.plan_and_execute_cartesian_path(relx=0.0, rely=0.0, relz=-0.03)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CartesianPathExecutor()
        node.run_sequence()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
