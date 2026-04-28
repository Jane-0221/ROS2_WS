#!/usr/bin/python3

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from medipick_planning_interfaces.srv import PlanToPose


class PlanToPoseClient(Node):
    def __init__(self) -> None:
        super().__init__("medipick_plan_to_pose_client")
        self._client = self.create_client(PlanToPose, "/medipick_planning_server/plan_to_pose")

    def wait_until_ready(self) -> bool:
        return self._client.wait_for_service(timeout_sec=5.0)

    def send_request(self, pose: PoseStamped, group_name: str, pose_link: str) -> PlanToPose.Response:
        request = PlanToPose.Request()
        request.group_name = group_name
        request.pose_link = pose_link
        request.use_start_joint_state = False
        request.target_pose = pose
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 3
        request.max_velocity_scaling = 0.25
        request.max_acceleration_scaling = 0.25

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Planning service did not return a response in time.")
        return future.result()


def build_pose_from_argv(argv: list[str]) -> PoseStamped:
    if len(argv) < 8:
        raise ValueError(
            "Usage: plan_to_pose_client.py frame_id x y z qx qy qz qw [group_name] [pose_link]"
        )

    pose = PoseStamped()
    pose.header.frame_id = argv[0]
    pose.pose.position.x = float(argv[1])
    pose.pose.position.y = float(argv[2])
    pose.pose.position.z = float(argv[3])
    pose.pose.orientation.x = float(argv[4])
    pose.pose.orientation.y = float(argv[5])
    pose.pose.orientation.z = float(argv[6])
    pose.pose.orientation.w = float(argv[7])
    return pose


def main() -> None:
    rclpy.init()
    node = PlanToPoseClient()
    try:
        args = sys.argv[1:]
        pose = build_pose_from_argv(args)
        group_name = args[8] if len(args) > 8 else "mobile_arm"
        pose_link = args[9] if len(args) > 9 else "sucker_link"

        if not node.wait_until_ready():
            raise RuntimeError("Planning service /medipick_planning_server/plan_to_pose is unavailable.")

        response = node.send_request(pose, group_name, pose_link)
        trajectory_points = len(response.joint_trajectory.points)
        print(f"success={response.success}")
        print(f"backend={response.backend_name}")
        print(f"group={response.resolved_group_name}")
        print(f"pose_link={response.resolved_pose_link}")
        print(f"planning_frame={response.planning_frame}")
        print(f"moveit_error_code={response.moveit_error_code}")
        print(f"planning_time={response.planning_time:.3f}")
        print(f"trajectory_points={trajectory_points}")
        print(f"final_joint_state_names={list(response.final_joint_state.name)}")
        print(f"message={response.message}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
