#!/usr/bin/env python3

import sys

import rclpy
from controller_manager import (
    configure_controller,
    load_controller,
    set_controller_parameters,
    switch_controllers,
)
from rclpy.node import Node


CONTROLLER_MANAGER = "/controller_manager"


class ControllerBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("medipick_controller_bootstrap")

    def _bootstrap_joint_trajectory_controller(
        self,
        controller_name: str,
        joints: list[str],
    ) -> bool:
        if not set_controller_parameters(
            self,
            CONTROLLER_MANAGER,
            controller_name,
            "type",
            "joint_trajectory_controller/JointTrajectoryController",
        ):
            return False

        if not set_controller_parameters(
            self, CONTROLLER_MANAGER, controller_name, "joints", joints
        ):
            return False
        if not set_controller_parameters(
            self,
            CONTROLLER_MANAGER,
            controller_name,
            "command_interfaces",
            ["position"],
        ):
            return False
        if not set_controller_parameters(
            self,
            CONTROLLER_MANAGER,
            controller_name,
            "state_interfaces",
            ["position", "velocity"],
        ):
            return False
        if not set_controller_parameters(
            self,
            CONTROLLER_MANAGER,
            controller_name,
            "allow_partial_joints_goal",
            True,
        ):
            return False

        if not load_controller(self, CONTROLLER_MANAGER, controller_name).ok:
            self.get_logger().error(f"Failed to load {controller_name}")
            return False

        if not configure_controller(self, CONTROLLER_MANAGER, controller_name).ok:
            self.get_logger().error(f"Failed to configure {controller_name}")
            return False

        return True

    def _bootstrap_joint_state_broadcaster(self) -> bool:
        controller_name = "joint_state_broadcaster"
        if not set_controller_parameters(
            self,
            CONTROLLER_MANAGER,
            controller_name,
            "type",
            "joint_state_broadcaster/JointStateBroadcaster",
        ):
            return False

        if not load_controller(self, CONTROLLER_MANAGER, controller_name).ok:
            self.get_logger().error(f"Failed to load {controller_name}")
            return False

        if not configure_controller(self, CONTROLLER_MANAGER, controller_name).ok:
            self.get_logger().error(f"Failed to configure {controller_name}")
            return False

        return True

    def run(self) -> int:
        if not self._bootstrap_joint_state_broadcaster():
            return 1

        if not self._bootstrap_joint_trajectory_controller(
            "mobile_arm_controller",
            [
                "base_x",
                "base_y",
                "base_theta",
                "raise_joint",
                "r1_joint",
                "r2_joint",
                "r3_joint",
                "r4_joint",
                "r5_joint",
                "r6_joint",
            ],
        ):
            return 1

        if not self._bootstrap_joint_trajectory_controller("tool_controller", ["sucker_joint"]):
            return 1

        if not switch_controllers(
            self,
            CONTROLLER_MANAGER,
            [],
            ["joint_state_broadcaster", "mobile_arm_controller", "tool_controller"],
            True,
            True,
            5.0,
            10.0,
        ).ok:
            self.get_logger().error("Failed to activate controllers")
            return 1

        self.get_logger().info("Controllers loaded, configured, and activated.")
        return 0


def main() -> int:
    rclpy.init()
    node = ControllerBootstrap()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
