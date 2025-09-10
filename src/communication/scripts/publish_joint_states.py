#!/usr/bin/env python3
"""
Safe /joint_states publisher for ROS 2 (arm only).
Generates sinusoidal motions within safe joint limits for Isaac Sim.
"""

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateDummyPublisher(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_dummy_publisher_safe")

        # Arm joints only, matching Isaac Sim names
        self.names: List[str] = [
            "waist",
            "shoulder",
            "elbow",
            "wrist_angle",
            "wrist_rotate",
            "gripper",
            "right_finger",
            "left_finger",

        ]

        # Safe motion amplitudes (rad)
        self.A = [0.5, 0.5, 0.7, 0.3, 0.3, 0.02, 0.03, 0.03, 0.05]  # reduced amplitudes
        self.X0 = [0.0] * len(self.names)  # neutral offsets
        self.W = [0.6, 0.5, 0.7, 0.8, 0.8, 0.03, 0.02, 0.09, 0.05]  # angular frequencies

        self.t0 = time.time()
        self.pub = self.create_publisher(JointState, "/wx200/joint_states", 10)
        self.timer = self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info("Publishing safe /wx200/joint_states at 30 Hz")

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        t = time.time() - self.t0

        pos = [0.0] * len(self.names)
        vel = [0.0] * len(self.names)
        eff = [0.0] * len(self.names)

        for idx in range(len(self.names)):
            Ai = self.A[idx]
            Xi = self.X0[idx]
            Wi = self.W[idx]
            # Compute sine-wave motion
            p = Xi + Ai * math.sin(Wi * t)
            v = Ai * Wi * math.cos(Wi * t)
            pos[idx] = p
            vel[idx] = v
            eff[idx] = 0.05 * abs(v)  # smaller effort for stability

        msg = JointState()
        msg.header.stamp = now
        msg.header.frame_id = ""
        msg.name = self.names
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateDummyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
