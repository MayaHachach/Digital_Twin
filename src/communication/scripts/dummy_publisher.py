#!/usr/bin/env python3

import math
import time
import random
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, BatteryState
from irobot_create_msgs.msg import WheelVels


class LocobotDummyPublisher(Node):
    def __init__(self):
        super().__init__("locobot_dummy_publisher")

        # Setup publishers
        self.joint_pub = self.create_publisher(JointState, "/wx200/joint_states", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/locobot1/battery_state", 10)
        self.wheel_vels_pub = self.create_publisher(WheelVels, "/locobot1/wheel_vels", 10)

        # Joint state simulation setup
        self.joint_names: List[str] = [
            "waist", "shoulder", "elbow", "wrist_angle",
            "wrist_rotate", "gripper", "right_finger", "left_finger"
        ]
        self.A = [0.5, 0.5, 0.7, 0.3, 0.3, 0.02, 0.03, 0.03]  # amplitude
        self.X0 = [0.0] * len(self.joint_names)              # offset
        self.W = [0.6, 0.5, 0.7, 0.8, 0.8, 0.03, 0.02, 0.09]  # frequency
        self.t0 = time.time()

        # Run at 30 Hz
        self.timer = self.create_timer(1.0, self.publish_all)
        self.get_logger().info("Merged Locobot dummy publisher started")

    def publish_all(self):
        self.publish_joint_states()
        self.publish_battery_state()
        self.publish_wheel_vels()

    def publish_joint_states(self):
        t = time.time() - self.t0
        pos, vel, eff = [], [], []

        for i in range(len(self.joint_names)):
            Ai, Xi, Wi = self.A[i], self.X0[i], self.W[i]
            p = Xi + Ai * math.sin(Wi * t)
            v = Ai * Wi * math.cos(Wi * t)
            pos.append(p)
            vel.append(v)
            eff.append(0.05 * abs(v))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff

        self.joint_pub.publish(msg)

    def publish_battery_state(self):
        msg = BatteryState()
        msg.percentage = random.uniform(0.0, 1.0)
        msg.voltage = random.uniform(11.0, 12.6)
        msg.current = random.uniform(-1.0, 1.0)
        msg.temperature = random.uniform(20.0, 40.0)
        self.battery_pub.publish(msg)

    def publish_wheel_vels(self):
        msg = WheelVels()
        msg.velocity_left = random.uniform(-1.0, 1.0)
        msg.velocity_right = random.uniform(-1.0, 1.0)
        self.wheel_vels_pub.publish(msg)


def main():
    rclpy.init()
    node = LocobotDummyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
