#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math


class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.navigator = BasicNavigator()

        # Wait for Nav2 to be active
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is now active!")

        # Subscribe to /navigationGoal topic (expects PoseStamped message)
        self.subscription = self.create_subscription(
            PoseStamped,  # Updated to use PoseStamped
            '/navigationGoal',  # Topic name
            self.goal_callback,  # Callback function
            10  # Queue size
        )
        self.get_logger().info("Subscribed to /navigationGoal, waiting for goal messages...")

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w

        # Convert quaternion to yaw for better readability
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)

        self.get_logger().info(f"Received goal: x={x}, y={y}, yaw={yaw:.2f} radians")
        self.send_goal(x, y, yaw)

    def send_goal(self, x: float, y: float, yaw: float):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        # Convert yaw angle to quaternion
        goal_pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.get_logger().info(f"Sending goal to: x={x}, y={y}, yaw={yaw:.2f} radians")
        self.navigator.goToPose(goal_pose)

        # Monitor execution
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining}")

        # Get result after task completion
        self._handle_result()

    def _handle_result(self):
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            self.shutdown()

        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Goal failed!")

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def shutdown(self):
        self.get_logger().info("Shutting down Robot Navigator...")
        rclpy.shutdown()


def main():
    rclpy.init()
    navigator_node = RobotNavigator()
    rclpy.spin(navigator_node)  # Keep the node alive to listen for goals
    navigator_node.shutdown()


if __name__ == '__main__':
    main()
