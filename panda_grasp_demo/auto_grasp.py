#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MoveGroupInterface, MotionPlanRequestBuilder

import time

class AutoGraspDemo(Node):
    def __init__(self):
        super().__init__('auto_grasp_demo')

        self.moveit = MoveItPy(node=self)
        self.arm = self.moveit.get_move_group("panda_arm")

        self.run_sequence()

    def run_sequence(self):
        self.get_logger().info("MoveIt 2 Python API auto grasp demo started...")

        # 设置目标姿态
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.position.x = 0.4
        pose_goal.pose.position.y = 0.0
        pose_goal.pose.position.z = 0.2
        pose_goal.pose.orientation.w = 1.0

        self.arm.set_pose_target(pose_goal.pose)
        plan = self.arm.plan()
        if plan:
            self.arm.execute()
            self.get_logger().info("Moved to pre-grasp.")
        else:
            self.get_logger().error("Planning failed.")
            return

        # 模拟接近目标
        pose_goal.pose.position.z = 0.1
        self.arm.set_pose_target(pose_goal.pose)
        plan = self.arm.plan()
        if plan:
            self.arm.execute()
            self.get_logger().info("Approached object.")
        else:
            self.get_logger().error("Failed to approach.")

        # 抬起
        pose_goal.pose.position.z = 0.3
        self.arm.set_pose_target(pose_goal.pose)
        plan = self.arm.plan()
        if plan:
            self.arm.execute()
            self.get_logger().info("Lifted object.")
        else:
            self.get_logger().error("Failed to lift.")

        self.get_logger().info("Auto grasp demo completed.")


def main(args=None):
    rclpy.init(args=args)
    node = AutoGraspDemo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
