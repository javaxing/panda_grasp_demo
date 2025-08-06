#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time

from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, moveit_commander

class AutoGraspDemo(Node):
    def __init__(self):
        super().__init__('auto_grasp_demo')
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("panda_arm")

        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)

        time.sleep(2.0)  # 等待 scene 初始化

        self.run_sequence()

    def add_box(self, box_name="target_box"):
        box_pose = PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.1
        self.scene.add_box(box_name, box_pose, size=(0.04, 0.04, 0.04))
        self.get_logger().info(f"Added box '{box_name}'.")

    def run_sequence(self):
        self.get_logger().info("Starting auto grasp demo...")

        # 添加物体
        box_name = "target_box"
        self.add_box(box_name)
        time.sleep(1.0)

        # 预抓取姿态（靠近物体）
        pose_goal = self.arm_group.get_current_pose().pose
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.2  # 上方 10cm
        self.arm_group.set_pose_target(pose_goal)

        success, plan, _, _ = self.arm_group.plan()
        if success:
            self.arm_group.go(wait=True)
            self.get_logger().info("Moved to pre-grasp position.")
        else:
            self.get_logger().error("Planning to pre-grasp failed.")
            return

        # 模拟抓取动作（仅位移）
        pose_goal.position.z = 0.1  # 下移接近物体
        self.arm_group.set_pose_target(pose_goal)
        success, plan, _, _ = self.arm_group.plan()
        if success:
            self.arm_group.go(wait=True)
            self.get_logger().info("Approached object.")
        else:
            self.get_logger().error("Failed to approach object.")
            return

        # 模拟闭合夹爪（这里可以调用夹爪控制）
        self.get_logger().info("Simulating gripper close...")

        # 抬起
        pose_goal.position.z = 0.3  # 抬高
        self.arm_group.set_pose_target(pose_goal)
        success, plan, _, _ = self.arm_group.plan()
        if success:
            self.arm_group.go(wait=True)
            self.get_logger().info("Lifted object.")
        else:
            self.get_logger().error("Failed to lift object.")
            return

        self.get_logger().info("Auto grasp demo completed.")

def main(args=None):
    rclpy.init(args=args)
    node = AutoGraspDemo()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
