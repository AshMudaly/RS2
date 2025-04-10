#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander import MoveGroupCommander
from moveit_commander.robot_trajectory import RobotTrajectory
from geometry_msgs.msg import PoseStamped

class UR3eMoveToJoint(Node):
    def __init__(self):
        super().__init__('ur3e_move_to_joint')
        
        # Initialize move group commander for the UR3e
        self.move_group = MoveGroupCommander("ur3e_arm")

        # Define the desired joint angles (replace with desired angles)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0.5  # Example angle for joint 1
        joint_goal[1] = -0.5  # Example angle for joint 2
        joint_goal[2] = 0.0  # Example angle for joint 3
        joint_goal[3] = -1.0  # Example angle for joint 4
        joint_goal[4] = 1.0  # Example angle for joint 5
        joint_goal[5] = 0.5  # Example angle for joint 6

        # Move the arm to the joint goal
        self.move_group.go(joint_goal, wait=True)

        # Check if the goal was reached
        if self.move_group.plan() == "success":
            self.get_logger().info("Move successful!")
        else:
            self.get_logger().error("Move failed.")

def main(args=None):
    rclpy.init(args=args)
    ur3e_move = UR3eMoveToJoint()
    rclpy.spin(ur3e_move)
    ur3e_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# This script initializes a ROS2 node that moves a UR3e robot arm to a specified joint configuration.
# It uses the MoveIt Commander library to interface with the robot's motion planning capabilities.
# The joint angles should be replaced with the desired values for your specific application.