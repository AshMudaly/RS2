#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

class TrollMovement(Node):  # Rename class to match file purpose
    def __init__(self):
        super().__init__('trollmovement')  # Use 'trollmovement' for the node name
        
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
    troll_move = TrollMovement()  # Match the class name here
    rclpy.spin(troll_move)
    troll_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
