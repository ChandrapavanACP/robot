#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):
	def __init__(self):
		super().__init__("turtle_controller_mode")
		
		self.current_pose = Pose()
		self.goal_pose = Pose()
		
		self.target_reached = True
		
		self.cmd_vel_publisher = self.create_publisher(
			Twist, "turtle1/cmd_vel", 10)
		self.pose_subscriber = self.create_subscription(
			Pose, "turtle1/pose", self.callback_turtle_pose, 10)
		
		self.control_loop_timer_ = self.create_timer(0.1, self.control_loop)
		
	def callback_turtle_pose(self, msg):
		self.current_pose = msg
	
	def control_loop(self):
		if self.target_reached == True:
			self.goal_pose.x = float(input("Set your x goal: "))
			self.goal_pose.y = float(input("Set your y goal: "))
			self.target_reached = False
			return
		dist_x = self.goal_pose.x - self.current_pose.x
		dist_y = self.goal_pose.y - self.current_pose.y
		distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
		
		msg = Twist()
		
		if distance > 0.1:
			# position
			msg.linear.x = 2*distance
			
			# orientation
			goal_theta = math.atan2(dist_y, dist_x)
			diff = goal_theta - self.current_pose.theta
			if diff > math.pi:
				diff -= 2*math.pi
			elif diff < -math.pi:
				diff += 2*math.pi
			
			msg.angular.z = 6*diff
		else:
			# target reached!
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			self.target_reached = True
		
		self.cmd_vel_publisher.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = TurtleControllerNode()
	rclpy.spin(node)
	rclpy.shutdown()


if __name__ == "__main__":
	main()
