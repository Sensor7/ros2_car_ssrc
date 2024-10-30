#!/usr/bin/env python3
# coding=utf-8

import rclpy
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import String
from math import copysign
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

#from dynamic_reconfigure.server import Server
#from simple_follower.cfg import arPIDConfig


class ArFollower(Node):
	def __init__(self):
		
		super().__init__('arfollower')
		
		#参数初始化
		self.linearfront_p = 0.3
		self.linearback_p = 0.6
		self.angularleft_p = 3.0
		self.angularright_p =3.0
		self.d_param = 0.8
		
		self.max_angular_speed =0.5
		self.min_angular_speed =-0.5
		self.max_linear_speed =0.2
		self.min_linear_speed =-0.2
		self.goal_x =0.6
		self.goal_y =0.0
				
		#订阅AR标签位姿信息，发布速度话题
		qos = QoSProfile(depth=10)
		self.cmdvelpublisher=self.create_publisher(Twist,'/cmd_vel',qos)
		self.arposesubscriber=self.create_subscription(
			Marker,
			'/aruco_single/marker',
			self.set_cmd_vel,
			qos)					

		self.move_cmd = Twist()
		self.move_cmd.linear.x = 0.0
		self.move_cmd.angular.z = 0.0
		

	def set_cmd_vel(self, msg):
				
		offset_y = 0.3 #小车中心与摄像头检测到的AR标签中心的偏差
		target_offset_y = msg.pose.position.x - offset_y #AR标签位姿信息x方向(已校正)-对应ROS中y方向
		target_offset_x = msg.pose.position.z #AR标签位姿信息z方向-对应ROS中x方向		

		#当AR标签和小车的距离与设定距离存在偏差时
		if target_offset_x > self.goal_x:
			linearspeed = target_offset_x * self.linearfront_p 
			if linearspeed < 0.01:
				linearspeed = 0.0
				#极低速置零，避免小车摇摆
			if linearspeed > self.max_linear_speed:
				linearspeed = self.max_linear_speed
				#速度限幅	
			self.move_cmd.linear.x = linearspeed
			#当AR标签中心与小车中心存在偏差时
			if target_offset_y > self.goal_y: 
				angularspeed = target_offset_y * self.angularleft_p
				if angularspeed < 0.01:
					angularspeed = 0.0
					#极低速置零，避免小车摇摆
				if angularspeed > self.max_angular_speed:
					angularspeed = self.max_angular_speed
					#速度限幅
				self.move_cmd.angular.z = -angularspeed
			else:
				angularspeed = target_offset_y * self.angularright_p
				if abs(angularspeed) < 0.01:
					angularspeed = 0.0
				if abs(angularspeed) > self.max_angular_speed:
					angularspeed = -self.max_angular_speed
				self.move_cmd.angular.z = -angularspeed
		else:
			linearspeed = (target_offset_x - self.goal_x) * self.linearback_p
			if abs(linearspeed) < 0.01:
				linearspeed = 0.0
			if linearspeed > self.max_linear_speed:
				linearspeed = -self.max_linear_speed
			self.move_cmd.linear.x = linearspeed
			#当AR标签中心与小车中心存在偏差时
			if target_offset_y > self.goal_y: 
				angularspeed = target_offset_y * self.angularleft_p
				if angularspeed < 0.01:
					angularspeed = 0.0
					#极低速置零，避免小车摇摆
				if angularspeed > self.max_angular_speed:
					angularspeed = self.max_angular_speed
					#速度限幅
				self.move_cmd.angular.z = -angularspeed
			else:
				angularspeed = target_offset_y * self.angularright_p
				if abs(angularspeed) < 0.01:
					angularspeed = 0.0
				if abs(angularspeed) > self.max_angular_speed:
					angularspeed = -self.max_angular_speed
				self.move_cmd.angular.z = -angularspeed
		
		self.cmdvelpublisher.publish(self.move_cmd)
	

def main(args=None):
	
	rclpy.init(args=args)
	print('ar following start!')
	arfollower=ArFollower()
	try:
		rclpy.spin(arfollower)
	except:
		ArFollower.destroy_node()
		rclpy.shutdown()
		

if __name__ == '__main__':
    main()

