#! /usr/bin/env python3

import random
import math
import threading
import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from math import atan2
import numpy as np
import sys
import argparse
import ast
import time


# define
f_comm = 250
beta = 0.5
l = 1
# l = 2*sqrt(2)*r
# r = 0.21/2


class Message:
	def __init__(self, p, wp, next_step, T, q_u, hop):
		self.p = p
		self.wp = wp
		self.next_step = next_step
		self.T = T
		self.q_u = q_u
		self.hop = hop

class Bot:

	def __init__(self, name, Q):
	
		rospy.init_node('plee')
		
		self.robot_name = name
		self.p = self.update_position()
		print(':]')
		self.wp = self.claim_waypoint()
		self.next_step = None
		self.T = random.choice(Q)
		self.q_u = random.choice(Q)
		self.hop = math.inf
		print('ehhhhh')
		
		print('/' + self.robot_name + '/cmd_vel')
		#self.pub = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size = 1)
		self.Q = Q.copy()
		self.delta_t = 2/f_comm
		self.last_check = time.perf_counter()
		self.msg_buff_t1 = [Message((3, 2), (3, 2), (3, 4), (5, 6), (3, 3), 1)]
		self.msg_buff_t3 = []
		self.i = 0
		print('uuuuuuooooouuuu')

		while not rospy.is_shutdown():
			
			t1 = threading.Thread(target = self.main_component)
			t2 = threading.Thread(target = self.broadcast_component)
			t3 = threading.Thread(target = self.goal_manager)
			t4 = threading.Thread(target = self.msg_buff_populator)
			t1.start()
			#t2.start()
			#t3.start()
			#t4.start()

	def distance_between(self, p1, p2):
		x1, y1 = p1
		x2, y2 = p2
		return abs(x1-x2) + abs(y1-y2)

	def lexigraphically_greater(self, p1, p2):
		x1, y1 = p1
		x2, y2 = p2
		if x1 > x2 or (x1 == x2 and y1 > y2):
			return True
		return False

	def update_position(self):
		print('o:')
		#rospy.init_node('pleaseeee')
		msg = rospy.wait_for_message('/robot0/odom', Odometry)
		print(':o')
		rot_q = msg.pose.pose.orientation
		roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		return (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)

	def claim_waypoint(self):
		return (round(self.p[0]), round(self.p[1]))

	def update_last_check(self):
		self.last_check = time.perf_counter()

	def msg_buff_populator(self):
		while True:
			print('whee')
			surroundings = {(self.wp[0] - l, self.wp[1]), (self.wp[0] + l, self.wp[1]), (self.wp[0], self.wp[1] - l), (self.wp[0], self.wp[1] + l)}
			for i in surroundings:
				msg = rospy.wait_for_message('position'+str(i), Message, 0.25)
				self.msg_buff_t1.append(msg)
				self.msg_buff_t3.append(msg)

	def main_component(self):
		while self.i < 55:
			surroundings = {(self.wp[0] - l, self.wp[1]), (self.wp[0] + l, self.wp[1]), (self.wp[0], self.wp[1] - l), (self.wp[0], self.wp[1] + l)}
			wait_flag = 0
			for i in surroundings:
				if self.distance_between(self.wp, self.T) >= l + self.distance_between(i, self.T):
					self.next_step = i
					break
			if self.msg_buff_t1 != []: 
				print("we in")
				msg_min = None
				min_hop = math.inf
				for msg in self.msg_buff_t1:
					if msg.hop < min_hop:
						msg_min = msg
						min_hop = msg.hop
				self.hop = 1 + min_hop
				if msg_min:
					self.q_u = msg_min.q_u
				for i in surroundings:
					for msg in self.msg_buff_t1:
						if msg.T == i:
							continue
					if i in Q:
						self.q_u = i
						self.hop = 0
						break
				
				for msg in self.msg_buff_t1:
					print("heh")
					if msg.wp == self.next_step:
						wait_flag = 1
					if msg.next_step == self.next_step and self.lexigraphically_greater(msg.p, self.p):
						wait_flag = 1
			
			if wait_flag == 0 and time.perf_counter() - self.last_check > self.delta_t and self.next_step:
				goal = self.next_step
				print(goal)
				while self.claim_waypoint() != goal:
					self.move(goal)
				self.p = self.update_position()
				self.update_last_check()
			self.msg_buff_t1 = []
			print(self.i)
			self.i += 1

	def move(self, goal):
		print("do you go here?")
		print(self.p)
		print("goalll")
		print(goal)
		r = rospy.Rate(10)
		self.p = self.update_position()
		inc_x = goal[0] - self.p[0]
		inc_y = goal[1] - self.p[1]
		speed = Twist()
		angle_to_goal = atan2(inc_y, inc_x)
		distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
		pub = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size = 1)
		
		if distance_to_goal >= 0.105:
			print("here??")
			
			print(abs(angle_to_goal))
			if abs(angle_to_goal - self.p[2]) > 0.1:
				speed.linear.x = 0.0
				speed.angular.z = 4*(angle_to_goal - self.p[2])
			else:
				print("what about this one")
				speed.linear.x = 1.1 * distance_to_goal
				speed.angular.z = 0.0
			
			pub.publish(speed)
			r.sleep()
		#	self.p = self.update_position()
		#	inc_x = goal[0] - self.p[0]
		#	inc_y = goal[1] - self.p[1]
		#	angle_to_goal = atan2(inc_y, inc_x)
		#	distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
		#speed.linear.x = 0.0
		#speed.angular.z = 0.0
		#pub.publish(speed)
		
		



	def broadcast_component(self):
		while True:
			pub = rospy.Publisher('position' + str(self.wp), Message, queue_size=8)
			rospy.init_node(self.robot_name + 'talker', anonymous=True)
			rate = rospy.Rate(4)
			while not rospy.is_shutdown():
				msg = Message(self.p, self.wp, self.next_step, self.T, self.q_u, self.hop)
				rospy.loginfo(msg)
				pub.publish(msg)
				rate.sleep()
				
			print('eek')
			
	def exchange(self):
		return True

	def goal_manager(self):
		while True:
			if self.msg_buff:
				message_recieved = self.msg_buff_t3.pop(0) 
				if message_recieved.T == self.T:
					if self.lexigraphically_greater(message_recieved.p, self.p):
						if random.uniform(0, 1.0) > 0.1:
							self.T = self.q_u
							self.update_last_check()
						else:
							self.T = random.choose(Q)
							self.update_last_check()
				if self.distance_between(self.p, message_recieved.T) < self.distance_between(self.p, self.T):
					if self.exchange(): #define
						self.T = message_recieved.T
						self.update_last_check()
				if self.distance_between(self.p, message_recieved.T) == self.distance_between(self.p, self.T):
					if random.uniform(0, 1.0) < beta:
						if self.exchange(): #define
							self.T = message_recieved.T
							self.update_last_check()
			print('meeeep')


if __name__ == '__main__':

	print('hi')
	myargs = rospy.myargv(argv = sys.argv)
	name = myargs[1]
	Q = []
	for string in myargs[2:]:
		Q.append(ast.literal_eval(string))
	
	Bot(name, Q)


