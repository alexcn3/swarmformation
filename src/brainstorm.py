#!/usr/bin/env python

import random
import math
import threading
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# define
f_comm = 1
beta = 0.5
l = 1
# l = 2*sqrt(2)*r
# r = 0.21/2


class Message:
	def __init__(self, pos, wp, next_step, T, q_u, hop):
		self.pos = pos
		self.wp = wp
		self.next = next_step
		self.T = T
		self.q_u = q_u
		self.hop = hop

class Bot:

	def __init__(self, wp, Q):
		self.wp = wp
		self.T = random.choose(Q)
		self.hop = math.inf
		self.q_u = random.choose(Q)
		self.Q = Q.copy()
		self.next_step = wp
		self.last_check = last_check
		self.delta_t = 2/f_comm
		self.last_check = clock()
		self.msg_buff = []

		self.t1 = threading.Thread(target = self.main_component)
		self.t2 = threading.Thread(target = self.broadcast_component)
		self.t3 = threading.Thread(target = self.goal_manager)
		self.t1.start()
		self.t2.start()
		self.t3.start()

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

	def update_last_check(self):
		self.last_check = clock()

	def main_component(self):
		while True:
			surroundings = {(self.wp[0] - l, self.wp[1]), (self.wp[0] + l, self.wp[1]), (self.wp[0], self.wp[1] - l), (self.wp[0], self.wp[1] + l)}
			wait_flag = 0
			for i in surroundings:
				if self.distance_between(self.wp, self.T) > self.distance_between(self.wp, i) + self.distance_between(i, goal):
					self.next_step = i
					break
			if self.msg_buff:
				msg_min = None
				min_hop = math.inf
				for msg in self.msg_buff:
					if msg.hop < min_hop:
						msg_min = msg
						min_hop = msg.hop
				self.hop = 1 + min_hop
				self.q_u = msg_min.q_u
				for i in surroundings:
					for msg in msg_buff:
						if msg.T == i:
							continue
					if i in Q:
						self.q_u = i
						self.hop = 0
						break
				for msg in msg_buff:
					if msg.wp == self.next_step:
						wait_flag = 1
					if msg.next_step == next_step and self.lexigraphically_greater(msg.p, p):
						wait_flag = 1
				if wait_flag == 0 and clock() - self.last_check > self.delta_t:
					goal = (None, None)
					if next_step[0] == wp[0]:
						goal = ('y', self.next_step[1] - self.wp[1])
					else:
						goal = ('x', self.next_step[0] - self.wp[0])
					self.wp = self.next_step
					self.move(goal)
					self.update_last_check()
				# else:
				# 	stay() # define

	def move(self, goal):
		client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	    client.wait_for_server()

	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = "map"
	    goal.target_pose.header.stamp = rospy.Time.now()
	    if goal[0] == 'x':
	    	goal.target_pose.pose.position.x = goal[1]
	    else:
	    	goal.target_pose.pose.position.y = goal[1]
	    goal.target_pose.pose.orientation.w = 1.0

	    client.send_goal(goal)
	    wait = client.wait_for_result()
	    if not wait:
	        rospy.logerr("Action server not available!")
	        rospy.signal_shutdown("Action server not available!")
	    else:
	        return client.get_result()


	def broadcast_component(self):
		while True:
			pub = rospy.Publisher('position' + str(self.wp), Message, queue_size=12)
   		    rospy.init_node('talker', anonymous=True)
   		    rate = rospy.Rate(2)
   		    while not rospy.is_shutdown():
   		    	msg = Message(self.p, self.wp, self.next_step, self.T, self.q_u, self.hop)
   		    	rospy.loginfo(msg)
   		    	pub.publish(msg)
   		    	rate.sleep()

   	def exchange(self):
   		return True

	def goal_manager(self):
		while True:
			if self.msg_buff:
				message_recieved = self.msg_buff[0] # define
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


if __name__ == '__main__':

	for i in range(4):
		Bot((0, i), [(-1, 1), (1, 1), (-1, -1), (1, -1)])


