#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import numpy as np

x = 0.0
y = 0.0
theta = 0.0

def test(msg):
	global x
	global y
	global theta
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
rospy.init_node ("speed_controller")

sub = rospy.Subscriber("/robot0/odom", Odometry, test)
pub = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size = 1)
speed = Twist()

r = rospy.Rate(10)

path_list = [(4,4), (4,2), (2,2), (2, 4)]
point_index = 0
goal = Point()

while not rospy.is_shutdown():
	if point_index < len(path_list):
		goal.x = path_list[point_index][0]
		goal.y = path_list[point_index][1]
	else:
		break
	inc_x = goal.x - x
	inc_y = goal.y - y
	
	angle_to_goal = atan2 (inc_y, inc_x)
	
	distance_2_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
	
	if distance_2_goal >= 0.105:
		if abs(angle_to_goal - theta) > 0.1:
			speed.linear.x = 0.0
			speed.angular.z = 4*(angle_to_goal - theta)
		else:
			speed.linear.x = 1.1 * distance_2_goal
			speed.angular.z = 0.0
		pub.publish(speed)
		
	else:
		point_index += 1
		point_index = point_index % 4
	r.sleep()
