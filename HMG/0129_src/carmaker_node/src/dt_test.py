#!/usr/bin/env python
import numpy as np
import math

import rospy
from pid_controller import *
from pure import *
from hellocm_msgs.msg import UDP
from hellocm_msgs.msg import sub_udp
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import sys, select, tty, termios

from datetime import datetime

class Test(object):
	
	def __init__(self):
		#rospy.init_node('global_path', anonymous=True)
		self.car_path = rospy.Subscriber('/path', PoseStamped, self.dt_test)
		#self.pose = PoseStamped()
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

	def dt_test(self, data_path):
		self.path_point = data_path
		#self.last_time = rospy.Time.now()
		
		self.path_point_x = self.path_point.pose.position.x
		self.path_point_y = self.path_point.pose.position.y
		

		while not rospy.is_shutdown():
			self.last_time_rospy = rospy.Time.now()
			self.current_time_rospy = rospy.Time.now()
			
			self.last_time_date = datetime.now()
			self.current_time_date = datetime.now()
			
			self.roy_dt = (self.current_time_rospy - self.last_time_rospy).to_sec() 

			print(self.roy_dt)


if __name__ == '__main__':

	rospy.init_node('Test')
	test = Test()
	rospy.spin()
	
