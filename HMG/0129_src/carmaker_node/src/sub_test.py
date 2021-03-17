#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import sys, select, tty, termios
import numpy as np
import math
import pandas as pd


class Controller(object):
	def __init__(self):
		self.car_path = rospy.Subscriber('/path', Path, self.path_callback)
		# self.car_path = None

		self.sw = False
		self.path_point = None
		self.seq = 0
		self.path_point_dict = {'index':[],
								'x':[],
								'y':[]}


	def path_callback(self, data_path):
		self.path_point = data_path
		self.seq = self.path_point.header.seq
		self.path_point_dict['index'].append(self.seq)
		self.path_point_dict['x'].append(self.path_point.poses[self.seq-1].pose.position.x)
		self.path_point_dict['y'].append(self.path_point.poses[self.seq-1].pose.position.y)

		# if data_path.header.seq != len(self.path_point_dict['index']):
		# 	self.path_point_dict['index'].append(len(self.path_point_dict['index']))
		# 	self.path_point_dict['x'].append(self.path_point.poses[len(self.path_point_dict['index'])].pose.position.x)
		# 	self.path_point_dict['y'].append(self.path_point.poses[len(self.path_point_dict['index'])].pose.position.y)
		# print(self.seq)


def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('controller')
	controller = Controller()

	try:
		while True:
			key = getKey()

			if key == 'p':
				controller.sw = False
				print("1 time")

			try:

				# while not controller.sw:
				# 	controller.car_path = rospy.Subscriber('/path', Path, controller.path_callback)
				# 	controller.sw = True

				print("index : ", len(controller.path_point_dict['index']),
					  "x : ", len(controller.path_point_dict['x']),
					  "y : ", len(controller.path_point_dict['y']),
					  "seq : ", controller.seq)


			except AttributeError:
				pass

			else:
				if (key == '\x03'):
					# pub_start.publish(0)
					break

	except rospy.ROSInterruptException:
		pass
