#!/usr/bin/env python

import rospy
from carmaker_node.msg import sub_udp
import numpy as np
import method as mt


def get_angle(c_i):
	global_path = mt.get_csv()

	try:
		pt1 = (global_path['x'][c_i+1] - global_path['x'][c_i], global_path['y'][c_i+1] - global_path['y'][c_i])
		pt2 = (global_path['x'][c_i+15] - global_path['x'][c_i], global_path['y'][c_i+15] - global_path['y'][c_i])
		ang1 = np.arctan2(*pt1[::-1])
		ang2 = np.arctan2(*pt2[::-1])
		res = np.rad2deg((ang1 - ang2) % (2 * np.pi))
		res = (res + 360) % 360
	except IndexError:
		res = 0

	if 300 < res < 355:
		direction = 'left'
		Lights_Indicator = 1
	elif 10 < res < 60:
		direction = 'right'
		Lights_Indicator = -1
	else:
		direction = None
		Lights_Indicator = 0

	return direction, Lights_Indicator
