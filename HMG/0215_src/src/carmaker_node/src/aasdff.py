#!/usr/bin/env python
import rospy
import os
import numpy as np
import math


class Config():
	def __init__(self):
		self.DATAROOT = "/home/kang/catkin_ws/src/carmaker_node"
		self.SRCPATH = os.path.join(self.DATAROOT, "src")
		self.P_PATH = os.path.join(self.SRCPATH, "path")
		self.USEPATH = os.path.join(self.P_PATH, self.num_file(1))


	def num_file(self, num):

		if num == 0:
			file = "test.csv"

		elif num == 1:
			file = "scenario1_TM.csv"

		elif num == 2:
			file = "scenario2_TM.csv"

		elif num == 3:
			file = "scenario3_TM.csv"

		elif num == 4:
			file = "scenario4_TM.csv"

		elif num == 5:
			file = "scenario1_300_TM.csv"

		elif num == 6:
			file = "scenario1_fixed4.5.csv"

		else:
			file = ""

		return file
