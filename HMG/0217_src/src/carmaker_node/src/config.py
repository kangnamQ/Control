#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import os.path as op

class Config():
	def __init__(self):
		self.DATAROOT = "/home/kang/catkin_ws/src/carmaker_node"
		self.SRCPATH = os.path.join(self.DATAROOT, "src")
		self.P_PATH = os.path.join(self.SRCPATH, "path")

		# select test file
		self.USEPATH = os.path.join(self.P_PATH, self.num_file(12))

		# option
		self.safe_distance = 30
		self.set_distance = 50
		self.state = 0
		self.acc_v = 0
		self.acc_go_gain = 0.2
		self.acc_stop_gain = 1

		# option
		self.mode = None

		# draw option
		# self.show_animation = True
		self.show_animation = False

		# self.show_animation_gain = True
		self.show_animation_gain = False

		self.show_print = True
		# self.show_print = False

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

		elif num == 7:
			file = "scenario1_100_TM_deleted.csv"
		elif num == 8:
			file = "scenario2_100_TM_deleted.csv"
		elif num == 9:
			file = "scenario3_100_TM_deleted.csv"
		elif num == 10:
			file = "scenario4_100_TM_deleted.csv"

		elif num == 11:
			file = "fxp_s1_TM.csv"
		elif num == 12:
			file = "fxp_s2_TM.csv"
		elif num == 13:
			file = "fxp_s3_TM.csv"
		elif num == 14:
			file = "fxp_s4_TM.csv"

		else:
			file = ""
		return file
