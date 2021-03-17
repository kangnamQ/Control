#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from config import Config
from pid_controller import *
from pure_controller import *
from controller import Controller
import method as mt

from carmaker_node.msg import sub_udp

import sys, select, tty, termios
import math
import matplotlib.pyplot as plt


def accs(distance_new, E_state, tar_vel_, cur_vel_, cur_dt, cfg, distance_old):
	Lights_Hazard = 0
	if distance_new <= cfg.safe_distance / 2:
		cfg.state = 5
		# udp.GearNo = -9
		print("Danger! Stop!")
		Lights_Hazard = 1

	elif cfg.safe_distance / 2 < distance_new < cfg.safe_distance:
		cfg.state = 4
		print("Deceleration")

	elif cfg.safe_distance <= distance_new <= cfg.safe_distance + 5:
		cfg.state = 3
		print("Safe distance")

	elif cfg.safe_distance + 5 < distance_new < cfg.safe_distance + 15:
		cfg.state = 2
		print("Acceleration")

	try:
		cfg.acc_v = cur_vel_ + (distance_new - distance_old) / cur_dt
	except ZeroDivisionError as z:
		print(z)

	if E_state == 0:
		cfg.state = 1
		cfg.acc_v = tar_vel_
		print("Nothing front")

	if cfg.state == 0:
		cfg.acc_v = tar_vel_
	# print("distance_new",distance_new)
	distance_old_ = distance_new
	# print("distance_old_",distance_old_)

	return distance_old_, cfg, Lights_Hazard
