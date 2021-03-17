#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import os.path as op

# from .config import Config
from pd_controller import *
from offset_pd_controller import *

from pure_controller import *
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from carmaker_node.msg import UDP, sub_udp, localization

import tf

from carmaker_node.msg import lane_center
# from lane_detection.msg import lane_center

from carmaker_tracking.msg import emergency_state, Front_vehicle_state

import sys, select, tty, termios
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt


class Config():
	def __init__(self):
		self.DATAROOT = "/home/kang/catkin_ws/src/carmaker_node"
		self.SRCPATH = os.path.join(self.DATAROOT, "src")
		self.P_PATH = os.path.join(self.SRCPATH, "path")

		# select test file
		self.USEPATH = os.path.join(self.P_PATH, self.num_file(11))

		# option
		self.safe_distance = 30
		self.set_distance = 50
		self.state = 0
		self.acc_v = 0
		self.acc_go_gain = 0.2
		self.acc_stop_gain = 1

		# draw option
		# self.show_animation = True
		self.show_animation = False

		# self.show_animation_gain = True
		self.show_animation_gain = False

		self.show_print = True

	# self.show_print = False

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


class Controller(object):
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
		self.car_pos = rospy.Subscriber('/odom', Odometry, self.pure_callback)
		self.car_local = rospy.Subscriber('/localization', localization, self.local_callback)

		self.car_lane = rospy.Subscriber('/lane_center_information', lane_center, self.lane_callback)
		self.car_e_stop = rospy.Subscriber('/Emergency_state', emergency_state, self.e_stop_callback)
		self.car_front = rospy.Subscriber('/Front_vehicle_information', Front_vehicle_state, self.Front_callback)

		self.traffic = rospy.Subscriber('/traffic_light', Int32, self.traffic_callback)

		self.loop = 0
		self.car_now_time = 0
		self.car_lasted_time = 0
		self.car_dt_time = 0

	def udp_callback(self, data_udp):
		self.cur_vel = float(data_udp.vx * 3.6)  # m/s -> km/h
		self.cur_yaw = float(data_udp.yaw)
		self.car_now_time = data_udp.header.stamp.to_sec()
		self.car_acc = float(data_udp.ax)

	def pure_callback(self, data_pure):
		self.car_point_x = data_pure.pose.pose.position.x
		self.car_point_y = data_pure.pose.pose.position.y

	def local_callback(self, data_local):
		self.car_local_point_x = data_local.x
		self.car_local_point_y = data_local.y
		self.car_local_point_theta = data_local.theta

	def lane_callback(self, lane_state):
		self.lane_center_pix = lane_state.center_pix
		self.lane_state_offset = lane_state.offset_

	# print(self.lane_center_pix)

	def e_stop_callback(self, e_state):
		self.emergency_state_info = e_state.state

	# self.emergency_state == 0  60 < distance
	# self.emergency_state == 1  30 < distance < 60
	# self.emergency_state == 2  30 < distance

	def Front_callback(self, Front_state):
		self.Front_car_x = Front_state.x
		self.Front_car_y = Front_state.y
		self.Front_car_dis = Front_state.distance
		self.Front_car_int = Front_state.intensity

	def traffic_callback(self, traffic_state):
		self.traffic_stat = traffic_state


"""
1*3.6m/s = 3.6km/h
tar_vel = km/h
cur_vel = m/s
cur_err = tar_vel - cur_vel*3.6 = km/h
"""


def get_csv():
	# Global path info
	cfg = Config()
	df_TM = pd.read_csv(cfg.USEPATH)

	df_tf = {}
	df_dic = {}
	# key_name = ['0', '1']
	key_name = ['x', 'y']
	for key in key_name:
		df_dic[key] = []
		df_tf[key] = []

	# df_tf['x']=df_TM['0'].to_dict()
	df_tf['x'] = df_TM['x'].to_dict()
	# df_tf['y']=df_TM['1'].to_dict()
	df_tf['y'] = df_TM['y'].to_dict()

	df_dic['x'] = df_tf['x'].values()
	df_dic['y'] = df_tf['y'].values()

	return df_dic


def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def make_dic():
	break_point_test = {}

	key_name = ['start_x', 'start_y', 'goal_x', 'goal_y', 'd_x', 'd_y', 'cal_distance']
	for key in key_name:
		break_point_test[key] = []

	return break_point_test


def main():
	global settings
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('controller')
	controller = Controller()
	PID_controller = Pid_controller()
	cfg = Config()
	udp = sub_udp()
	OffSet_controller = Offset_controller()

	udp.GearNo = 1
	udp.Ax = 0
	udp.SteeringWheel = 0
	cur_vel_ = 0
	tar_vel_ = 0
	udp.VC_SwitchOn = 1
	udp.GearNo = 1
	controller.loop = 0

	pose_dict = get_csv()
	cx = pose_dict['x']
	cy = pose_dict['y']

	target_course = TargetCourse(cx, cy)
	states = States()

	# get point distance
	test_breaking = make_dic()
	stop_test_sw = False
	stop_test_sw_info = 0
	v_dt = 0
	distance_old = 0

	# Use TF
	# listener = tf.TransformListener()

	# start -----------------------------------------------

	while 1:
		# Use TF
		# try:
		# 	(trans, rot) = listener.lookupTransform('/odom', 'Wheel_Rear', rospy.Time(0))
		# 	print(trans)
		# 	print(rot)
		#
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	print("aaa")
		# 	continue
		key = getKey()
		# cal dt
		if controller.car_lasted_time == 0:
			print("none test")
			controller.car_lasted_time = controller.car_now_time
			print(controller.car_lasted_time)
			print(controller.car_now_time)
		# print("time_now : ", controller.car_now_time)
		# print("time_lasted : ", controller.car_lasted_time)
		controller.car_dt_time = controller.car_now_time - controller.car_lasted_time
		controller.car_lasted_time = controller.car_now_time
		# print("time_dt : ", controller.car_dt_time)

		# set target target V

		if key == 't':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			try:
				tar_vel_ = float(input("target velue : "))
				print("set", tar_vel_, "km/h")
			except NameError:
				print("try again")
			except SyntaxError:
				print("try again")

		try:
			# use odom
			# state = State(x=controller.car_point_x,
			# 			  y=controller.car_point_y,
			# 			  yaw=controller.cur_yaw,
			# 			  v=cur_vel_)

			# use localization
			state = State(x=controller.car_local_point_x,
						  y=controller.car_local_point_y,
						  yaw=controller.car_local_point_theta,
						  v=controller.cur_vel)

			lastIndex = len(cx) - 1
			target_ind, Lf, Ld = target_course.search_target_index(state)
			# print(target_ind)

			# Loop in index, untill last index
			if lastIndex > target_ind:
				cur_vel_ = controller.cur_vel
				cur_dt = controller.car_dt_time
				cur_acc_ = controller.car_acc
				# try:
				# 	safe_offset = 0.32
				# 	off_string = OffSet_controller.accel_control(safe_offset,
				# 												 controller.lane_state_offset,
				# 												 cur_dt)
				# 	udp.SteeringWheel = off_string
				# except AttributeError as p:
				# 	print("no offset")

				if controller.traffic_stat == 1:
					udp.GearNo = -9
				else:
					udp.GearNo = 1

				try:
					distance_new = controller.Front_car_dis
					E_state = controller.emergency_state_info
					if cfg.state == 0:
						cfg.acc_v = tar_vel_
						mov_vel = PID_controller.accel_control(cfg.acc_v, cur_vel_, cur_dt)

					if distance_new <= cfg.safe_distance / 2:
						cfg.state = 5
						# udp.GearNo = -9
						print("Danger! Stop!")
						udp.Lights_Hazard = 1

					elif cfg.safe_distance / 2 < distance_new < cfg.safe_distance:
						cfg.state = 4
						print("Deceleration")
						udp.Lights_Hazard = 0

					elif cfg.safe_distance <= distance_new <= cfg.safe_distance + 5:
						cfg.state = 3
						print("Safe distance")
						udp.Lights_Hazard = 0

					elif cfg.safe_distance + 5 < distance_new < cfg.safe_distance + 15:
						cfg.state = 2
						print("Acceleration")
						udp.Lights_Hazard = 0

					# else:
					# 	cfg.state = 1
					# 	cfg.acc_v = tar_vel_
					# 	print("Nothing front")
					# cfg.acc_v = math.sqrt((cur_vel_ ** 2) + 2 * cur_acc_ * (cfg.safe_distance - distance))

					try:
						cfg.acc_v = cur_vel_ + (distance_new - distance_old) / cur_dt
					except ZeroDivisionError as z:
						print(z)

					if controller.emergency_state_info == 0:
						cfg.state = 1
						cfg.acc_v = tar_vel_
						print("Nothing front")

					# pd controller
					mov_vel = PID_controller.accel_control(cfg.acc_v, cur_vel_, cur_dt)
					udp.Ax = mov_vel  # m/s^2
					print("acc_vel : ", cfg.acc_v)

					distance_old = distance_new

				except AttributeError:
					# Nothing front
					# pd controller
					mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_, cur_dt)
					udp.Ax = mov_vel  # m/s^2
					cfg.state = 0
					print("Nothing--------------------------")

				try:
					top_pix, low_pix, mid_pix = controller.lane_center_pix

					t_l_dx = top_pix - low_pix
					t_l_dy = 0 - 800
					t_l_alpha = math.atan2(t_l_dy, t_l_dx)

					m_l_dx = mid_pix - low_pix
					m_l_dy = 400 - 800
					m_l_alpha = math.atan2(m_l_dy, m_l_dx)

					# rad
					pix_alpha = t_l_alpha - m_l_alpha
					# deg
					pix_alpha_d = pix_alpha * (180 / math.pi)

					print("pix_alpha : ", pix_alpha)
					print("pix_alpha_d : ", pix_alpha_d)

					if -1.5 <= pix_alpha_d <= 1.5:
						print("############################################3")
						# pix_theta = math.pi/2 - t_l_alpha

						if -0.05 <= t_l_dx <= 0.05:
							udp.SteeringWheel = 0

						else:
							udp.SteeringWheel = t_l_dx / -1000

					else:
						# # steering wheel ang! = deg
						delta, target_ind, delta_rad, delta_gain, alpha, alpha_m, car_alpha_m, delta_m, Ld_m = pure_pursuit_steer_control(
							state, target_course, target_ind)
						udp.SteeringWheel = delta_gain

				except AttributeError as k:
					print(k)

				v_dt += cur_dt
				# state.update(mov_vel, delta_rad, cur_dt)
				states.append(v_dt, state)

				# try:
				# 	if controller.emergency_state_info == 2:
				# 		udp.VC_SwitchOn = 1
				# 		udp.GearNo = -9
				# 		print('stop - 1')
				# 	elif controller.emergency_state_info == 0:
				# 		udp.VC_SwitchOn = 1
				# 		udp.GearNo = 1
				# 		print('go!')
				#
				# except AttributeError as m:
				# 	print("not yet E_state!")
				# 	pass

				# drawing plot
				if cfg.show_animation:  # pragma: no cover
					plt.cla()
					# for stopping simulation with the esc key.

					plt.gcf().canvas.mpl_connect(
						'key_release_event',
						lambda event: [exit(0) if event.key == 'escape' else None])
					plot_arrow(state.x, state.y, state.yaw)
					plt.plot(cx, cy, "-r", label="course")
					plt.plot(states.x, states.y, "-b", label="trajectory")
					plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
					plt.axis("equal")
					plt.grid(True)
					plt.title("Speed[km/h]:" + str(state.v)[:4])
					plt.pause(0.001)

				if cfg.show_animation_gain:
					plt.cla()
					plt.gcf().canvas.mpl_connect(
						'key_release_event',
						lambda event: [exit(0) if event.key == 'escape' else None])
					plt.grid(True)
					plt.plot(states.t, [iv for iv in states.v], "-r")
					plt.legend()
					plt.xlabel("Time[s]")
					plt.ylabel("Speed[km/h]")
					plt.axis("equal")
					plt.title("Speed[km/h]:" + str(state.v)[:4])
					plt.grid(True)
					plt.pause(0.00001)

				# loop check
				controller.loop += 1
				# --------------------------------------------------------------------
				# printing line
				if cfg.show_print:
					# print("*-----s-t-a-r-t---l-i-n-e-----*")
					print("loop : ", controller.loop)
					# print("-----p-o-i-n-t---l-i-n-e-----")
					# print("target_ind : ", target_ind)
					# print("target_x : ", cx[target_ind])
					# print("target_y : ", cy[target_ind])
					# print("car_pint_x : ", controller.car_local_point_x)
					# print("car_pint_y : ", controller.car_local_point_y)
					# print("rear_x", state.rear_x)
					# print("rear_y", state.rear_y)
					# print("yaw", state.yaw)
					# print("v", state.v)
					print("-----v-a-l-u-e---l-i-n-e-----")
					print("tar_vel : ", tar_vel_)
					print("cur_vel : ", cur_vel_)
					print("dt : ", cur_dt)
					# print("mov_vel : ", mov_vel)
					# print("steeringWeel : ", delta)
					# print("Lf : ", Lf)
					# print("Ld : ", Ld)
					# print("-----U-D-P----l-i-n-e-----")
					# print("udp.VC switch : ", udp.VC_SwitchOn)
					# print("udp.Ax : ", udp.Ax)
					print("udp.SteeringWheel : ", udp.SteeringWheel)
					# print("udp.GearNo : ", udp.GearNo)
					# print("*-*--*  cal  *--*-*")
					# print("delta : ", delta)
					# print("delta_m() : ", delta_m)
					# print("delta_gain() : ", delta_gain)
					# print("alpha : ", alpha)
					# print("alpha_m() : ", alpha_m)
					# print("car_alpha_m() : ", car_alpha_m)
					# print("Ld : ", Ld)
					# print("Ld_m : ", Ld_m)
					# print("Lf : ", Lf)
					# print("ind : ", target_ind)

					# print("----- test point -----")
					# print("goal_x : ", test_breaking['goal_x'])
					# print("start_x : ", test_breaking['start_x'])
					# print("goal_y : ", test_breaking['goal_y'])
					# print("start_y : ", test_breaking['start_y'])
					# print("d_x : ", test_breaking['d_x'])
					# print("d_y : ", test_breaking['d_y'])
					# print("cal_distance : ", test_breaking['cal_distance'])
					# print("")
					# print("goal_x len : ", len(test_breaking['goal_x']))
					# print("start_x len : ", len(test_breaking['start_x']))
					# print("goal_y len : ", len(test_breaking['goal_y']))
					# print("start_y len : ", len(test_breaking['start_y']))
					# print("d_x len : ", len(test_breaking['d_x']))
					# print("d_y len : ", len(test_breaking['d_y']))
					# print("stop_test_sw_info : ", stop_test_sw_info)

					print("*-*--*  show you  *--*-*")
					try:
						# print("lane_state_curve len : ", (controller.lane_center_pix))
						# print("lane_state_offset : ", controller.lane_state_offset)

						# print("emergency_state : ", controller.emergency_state_info)
						# print("Front_car_x : ", controller.Front_car_x)
						# print("Front_car_y : ", controller.Front_car_y)
						print("Front_car_dis : ", controller.Front_car_dis)
						# print("Front_car_int : ", controller.Front_car_int)
						print("cfg.state : ", cfg.state)
						print("cfg.acc_v : ", cfg.acc_v)

					except AttributeError:
						print("not yet!")

					print("*-----e-n-d-----l-i-n-e-----*")

				# use point cal_distance
				if stop_test_sw == True:
					stop_test_sw_info += 1
					dx = None
					dy = None
					cal_d = None
					test_breaking = make_dic()
					stop_test_sw = False

			else:
				print("lastIndex < target_ind")
				print("*-*-*-*-*-*-END-*-*-*-*-*-*")

		except AttributeError as e:
			print(e)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue
		except IndexError:
			print("!")

		# ----------------------------------------------------

		# use point cal_distance
		if key == 'q':
			udp.VC_SwitchOn = 1
			udp.GearNo = -9
			print('breaking point start')
			test_breaking['start_x'].append(controller.car_local_point_x)
			test_breaking['start_y'].append(controller.car_local_point_y)

		if key == 'w':
			udp.VC_SwitchOn = 1
			udp.GearNo = -9
			print('breaking point check!!')
			test_breaking['goal_x'].append(controller.car_local_point_x)
			test_breaking['goal_y'].append(controller.car_local_point_y)

		if key == 'e':
			udp.VC_SwitchOn = 1
			udp.GearNo = -9
			print('test cal_distance')
			# dx=[]
			# dy=[]
			try:
				for i in range(len(test_breaking['start_x'])):
					dx = test_breaking['goal_x'][i] - test_breaking['start_x'][i]
					test_breaking['d_x'].append(dx)
				for j in range(len(test_breaking['start_y'])):
					dy = test_breaking['goal_y'][j] - test_breaking['start_y'][j]
					test_breaking['d_y'].append(dy)

				if len(test_breaking['start_x']) == len(test_breaking['goal_x']) and len(
						test_breaking['start_y']) == len(test_breaking['goal_y']):
					for k in range(len(test_breaking['start_x'])):
						cal_d = math.sqrt((test_breaking['d_y'][k] ** 2) + (test_breaking['d_x'][k] ** 2))
						test_breaking['cal_distance'].append(cal_d)
			except IndexError:
				print("Error!!")
				stop_test_sw = True

		if key == 'r':
			udp.VC_SwitchOn = 1
			udp.GearNo = -9
			print('saving data')
			try:
				df_bp = pd.DataFrame(test_breaking)
				df_bp.to_csv('~/catkin_ws/src/carmaker_node/src/test_break/test_breaking.csv',
							 sep=',', na_rep='NaN')
			except ValueError:
				print("Error!!")
				stop_test_sw = True

		# set target V
		if key == '0':
			udp.VC_SwitchOn = 0
			print('maneuver')

		if key == '1':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 10

		if key == '2':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 20

		if key == '3':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 30

		if key == '4':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 40

		if key == '5':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 50

		# Brake
		if key == 'c':
			udp.VC_SwitchOn = 1
			udp.GearNo = 0
			udp.Ax = 0
			udp.SteeringWheel = 0
			print('stop - 0.5')

		if key == 'x':
			udp.VC_SwitchOn = 1
			udp.GearNo = -9
			udp.Ax = 0
			udp.SteeringWheel = 0
			print('stop - 1')

		else:
			if (key == '\x03'):
				break

		controller.car_pub.publish(udp)

	# draw plot
	try:
		if controller.show_animation:  # pragma: no cover
			plt.cla()
			plt.plot(cx, cy, ".r", label="course")
			plt.plot(states.x, states.y, "-b", label="trajectory")
			plt.legend()
			plt.xlabel("x[m]")
			plt.ylabel("y[m]")
			plt.axis("equal")
			plt.grid(True)

			plt.subplots(1)
			plt.plot(states.t, [iv for iv in states.v], "-r")
			plt.xlabel("Time[s]")
			plt.ylabel("Speed[km/h]")
			plt.grid(True)
			plt.show()

	except AttributeError:
		print("*-*-*-n-o-d-a-t-a-*-*-*")


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
