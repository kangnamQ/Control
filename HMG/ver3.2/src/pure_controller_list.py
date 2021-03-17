#!/usr/bin/env python
import rospy
# from pid_controller_list import *
from pid_controller import *
from pure_list import *
from hellocm_msgs.msg import UDP
from hellocm_msgs.msg import sub_udp
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import sys, select, tty, termios
import numpy as np
import pandas as pd
import math


class Controller(object):
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
		self.car_pos = rospy.Subscriber('/Odometry', Odometry, self.pure_callback)
		# self.car_path = rospy.Subscriber('/path', PoseStamped, self.path_callback)

		self.setting = PoseStamped()
		self.setting.header.frame_id = "Info"
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

		self.dt = 0
		self.dts_time = 0
		self.loop = 0


	def udp_callback(self, data_udp):
		self.car_sta = data_udp
		self.cur_vel = float(self.car_sta.vx * 3.6)  # m/s -> km/h
		self.cur_yaw = float(self.car_sta.yaw)

		# print("speed = ", self.cur_vel)
		# self.calc()

	def pure_callback(self, data_pure):
		self.car_point = data_pure
		self.car_point_x = self.car_point.pose.pose.position.x
		self.car_point_y = self.car_point.pose.pose.position.y

		self.setting.pose.position.x = self.car_point.pose.pose.position.x
		self.setting.pose.position.y = self.car_point.pose.pose.position.y
		self.setting.pose.position.z = self.car_point.pose.pose.position.z
		self.setting.pose.orientation.x = self.car_point.pose.pose.orientation.x
		self.setting.pose.orientation.y = self.car_point.pose.pose.orientation.y
		self.setting.pose.orientation.z = self.car_point.pose.pose.orientation.z
		self.setting.pose.orientation.w = self.car_point.pose.pose.orientation.w



	# def path_callback(self, data_path):
	# 	self.path_point = data_path
	# 	self.path_point_x = self.path_point.pose.position.x + 1.15
	# 	self.path_point_y = self.path_point.pose.position.y + 0
	# 	self.point_x_ = np.array([])
	# 	self.point_y_ = np.array([])
	# 	self.point_x = np.append(self.point_x_, np.array([self.path_point_x]))
	# 	self.point_y = np.append(self.point_y_, np.array([self.path_point_y]))


		"""
		Whell_FL = (3.85, 0.77525, 0.296)
		Whell_FR = (3.85, -0.77525, 0.296)
		whell_FM = (3.85, 0, 0.296)
		Whell_RL = (1.15, 0.77525, 0.296)
		Whell_RR = (1.15, -0.77525, 0.296)
		whell_RM = (1.15, 0, 0.296)
		"""



def get_csv():
	df_list = []
	df_list.append(df_TM['field.header.stamp'])
	df_list.append(df_TM['field.pose.pose.position.x'])
	df_list.append(df_TM['field.pose.pose.position.y'])
	df_list.append(df_TM['field.pose.pose.position.z'])

	df_list.append(df_TM['field.pose.pose.orientation.x'])
	df_list.append(df_TM['field.pose.pose.orientation.y'])
	df_list.append(df_TM['field.pose.pose.orientation.z'])
	df_list.append(df_TM['field.pose.pose.orientation.w'])
	return df_list


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
	PID_controller = pid_controller()

	udp = sub_udp()
	udp.GearNo = 1
	udp.Ax = 0
	udp.SteeringWheel = 0
	tar_vel_ = 10
	cur_vel_ = 0
	udp.VC_SwitchOn = 1
	udp.GearNo = 1

	df_TM = pd.read_csv('./file2.csv')
	pose_list = get_csv()
	cx_list = pose_list[1]
	cy_list = pose_list[2]
	controller.loop = 0

	cx_arr = np.array(cx_list)
	cy_arr = np.array(cy_list)

	#
	# cx = np.array([-138.4821722, -116.88, -100.73, -85.26, -63.96, -36.80, -20.17,
	# 			   -5.93, 16.44, 36.71, 62.76, 80.81, 95.56, 127.04,
	# 			   152.50, 174.01, 204.72, 234.90, 258.34, 262.71, 273.30,
	# 			   282.06, 301.29, 328.40, 342.70, 350.54, 356.53, 362.96,
	# 			   376.32, 402.06])
	#
	# cy = np.array([-70.43534909, -86.06, -97.74, -108.99, -124.29, -144.08, -156.22,
	# 			   -166.47, -182.63, -197.29, -216.02, -229.25, -240.06, -262.84,
	# 			   -281.23, -296.92, -319.26, -340.51, -357.42, -360.39, -365.99,
	# 			   -370.98, -384.95, -404.90, -415.27, -422.31, -428.35, -434.23,
	# 			   -443.93, -462.60])

	# initial state
	state = State(x=controller.car_point_x,
				  y=controller.car_point_y,
				  yaw=controller.cur_yaw,
				  v=cur_vel_)
	# x = controller.car_point_x
	# y = controller.car_point_y
	# z = controller.car_point_z

	lastIndex = len(cx_arr) - 1

	states = States()
	states.append(controller.dts_time, state)
	target_course = TargetCourse(cx_arr, cy_arr)
	target_ind, _ = target_course.search_target_index(state)

	try:
		while (1):
			key = getKey()
			print(controller.loop)


			controller.current_time = rospy.Time.now()
			controller.setting.header.stamp.secs = (controller.current_time).to_sec()
			controller.setting.header.stamp.nsecs = controller.dts_time

			controller.dt = (controller.current_time - controller.last_time).to_sec()
			controller.dts_time = controller.dts_time + controller.dt

			if key == 't':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				try:
					tar_vel_ = float(input("target velue : "))
					print("set", tar_vel_, "km/h")
				except NameError:
					print("try again")


			cur_vel_ = controller.cur_vel
			cur_dt_ = controller.dt

			mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_, cur_dt_)
			# steering wheel ang! = deg
			delta, target_ind, delta_rad = pure_pursuit_steer_control(
				state, target_course, target_ind)

			udp.Ax = mov_vel
			# m/s^2
			udp.SteeringWheel = delta

			# udp.SteeringWheel = di
			# deg
			# print(target_ind)

			state.update(mov_vel, delta_rad, cur_dt_)
			# state.update(mov_vel, di, cur_dt_)  # Control vehicle

			states.append(controller.dts_time, state)


			distance_this_index = state.calc_distance(cx, cy)
			print("distance_this_index ========", distance_this_index)

			distance_next_index = state.calc_distance(cx_arr[i + 5], cy_arr[i + 5])
			if distance_this_index > distance_next_index:
				i = i + 5

			if distance_this_index < 1.0:
				i = i + 1

			print("iiiiiiiiiiiiiiiiiiiiiiiiii = ", i)


			# --------------------------------------------------------------------
			controller.last_time = controller.current_time
			controller.loop += 1


			# mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
			"""
			1*3.6m/s = 3.6km/h
			tar_vel = km/h
			cur_vel = m/s
			cur_err = tar_vel - cur_vel*3.6 = km/h
			"""
			print("tar_vel : ", tar_vel_)
			print("cur_vel : ", cur_vel_)
			print("dt : ", cur_dt_)
			print("mov_vel : ", mov_vel)
			print("----------------------------")
			print("car_pint_x : ", controller.car_point_x)
			print("car_pint_y : ", controller.car_point_y)
			print("x", state.x)
			print("y", state.y)
			print("yaw", state.yaw)
			print("v", state.v)


			# ----------------------------------------------------
			#  target course << waypoint
			# cx = controller.path_point_x
			# cy = controller.path_point_y
			# cx = controller.point_x
			# cy = controller.point_y

			# print("cx : ", cx)
			# print("cy : ", cy)
			# print("path_point x : ", controller.path_point_x)
			# print("path_point y : ", controller.path_point_y)
			# print("point_x_ : ", controller.point_x_)
			# print("point_y_ : ", controller.point_y_)

			#cx = np.arange(0, 50, 0.5)
			#cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
			# print("cx ",cx)
			# print("cy ",cy)



			# while T >= time and lastIndex > target_ind:
			# 	# Calc control input
			# 	# ai = pure.proportional_control(tar_vel_, state.v)
			# 	mov_vel = PID_controller.accel_control(tar_vel_, state.v)
			# 	di, target_ind = pure_pursuit_steer_control(
			# 		state, target_course, target_ind)
			#
			# 	udp.Ax = mov_vel
			# 	udp.SteeringWheel = di
			# 	# print(target_ind)
			#
			# 	state.update(mov_vel, di)  # Control vehicle
			#
			# 	time += dt
			# 	states.append(time, state)




			if key == '1':
				udp.VC_SwitchOn = 0
				# mode_pub.publish(udp)
				print('maneuver')

			if key == 'c':
				udp.VC_SwitchOn = 1
				udp.GearNo = 0
				udp.Ax = 0
				udp.SteeringWheel = 0
				# mode_pub.publish(udp)
				print('stop - 0.5')

			if key == 'x':
				udp.VC_SwitchOn = 1
				udp.GearNo = -9
				udp.Ax = 0
				udp.SteeringWheel = 0
				# mode_pub.publish(udp)
				print('stop - 1')

			if key == 'w':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				udp.Ax += 1
				# mode_pub.publish(udp)
				print('Ax +')

			if key == 's':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				udp.Ax -= 1
				# mode_pub.publish(udp)
				print('Ax -')

			if key == 'a':
				udp.VC_SwitchOn = 1
				udp.SteeringWheel += 0.1
				# mode_pub.publish(udp)
				print('SteeringWheel +')

			if key == 'd':
				udp.VC_SwitchOn = 1
				udp.SteeringWheel -= 0.1
				# mode_pub.publish(udp)
				print('SteeringWheel -')

			if key == 'e':
				udp.VC_SwitchOn = 1
				udp.SteeringWheel = 0
				# mode_pub.publish(udp)
				print('SteeringWheel 0')

			else:
				if (key == '\x03'):
					# pub_start.publish(0)
					break

			controller.car_pub.publish(udp)

	except rospy.ROSInterruptException:
		pass