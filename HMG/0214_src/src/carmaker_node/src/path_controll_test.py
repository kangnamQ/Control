#!/usr/bin/env python
import rospy
from pid_controller import *
from pure import *
from hellocm_msgs.msg import UDP
from hellocm_msgs.msg import sub_udp
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import sys, select, tty, termios
import numpy as np
import math
import pandas as pd


class Controller(object):
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
		self.car_pos = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

		self.car_path = rospy.Subscriber('/path', Path, self.path_callback)
		# self.car_path = None

		self.pub_sw = False
		self.sub_sw = True
		self.main_sw = False
		self.path_point = None
		self.seq = 0
		self.path_point_dict = {'index':[],
								'x':[],
								'y':[]}

		self.car_sta = None
		self.cur_vel = 0
		self.cur_yaw = 0
		self.car_point = None
		self.car_point_x = 0
		self.car_point_y = 0


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
		self.cur_yaw = float(self.car_sta.yaw)	# rad

		# print("speed = ", self.cur_vel)
		# self.calc()

	def odom_callback(self, data_odom):
		self.car_point = data_odom
		self.car_point_x = self.car_point.pose.pose.position.x
		self.car_point_y = self.car_point.pose.pose.position.y


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
	PID_controller = pid_controller()
	udp = sub_udp()

	udp.GearNo = 1
	udp.Ax = 0
	udp.SteeringWheel = 0
	tar_vel_ = 0
	cur_vel_ = 0
	udp.VC_SwitchOn = 1

	controller.loop = 0
	time = 0

	state = State(x=controller.car_point_x,
				  y=controller.car_point_y,
				  yaw=controller.cur_yaw,
				  v=controller.cur_vel)
	loop = 0
	while not controller.main_sw:
		print("loading")
		while not controller.pub_sw:
			# controller.car_path = rospy.Subscriber('/path', Path, controller.path_callback)
			print(loop,
				"index : ",len(controller.path_point_dict['index']),
				  "x : ", len(controller.path_point_dict['x']),
				  "y : ", len(controller.path_point_dict['y']),
				  "seq : ",controller.seq)

			if controller.seq == 10820 and len(controller.path_point_dict['x']) > 1000:
				controller.pub_sw = True
				controller.sub_sw = False
				print("loading complete")
			loop += 1

		try:
			while not controller.sub_sw:
				key = getKey()
				print("start sub_sw")

				if key == 'p':
					controller.pub_sw = False
					print("1 time pub")

				if key == 'o':
					controller.sub_sw = False
					print("1 time sub")


				if key == 't':
					udp.VC_SwitchOn = 1
					udp.GearNo = 1
					try:
						tar_vel_ = float(input("target velue : "))
						print("set", tar_vel_, "km/h")
					except NameError:
						print("try again")

				print("x", controller.car_point_x)
				print("y", controller.car_point_y)
				print("v", controller.cur_vel)
				print(state.x)
				print(state.y)
				print(state.v)
				# x = controller.car_point_x
				# y = controller.car_point_y
				# z = controller.car_point_z


				try:
					controller.current_time = rospy.Time.now()
					controller.setting.header.stamp.secs = (controller.current_time).to_sec()
					controller.setting.header.stamp.nsecs = controller.dts_time

					controller.dt = (controller.current_time - controller.last_time).to_sec()
					controller.dts_time = controller.dts_time + controller.dt

					if len(controller.path_point_dict['x']) != 0:
						while not controller.sub_sw:
							# initial state
							state = State(x=controller.car_point_x,
										  y=controller.car_point_y,
										  yaw=controller.cur_yaw,
										  v=controller.cur_vel)

							cx = controller.path_point_dict['x']
							cy = controller.path_point_dict['y']

							cur_vel_ = controller.cur_vel
							cur_dt_ = controller.dt


							lastIndex = len(cx) - 1

							states = States()
							states.append(cur_dt_, state)
							target_course = TargetCourse(cx, cy)
							target_ind, _ = target_course.search_target_index(state)

							while lastIndex > target_ind:
							# Calc control input
								mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
								delta, target_ind, delta_rad = pure_pursuit_steer_control(
									state, target_course, target_ind)
								# steering wheel ang! = deg

								udp.Ax = mov_vel
								# m/s^2
								udp.SteeringWheel = delta

								# state.update(mov_vel, delta_rad, cur_dt_)  # Control vehicle
								time += cur_dt_
								states.append(time, state)


							# --------------------------------------------------------------------
							controller.last_time = controller.current_time
							controller.loop += 1

				except AttributeError:
					print("2 error")

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

		except rospy.ROSInterruptException:
			pass
