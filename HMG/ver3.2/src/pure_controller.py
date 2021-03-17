#!/usr/bin/env python
import rospy
from pid_controller import *
from pure import *
from hellocm_msgs.msg import UDP
from hellocm_msgs.msg import sub_udp
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import sys, select, tty, termios
import numpy as np
import math


class Controller(object):
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
		self.car_pos = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

		self.car_path = rospy.Subscriber('/path', PoseStamped, self.path_callback)

	def udp_callback(self, data_udp):
		self.car_sta = data_udp
		self.cur_vel = float(self.car_sta.vx * 3.6)  # m/s -> km/h
		self.cur_yaw = float(self.car_sta.yaw)

		# print("speed = ", self.cur_vel)
		# self.calc()

	def odom_callback(self, data_odom):
		self.car_point = data_odom
		self.car_point_x = self.car_point.pose.pose.position.x
		self.car_point_y = self.car_point.pose.pose.position.y


	def path_callback(self, data_path):
		self.path_point = data_path
		self.path_point_x = self.path_point.pose.position.x
		self.path_point_y = self.path_point.pose.position.y
		# self.point_x_ = np.array([])
		# self.point_y_ = np.array([])
		# self.point_x = np.append(self.point_x_, np.array([self.path_point_x]))
		# self.point_y = np.append(self.point_y_, np.array([self.path_point_y]))

		"""
		Whell_FL = (3.85, 0.77525, 0.296)
		Whell_FR = (3.85, -0.77525, 0.296)
		whell_FM = (3.85, 0, 0.296)
		Whell_RL = (1.15, 0.77525, 0.296)
		Whell_RR = (1.15, -0.77525, 0.296)
		whell_RM = (1.15, 0, 0.296)
		
		1*3.6m/s = 3.6km/h
		tar_vel = km/h
		cur_vel = m/s
		cur_err = tar_vel - cur_vel*3.6 = km/h
		"""

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
	udp.GearNo = 1


	try:
		while (1):
			key = getKey()

			if key == 't':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				tar_vel_ = float(input("target velue : "))
				print("set", tar_vel_, "km/h")

			print("test", controller.path_point)


			# initial state
			state = State(x=controller.car_point_x,
						  y=controller.car_point_y,
						  yaw=controller.cur_yaw,
						  v=controller.cur_vel)
			# x = controller.car_point_x
			# y = controller.car_point_y
			# z = controller.car_point_z

			tar_vel_ = 10
			# cur_vel_ = controller.cur_vel
			# # mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
			target_course = TargetCourse(controller.path_point_x, controller.path_point_y)

			Ld, alpha = target_course.search_target(state)

			mov_vel = PID_controller.accel_control(tar_vel_, state.v)
			di = pure_pursuit_steer_control(state, target_course)

			udp.Ax = mov_vel
			udp.SteeringWheel = di

			print("tar_vel : ", tar_vel_)
			print("cur_vel : ", cur_vel_)
			print("before : ", udp.Ax)
			print("P : ", PID_controller.P)
			print("mov_vel : ", mov_vel)

			print("after Ax: ", udp.Ax)
			print("----------------------------")

			print("car_x : ", state.x)
			print("car_y : ", state.y)
			print("tar_x : ", target_course.target_x)
			print("tar_y : ", target_course.target_y)
			print("Ld : ", Ld)
			print("alpha : ", alpha)

			print("car_point_x", controller.car_point_x)
			print("target_point_x : ", controller.path_point_x)
			print("car_pint_y", controller.car_point_y)
			print("target_point_y : ", controller.path_point_y)
			print("yaw", state.yaw)
			print("v", state.v)
			print("----------------------------")

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

			else:
				if (key == '\x03'):
					# pub_start.publish(0)
					break

			controller.car_pub.publish(udp)

	except rospy.ROSInterruptException:
		pass