#!/usr/bin/env python
import rospy
from pid_controller import *
from pure import *
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from carmaker_node.msg import UDP
from carmaker_node.msg import sub_udp
from carmaker_node.msg import localization

import sys, select, tty, termios
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt

class Controller(object):
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
		self.car_pos = rospy.Subscriber('/odom', Odometry, self.pure_callback)
		self.car_local = rospy.Subscriber('/localization', localization, self.local_callback)
		# self.car_path = rospy.Subscriber('/path', PoseStamped, self.path_callback)

		self.setting = PoseStamped()
		self.setting.header.frame_id = "Info"
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

		self.dt = 0
		self.dts_time = 0
		self.loop = 0
		self.show_animation = True


	def udp_callback(self, data_udp):
		self.car_sta = data_udp
		self.cur_vel = float(self.car_sta.vx * 3.6)  # m/s -> km/h
		self.cur_yaw = float(self.car_sta.yaw)


	def pure_callback(self, data_pure):
		self.car_point = data_pure
		self.car_point_x = self.car_point.pose.pose.position.x
		self.car_point_y = self.car_point.pose.pose.position.y

		self.setting.pose.position.x = self.car_point.pose.pose.position.x
		self.setting.pose.position.y = self.car_point.pose.pose.position.y

	def local_callback(self, data_local):
		self.car_local_point = data_local
		self.car_local_point_x = self.car_local_point.x
		self.car_local_point_y = self.car_local_point.y
		self.car_local_point_theta = self.car_local_point.theta

		self.setting.pose.orientation.x = self.car_local_point.x
		self.setting.pose.orientation.y = self.car_local_point.y
		self.setting.pose.orientation.w = self.car_local_point.theta

"""
1*3.6m/s = 3.6km/h
tar_vel = km/h
cur_vel = m/s
cur_err = tar_vel - cur_vel*3.6 = km/h
"""


def get_csv():
	# Global path info
	df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s1_new4waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s2_new4waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s3_new4waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s4_new4waypoint_carmaker.csv')

	df_tf = {}
	df_dic = {}
	key_name = ['0', '1']
	for key in key_name:
		df_dic[key] = []
		df_tf[key] = []

	df_tf['x']=df_TM['0'].to_dict()
	df_tf['y']=df_TM['1'].to_dict()

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


if __name__ == '__main__':

	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('controller')
	controller = Controller()
	PID_controller = pid_controller()

	udp = sub_udp()
	udp.GearNo = 1
	udp.Ax = 0
	udp.SteeringWheel = 0
	cur_vel_ = 0
	tar_vel_ = 0
	udp.VC_SwitchOn = 1
	udp.GearNo = 1

	pose_dict = get_csv()

	cx = pose_dict['x']
	cy = pose_dict['y']
	print(len(cx))
	print(len(cy))
	controller.loop = 0

	target_course = TargetCourse(cx, cy)
	states = States()
	# controller.show_animation = True
	controller.show_animation = False

	try:
		while (1):
			key = getKey()
			print("\n\n")
			print("*-----s-t-a-r-t---l-i-n-e-----*")
			print("loop : ", controller.loop)

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
				except SyntaxError:
					print("try again")



			lastIndex = len(cx) - 1
			print(lastIndex)

			try:
				# use odom
				# state = State(x=controller.car_point_x,
				# 			  y=controller.car_point_y,
				# 			  yaw=controller.cur_yaw,
				# 			  v=cur_vel_)
				#
				# use localization
				state = State(x=controller.car_local_point_x,
							  y=controller.car_local_point_y,
							  yaw=controller.car_local_point_theta,
							  v=cur_vel_)


				target_ind, Lf, Ld = target_course.search_target_index(state)
				print(target_ind)


				if lastIndex > target_ind:

					states.append(controller.dts_time, state)

					cur_vel_ = controller.cur_vel
					cur_dt_ = controller.dt

					mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
					# steering wheel ang! = deg
					delta, target_ind, delta_rad, delta_gain, alpha, alpha_m, car_alpha_m, delta_m, Ld_m = pure_pursuit_steer_control(
						state, target_course, target_ind)



					udp.Ax = mov_vel
					# m/s^2
					# udp.SteeringWheel = delta
					udp.SteeringWheel = delta_gain
					# udp.SteeringWheel = delta_m

					# state.update(mov_vel, delta_rad, cur_dt_)
					states.append(controller.dts_time, state)

					if controller.show_animation:  # pragma: no cover
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

					# --------------------------------------------------------------------
					controller.last_time = controller.current_time
					controller.loop += 1

					print("-----p-o-i-n-t---l-i-n-e-----")
					print("target_ind : ", target_ind)
					print("target_x : ", cx[target_ind])
					print("target_y : ", cy[target_ind])
					print("car_pint_x : ", controller.car_point_x)
					print("car_pint_y : ", controller.car_point_y)
					print("x", state.x)
					print("y", state.y)
					print("yaw", state.yaw)
					print("v", state.v)
					print("-----v-a-l-u-e---l-i-n-e-----")
					print("tar_vel : ", tar_vel_)
					print("cur_vel : ", cur_vel_)
					print("dt : ", cur_dt_)
					print("mov_vel : ", mov_vel)
					print("steeringWeel : ", delta)
					print("Lf : ", Lf)
					print("Ld : ", Ld)
					print("-----U-D-P----l-i-n-e-----")
					print("udp.VC switch : ", udp.VC_SwitchOn)
					print("udp.Ax : ", udp.Ax)
					print("udp.SteeringWheel : ", udp.SteeringWheel)
					print("udp.GearNo : ", udp.GearNo)
					print("steeringWeel : ", controller.car_sta.vx)
					print("*-----t-e-s-t-----l-i-n-e-----*")
					print("")
					print("*-*--*  X  *--*-*")
					print("localization_x : ", controller.car_local_point_x)
					print("odom_x : ", controller.car_point_x)
					print("*-*--*  Y  *--*-*")
					print("localization_y : ", controller.car_local_point_y)
					print("odom_y : ", controller.car_point_y)
					print("*-*--*  Yaw  *--*-*")
					print("localization_theta : ", controller.car_local_point_theta)
					print("udp_yaw", state.yaw)
					print("")
					print("*-*--*  cal  *--*-*")
					print("delta : ", delta)
					print("delta_m() : ", delta_m)
					print("delta_gain() : ", delta_gain)

					print("alpha : ", alpha)
					print("alpha_m() : ", alpha_m)
					print("car_alpha_m() : ", car_alpha_m)
					print("Ld : ", Ld)
					print("Ld_m : ", Ld_m)

					print("Lf : ", Lf)
					print("ind : ", target_ind)

					print("*-----e-n-d-----l-i-n-e-----*")

				else:
					print("lastIndex < target_ind")
					print("*-*-*-*-*-*-END-*-*-*-*-*-*")


			except AttributeError:
				print("*-*-*-L-O-D-I-N-G-*-*-*")
				continue

			# ----------------------------------------------------

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

			if key == 'w':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				udp.Ax += 1
				print('Ax +')

			if key == 's':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				udp.Ax -= 1
				print('Ax -')

			if key == 'a':
				udp.VC_SwitchOn = 1
				udp.SteeringWheel += 0.1
				print('SteeringWheel +')

			if key == 'd':
				udp.VC_SwitchOn = 1
				udp.SteeringWheel -= 0.1
				print('SteeringWheel -')

			if key == 'e':
				udp.VC_SwitchOn = 1
				udp.SteeringWheel = 0
				print('SteeringWheel 0')

			else:
				if (key == '\x03'):
					break

			controller.car_pub.publish(udp)
	except rospy.ROSInterruptException:
		pass


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

