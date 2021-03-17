#!/usr/bin/env python
import rospy
# from pid_controller import *
from pid_controller_pi import *
from pure import *
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from carmaker_node.msg import UDP
from carmaker_node.msg import sub_udp
from carmaker_node.msg import localization

import tf

from carmaker_node.msg import lane_center

from carmaker_tracking.msg import emergency_state
from carmaker_tracking.msg import Front_vehicle_state

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

		self.car_lane = rospy.Subscriber('/lane_center_information', lane_center, self.lane_callback)

		# self.car_path = rospy.Subscriber('/path', PoseStamped, self.path_callback)
		self.car_e_stop = rospy.Subscriber('/Emergency_state', emergency_state, self.e_stop_callback)
		self.car_front = rospy.Subscriber('/Front_vehicle_information', Front_vehicle_state, self.Front_callback)

		self.loop = 0
		self.show_animation = True
		self.show_animation_gain = True

		self.car_now_time = None
		self.car_lasted_time = None
		self.car_dt_time = None

	def udp_callback(self, data_udp):
		self.car_sta = data_udp
		self.cur_vel = float(self.car_sta.vx * 3.6)  # m/s -> km/h
		self.cur_yaw = float(self.car_sta.yaw)
		self.car_now_time = (self.car_sta.header.stamp).to_sec()


	def pure_callback(self, data_pure):
		self.car_point = data_pure
		self.car_point_x = self.car_point.pose.pose.position.x
		self.car_point_y = self.car_point.pose.pose.position.y


	def local_callback(self, data_local):
		self.car_local_point = data_local
		self.car_local_point_x = self.car_local_point.x
		self.car_local_point_y = self.car_local_point.y
		self.car_local_point_theta = self.car_local_point.theta


	def lane_callback(self, lane_state):
		self.lane_state = lane_state
		self.lane_state_curve = self.lane_state.lane_curve_
		self.lane_state_offset = self.lane_state.offset_


	def e_stop_callback(self, e_state):
		self.emergency_state = e_state
		self.emergency_state_info = self.emergency_state.state
		# self.emergency_state == 0  60 < distance
		# self.emergency_state == 1  30 < distance < 60
		# self.emergency_state == 2  30 < distance


	def Front_callback(self, Front_state):
		self.Front_car = Front_state
		self.Front_car_x = self.Front_car.x
		self.Front_car_y = self.Front_car.y
		self.Front_car_dis = self.Front_car.distance
		self.Front_car_int = self.Front_car.intensity

"""
1*3.6m/s = 3.6km/h
tar_vel = km/h
cur_vel = m/s
cur_err = tar_vel - cur_vel*3.6 = km/h
"""

def get_csv():
	# Global path info

	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/gps_bspline_400_2.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s1_newcoord.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s2_newcoord.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s3_newcoord.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s4_newcoord.csv')

	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s1_new3waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s2_new3waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s3_new3waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s4_new3waypoint_carmaker.csv')

	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s1_new4waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s2_new4waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s3_new4waypoint_carmaker.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/s4_new4waypoint_carmaker.csv')

	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario1_wgs84.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario2_wgs84.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario3_wgs84.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario4_wgs84.csv')

	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario1_TM.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario2_TM.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario3_TM.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario4_TM.csv')

	df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario1_300_TM.csv')
	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/scenario1_fixed4.5.csv')

	# df_TM = pd.read_csv('~/catkin_ws/src/carmaker_node/src/path/test.csv')

	df_tf = {}
	df_dic = {}
	# key_name = ['0', '1']
	key_name = ['x', 'y']
	for key in key_name:
		df_dic[key] = []
		df_tf[key] = []

	# df_tf['x']=df_TM['0'].to_dict()
	df_tf['x']=df_TM['x'].to_dict()
	# df_tf['y']=df_TM['1'].to_dict()
	df_tf['y']=df_TM['y'].to_dict()

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

	key_name = ['start_x', 'start_y', 'goal_x', 'goal_y','d_x','d_y','cal_distance']
	for key in key_name:
		break_point_test[key] = []

	return break_point_test

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

	# cx = np.array(cx) * 100
	# cy = np.array(cy) * 100

	print(len(cx))
	print(len(cy))
	controller.loop = 0

	target_course = TargetCourse(cx, cy)
	states = States()

	# draw plot option -----------------------------------------------
	# controller.show_animation = True
	controller.show_animation = False

	# controller.show_animation_gain = True
	controller.show_animation_gain = False

	test_breaking = make_dic()
	stop_test_sw = False
	stop_test_sw_info = 0
	v_dt = 0

	listener = tf.TransformListener()
	# start -----------------------------------------------
	try:
		while (1):

			try:
				(trans, rot) = listener.lookupTransform('/odom', 'Wheel_Rear', rospy.Time(0))
				print(trans)
				print(rot)

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print("aaa")
				continue

			key = getKey()
			# print("\n\n")
			# print("*-----s-t-a-r-t---l-i-n-e-----*")
			# print("loop : ", controller.loop)

			if controller.car_lasted_time == None:
				# print("none test")
				controller.car_lasted_time = controller.car_now_time
			# print("time_now : ", controller.car_now_time)
			# print("time_lasted : ", controller.car_lasted_time)
			controller.car_dt_time = controller.car_now_time - controller.car_lasted_time
			controller.car_lasted_time = controller.car_now_time
			# print("time_dt : ", controller.car_dt_time)

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
				#
				# use localization
				state = State(x=controller.car_local_point_x,
							  y=controller.car_local_point_y,
							  yaw=controller.car_local_point_theta,
							  v=cur_vel_)

				lastIndex = len(cx) - 1

				target_ind, Lf, Ld = target_course.search_target_index(state)
				# print(target_ind)

				if lastIndex > target_ind:
					cur_vel_ = controller.cur_vel
					cur_dt = controller.car_dt_time

					# p controller
					# mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)

					# pid controller
					mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_, cur_dt)

					# steering wheel ang! = deg
					delta, target_ind, delta_rad, delta_gain, alpha, alpha_m, car_alpha_m, delta_m, Ld_m = pure_pursuit_steer_control(
						state, target_course, target_ind)
					udp.Ax = mov_vel
					# m/s^2
					# udp.SteeringWheel = delta
					udp.SteeringWheel = delta_gain
					# udp.SteeringWheel = delta_m


					v_dt += cur_dt
					# state.update(mov_vel, delta_rad, cur_dt)
					states.append(v_dt, state)


					try:
						if controller.emergency_state_info == 2:
							udp.VC_SwitchOn = 1
							udp.GearNo = -9
							print('stop - 1')
						elif controller.emergency_state_info == 0:
							udp.VC_SwitchOn = 1
							udp.GearNo = 1
							print('go!')

					except AttributeError as m:
						print("not yet E_state!")
						pass

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

					if controller.show_animation_gain:
						plt.cla()
						plt.gcf().canvas.mpl_connect('key_release_event',
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

					# --------------------------------------------------------------------
					controller.loop += 1
					print("*-----s-t-a-r-t---l-i-n-e-----*")
					print("loop : ", controller.loop)
					print("-----p-o-i-n-t---l-i-n-e-----")
					print("target_ind : ", target_ind)
					print("target_x : ", cx[target_ind])
					print("target_y : ", cy[target_ind])
					print("car_pint_x : ", controller.car_local_point_x)
					print("car_pint_y : ", controller.car_local_point_y)
					print("x", state.x)
					print("y", state.y)
					print("yaw", state.yaw)
					print("v", state.v)
					print("-----v-a-l-u-e---l-i-n-e-----")
					print("tar_vel : ", tar_vel_)
					print("cur_vel : ", cur_vel_)
					print("dt : ", cur_dt)
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
					# print("*-----t-e-s-t-----l-i-n-e-----*")
					# print("*-*--*  X  *--*-*")
					# print("localization_x : ", controller.car_local_point_x)
					# print("odom_x : ", controller.car_point_x)
					# print("odom_x_cal : ", controller.car_point_x * 0.000009)
					# print("*-*--*  Y  *--*-*")
					# print("localization_y : ", controller.car_local_point_y)
					# print("odom_y : ", controller.car_point_y)
					# print("odom_y_cal : ", controller.car_point_y * 0.000015)
					# print("*-*--*  Yaw  *--*-*")
					# print("localization_theta : ", controller.car_local_point_theta)
					# print("udp_yaw", state.yaw)
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
						print("lane_state_curve : ", controller.lane_state_curve)
						print("lane_state_offset : ", controller.lane_state_offset)

						print("emergency_state : ", controller.emergency_state_info)
						print("Front_car : ", controller.Front_car)
						print("Front_car_x : ", controller.Front_car_x)
						print("Front_car_y : ", controller.Front_car_y)
						print("Front_car_dis : ", controller.Front_car_dis)
						print("Front_car_int : ", controller.Front_car_int)
					except AttributeError:
						print("not yet!")


					print("*-----e-n-d-----l-i-n-e-----*")

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

