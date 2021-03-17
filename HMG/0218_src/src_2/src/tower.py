#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from config import Config
from pid_controller import *
from pure_controller import *
from controller import Controller
import method as mt
import lkas_controller as lc
import acc_controller as ac
import left_and_right as lar
import sys, select, tty, termios

from carmaker_node.msg import sub_udp

import matplotlib.pyplot as plt


def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

	return key


def main():
	global settings
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('controller')
	controller = Controller()
	PID_controller = Pid_Controller()
	cfg = Config()
	udp = sub_udp()
	states = States()

	pose_dict = mt.get_csv()
	cx = pose_dict['x']
	cy = pose_dict['y']
	target_course = TargetCourse(cx, cy)

	udp.GearNo = 1
	tar_vel_ = 0
	controller.loop = 0
	v_dt = 0
	distance_old = 0
	count = 0

	acc_mode = 0
	pure_mode = 0

	local_x = None
	local_y = None
	local_course = None
	local_ind = None
	local_now_ind = None
	local_state = 2
	local_stop = 0

	E_state = None
	distance_new = None

	Lf_k = 1.6

	# start -----------------------------------------------
	while 1:
		# cal dt
		if controller.car_lasted_time == 0:
			controller.car_lasted_time = controller.car_now_time
		controller.car_dt_time = controller.car_now_time - controller.car_lasted_time
		controller.car_lasted_time = controller.car_now_time
		key = getKey()

		try:
			# use localization
			state = State(x=controller.car_local_point_x, y=controller.car_local_point_y,
						  yaw=controller.car_local_point_theta, v=controller.cur_vel)

			last_index = len(cx) - 1
			target_ind, Ld, now_ind = target_course.search_target_index(state, Lf_k)

			cur_vel_ = controller.cur_vel
			cur_dt = controller.car_dt_time
			cur_acc_ = controller.car_acc

		except (AttributeError, IndexError) as set_err:
			print(set_err)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue

		try:
			E_state = controller.emergency_state_info
			distance_new = controller.Front_car_dis
			print("1 distance : ", distance_new)
			controller.Front_car_dis = None
		except AttributeError as e_s_err:
			print("No E_state, No distance")

		print("distance_ in : ", distance_new)

		print("mode : ", cfg.mode)

		# --------------------------------------------------------------------------------------
		# In global path
		if last_index > target_ind:
			cfg.mode = "waypoint"

			try:
				local_x = controller.lo_x
				local_y = controller.lo_y
				local_state = controller.lo_state
				print("local!")
				print("local_x : ", local_x)
				print("local_y : ", local_y)

				if local_x is not None:
					local_course = TargetCourse(local_x, local_y)
					local_last_index = len(cx) - 1
					local_ind, local_Ld, local_now_ind = local_course.search_target_index(state, Lf_k)
					if (local_last_index - 10) <= local_ind:
						local_stop = 1
					if local_state == 1:
						local_stop = 1

			except (AttributeError, ValueError) as local_err:
				print("local err : ", local_err)


			# 신호등
			if cfg.mode is "local" and local_x is not None and local_ind is not None:
				direction, LI = lar.local_get_angle(local_now_ind, local_x, local_y)
				print("local traffic")
			else:
				direction, LI = lar.get_angle(now_ind)
			# print("LI : ", LI)
			# print("direction : ", direction)
			udp.Lights_Indicator = LI

			# 직선 스티어링
			steering_re = lc.lkas(controller.lane_center_pix)

			# mode pick --------------------------------------------------------------------------------------

			if steering_re is not None and cfg.mode is not "turn":
				print("############################################")
				cfg.mode = "line"

			if direction is not None:
				cfg.mode = "turn"

			if local_state == 0:
				cfg.mode = "local"

			print("local_state : ", local_state)

			if local_stop == 1:
				print("local_state : ", local_state)
				cfg.mode = "waypoint"
				local_x = None
				local_y = None
				local_course = None
				local_ind = None
				local_state = 2
				local_stop = 0

			print("wayin",cfg.mode)

			#
			Ks = 0.2
			Lf_k = 1.6
			if cfg.mode == "waypoint":
				acc_mode = 1
				pure_mode = 0

			if cfg.mode == "line":
				acc_mode = 1
				pure_mode = 1

			if cfg.mode == "turn":
				acc_mode = 0
				pure_mode = 0
				Ks = 0.6
				Lf_k = 0.5

			if cfg.mode == "local":
				acc_mode = 0
				pure_mode = 2
				Ks = 0.6
				Lf_k = 1

			# mode --------------------------------------------------------------------------------------
			# acc mode
			target_velocity = tar_vel_
			if acc_mode == 1:
				if distance_new is not None:
					distance_old, cfg, Lights_Hazard, GearNo = ac.accs(distance_new, E_state, tar_vel_, cur_vel_,
																	   cur_dt, cfg, distance_old)
					udp.Lights_Hazard = Lights_Hazard
					if GearNo == -9:
						udp.GearNo = -9
					else:
						udp.GearNo = 1
					# pd controller
					target_velocity = cfg.acc_tar
				else:
					acc_mode = 0
			if cfg.mode == "local":
				target_velocity = 5
			if cfg.mode == "turn":
				target_velocity = 10

			mov_vel = PID_controller.accel_control(target_velocity, cur_vel_, cur_dt, 1, 0, 1)
			udp.Ax = mov_vel
			#

			# pure mode
			delta_gain = pure_pursuit_steer_control(state, target_course, target_ind, Ks)
			steering_value = delta_gain
			if pure_mode == 1:
				steering_value = steering_re
				if steering_re is None:
					steering_value = 0
			if pure_mode == 2:
				local_delta_gain = pure_pursuit_steer_control(state, local_course, local_ind, Ks)
				steering_value = local_delta_gain
			udp.SteeringWheel = steering_value
			#

			controller.loop += 1

			# draw plot
			v_dt += cur_dt
			states.append(v_dt, state)

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
			#

		else:
			print("lastIndex < target_ind")
			print("*-*-*-*-*-*-END-*-*-*-*-*-*")
		# ----------------------------------------------------

		# traffic light | red, yellow : stop, green : go
		if controller.traffic_state == 1:
			udp.GearNo = -9
			cfg.mode = "stop"

		else:
			udp.GearNo = 1

		# ----------------------------------------------------

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
