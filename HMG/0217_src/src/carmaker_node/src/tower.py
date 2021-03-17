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
			target_ind, Lf, Ld = target_course.search_target_index(state, 1.6)

			cur_vel_ = controller.cur_vel
			cur_dt = controller.car_dt_time
			cur_acc_ = controller.car_acc

		except (AttributeError, IndexError) as set_err:
			print(set_err)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue

		# traffic light | red, yellow : stop, green : go
		if controller.traffic_stat == 1:
			udp.GearNo = -9
			print("traffic light red!!")
		# else:
		# 	udp.GearNo = 1

		print("tra : ", controller.traffic_stat)

		# In global path
		if last_index > target_ind:

			# left_and_right.py
			# if cfg.mode == "trun"
			direction, LI = lar.get_ang(target_ind)
			udp.Lights_Indicator = LI

			if direction is None:
				# 직진
				print("직진")
			else:
				print(direction)
				# 여기서 turn모드

			# acc_controller.py
			# if cfg.mode == "acc":
			try:
				distance_new = controller.Front_car_dis
				E_state = controller.emergency_state_info
				distance_old, cfg, Lights_Hazard = ac.accs(distance_new, E_state, tar_vel_, cur_vel_, cur_dt, cfg,
														   distance_old)
				udp.Lights_Hazard = Lights_Hazard
				# pd controller
				mov_vel = PID_controller.accel_control(cfg.acc_v, cur_vel_, cur_dt, 1, 0, 1)
				udp.Ax = mov_vel  # m/s^2

			except AttributeError as a_e:
				mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_, cur_dt, 1, 0, 1)
				udp.Ax = mov_vel  # m/s^2
				cfg.state = 0
			#

			# print(cfg.state, cfg.acc_v)

			# lkas_controller.py
			# if cfg.mode == "lkas":
			steering_re = lc.lkas(controller.lane_center_pix)
			if steering_re is None:
				delta_gain = pure_pursuit_steer_control(
					state, target_course, target_ind, 0.08)
				udp.SteeringWheel = delta_gain
			else:
				print("############################################")
				udp.SteeringWheel = steering_re
			#

			# local_path.py
			# if cfg.mode == "local":
			try:
				# E_state = controller.emergency_state_info
				local_x = controller.lo_x
				local_y = controller.lo_y
				local_state = controller.lo_state

				local_course = TargetCourse(local_x, local_y)
				local_last_index = len(cx) - 1

				# 여기서 스테이트만 주지 말고 게인값 까지 다 줄 수 있게 수정할 것.
				# 라스트 인덱스도 필요하면 주고 필요없으면 제거 하면 될 것 같음.
				local_ind, local_Lf, local_Ld = local_course.search_target_index(state, 1)

				# here, 필요없는 변수 리턴 다 풀고 필요한것만 리턴하게 할것.
				local_delta_gain = pure_pursuit_steer_control(
					state, local_course, local_ind, 0.2)
				udp.SteeringWheel = local_delta_gain

				"""
				오늘 할일
				먼저 로컬 기본적인 틀은 짜놨지만 퓨어 제어기에서 들어가거나 나오는 값들의 수정이 필요하고 게인값을 넣을 수 있게 조정이 필요함.
				그리고 방향 지시등에 따른 변경도 추가해야함
				한다음에 모드로 움직이는 트리거를 만들어야함.
				모드를 선택해 주는 셀렉 모드도 하나 만들고 이런 로컬이 끝나면 셀렉모드로 돌아와서 턴이면 턴하고 웨이면 웨이포인트 따라가는 트리거를
				만든 다음 모드 추가를 진행하면 될 것 같음.
				
				다만 게인값을 넣는다고 한들 그게 과연 튜닝없이 될까 라는 생각이긴함.
				일단은 해보고 이쪽은 대강 값아니까 그거 넣어보면 되고
				
				로컬이랑 글로벌은 완전히 다른변수로 돌아가야해서 혹시나 찝히거나 하는 부분이 있는지 잘 확인해야함.
				글로벌변수는 완전하게 납둔 채로 로컬이 진행되어야함.
				
				그 표지판 정민이형 컴에서만 돌아간다고 해서 완성한 다음 그부분 전달해서 추가받아야 할듯.
				
				그리고 웨이포인트 만들어준 4개 테스트 해봐야함.
				3번 4번을 아직 안해봤음..
				
				차간거리제어도 공식 아까 다시 만들어 놓은거 적용해서 잘 작동하나 테스트 해볼 것.
				"""

			except AttributeError as local_err:
				count += 1
				local_x = []
				local_y = []
				local_state = None

			v_dt += cur_dt
			states.append(v_dt, state)
			controller.loop += 1

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

		else:
			print("lastIndex < target_ind")
			print("*-*-*-*-*-*-END-*-*-*-*-*-*")
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
