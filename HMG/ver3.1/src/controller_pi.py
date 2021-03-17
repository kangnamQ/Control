#!/usr/bin/env python
import rospy
from pid_controller_pi import *
from hellocm_msgs.msg import UDP
from hellocm_msgs.msg import sub_udp
import sys, select, tty, termios

class Controller(object):
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)

	def udp_callback(self, data):
		self.car_sta = data
		self.cur_vel = float(self.car_sta.vx * 3.6)  #m/s -> km/h
		#print("speed = ", self.cur_vel)
		#self.calc()


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

			# tar_vel_ = tar_vel_want
			cur_vel_ = controller.cur_vel
			mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
			"""
			1*3.6m/s = 3.6km/h
			tar_vel = km/h
			cur_vel = m/s
			cur_err = tar_vel - cur_vel*3.6 = km/h
			"""
			print("tar_vel : ", tar_vel_)
			print("cur_vel : ", cur_vel_)
			print("before : ", udp.Ax)
			print("P : ", PID_controller.P)
			print("mov_vel : ", mov_vel)

			udp.Ax = mov_vel
			print("after Ax: ", udp.Ax)
			print("----------------------------")

			"""
			if PID_controller.P > 30:
				udp.Ax = 0
				udp.GearNo = 1
			"""

			"""
			elif PID_controller.cur_err > 0:
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				udp.Ax += PID_controller.acc_cmd

			elif PID_controller.cur_err < 0:
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				udp.Ax = PID_controller.acc_cmd
			"""


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