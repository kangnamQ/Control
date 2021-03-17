#!/usr/bin/env python
import rospy
from pid_controller import *
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
	mov_vel = 0

	udp.VC_SwitchOn = 1
	udp.GearNo = 1
	"""
	1*3.6m/s = 3.6km/h
	tar_vel = km/h
	cur_vel = m/s
	cur_err = tar_vel - cur_vel*3.6 = km/h
	"""

	try:
		while (1):
			key = getKey()

			if key == 't':
				udp.VC_SwitchOn = 1
				udp.GearNo = 1
				tar_vel_ = float(input("target value : "))
				print("set", tar_vel_, "km/h")
				mod_switch = 1

			cur_vel_ = controller.cur_vel
			mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
			udp.Ax = mov_vel

			print("If you change the target_value : push the 't'")
			print("target_value : ", tar_vel_)
			print("current_value : ", cur_vel_)
			print("Ax_value : ", udp.Ax)
			# print("P : ", PID_controller.P)
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