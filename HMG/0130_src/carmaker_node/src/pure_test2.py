"""
Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import rospy
from pid_controller import *
#from pure_list import *
from hellocm_msgs.msg import UDP
from hellocm_msgs.msg import sub_udp
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import sys, select, tty, termios
import numpy as np
import pandas as pd
import math

# Parameters
k = 1.5  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 1.1  # [s] time tick
WB = 2.7  # [m] wheel base of vehicle
PI = 3.141592

#whell_pos
Whell_FL = (3.85, 0.77525, 0.296)
Whell_FR = (3.85, -0.77525, 0.296)
whell_FM = (3.85, 0, 0.296)
Whell_RL = (1.15, 0.77525, 0.296)
Whell_RR = (1.15, -0.77525, 0.296)
whell_RM = (1.15, 0, 0.296)


class Controller(object):
    def __init__(self):
        self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
        self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
        self.car_pos = rospy.Subscriber('/Odometry', Odometry, self.pure_callback)
        
        # self.car_path = rospy.Subscriber('/path', PoseStamped, self.path_callback)
        
        
    def udp_callback(self, data_udp):    
        self.car_sta = data_udp
        self.cur_vel = float(self.car_sta.vx * 3.6)  # m/s -> km/h
        self.cur_yaw = float(self.car_sta.yaw)
        
        
    def pure_callback(self, data_pure):    
        self.car_point = data_pure
        self.car_point_x = self.car_point.pose.pose.position.x
        self.car_point_y = self.car_point.pose.pose.position.y
        

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)



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

   


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    #  target course
    rospy.init_node('controller')
    controller = Controller()
    PID_controller = pid_controller()

    udp = sub_udp()
    udp.GearNo = 1
    udp.Ax = 0
    #udp.SteeringWheel = 0
    #tar_vel_ = 10
    cur_vel_ = 0

    udp.VC_SwitchOn = 1

    df_TM = pd.read_csv('./file2.csv')
    pose_list = get_csv()
    list_cx = pose_list[1]
    list_cy = pose_list[2]
    
    ar_cx = np.array(list_cx)
    ar_cy = np.array(list_cy)
    # initial state

    state = State(x=controller.car_point_x, y=controller.car_point_y, yaw=controller.cur_yaw, v=cur_vel_)

    time = 0.0
    states = States()
    states.append(time, state)

    i = 0
    while(1):
        
        cx = ar_cx[i]
        cy = ar_cy[i]
        
        Lf = k * controller.cur_vel + Lfc  # update look ahead distance
        
        state = State(x=controller.car_point_x, y=controller.car_point_y, yaw=controller.cur_yaw, v=cur_vel_)
        
        
        
        tar_vel_ = 10
        cur_vel_ = controller.cur_vel
        mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_)
        
        alpha = math.atan2(cy - state.rear_y, cx - state.rear_x) - state.yaw
        
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)  
        
        print("delta = ", delta)

        udp.Ax = mov_vel
        udp.SteeringWheel = delta * (180/PI)
        print("udp.SteeringWheel = ", udp.SteeringWheel)
        print("udp.Ax = ", udp.Ax)
        state.update(mov_vel, delta)  # Control vehicle



        controller.car_pub.publish(udp)


        
        #print(delta)
        time += dt
        states.append(time, state)
        
        
        distance_this_index = state.calc_distance(cx, cy)
        
        print("distance_this_index ========", distance_this_index)
        
        distance_next_index = state.calc_distance(ar_cx[i + 5], ar_cy[i + 5])
        #print("distance_next_index ========", distance_next_index)
        
        
        
        if distance_this_index > distance_next_index:
            i = i+5
        
        if distance_this_index < 1.0 :
            i = i+1
        
        print("iiiiiiiiiiiiiiiiiiiiiiiiii = ",i)    
        
        
        
        
      
        
         
        '''
            if distance_this_index < distance_next_index:
                print("333333333333333333333333333333333333333333333333333")
                break
            i = i + 1 if (i + 1) < len(ar_cx[0:i]) else i
            distance_this_index = distance_next_index
        
              
        while distance_next_index > distance_this_index :
            i = i + 1   # not exceed goal
            break
        '''
        
        

        
        
        
        
        
        
        
        
        
        
   
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        




