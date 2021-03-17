#!/usr/bin/env python
#import rospy
import time

class pid_controller(object):

    def __init__(self):
        self.Kp = 2.0
        self.P = 0
        self.Ki = 0.01
        #self.Kc = self.Ki * 0.3
        self.Kt = 0
        self.I = 0
        #self.awu = 0
        #self.Kd = 0.1
        #self.D = 0
        self.tar_vel = 0
        self.cur_vel = 0
        self.cur_err = 0
        self.acc_cmd = 0

        self.cur_time = time.time()
        self.last_time = self.cur_time
        self.del_time = 0


    def p_control(self):
        self.P = self.Kp * self.cur_err
        return self.P

    def pi_control(self):
        self.cur_time = time.time()
        self.del_time = self.cur_time - self.last_time

        self.Kt += self.cur_err * self.del_time
        self.I = self.Ki * self.Kt
        #self.I = self.I + (self.cur_err * self.Ki - self.awu * self.Kc)
        self.acc_cmd = self.p_control() + self.I
        #self.awu = (self.p_control() + self.I) - self.acc_cmd


    def accel_control(self, tar_vel_, cur_vel_):
        self.tar_vel = tar_vel_
        self.cur_vel = cur_vel_
        self.cur_err = (self.tar_vel - self.cur_vel)
        #self.p_control()
        self.pi_control()
        #return self.P
        return self.acc_cmd
