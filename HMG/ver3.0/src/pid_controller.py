#!/usr/bin/env python

class pid_controller(object):

    def __init__(self):
        self.Kp = 2.0
        self.P = 0
        self.Ki = 0.01
        self.tar_vel = 0
        self.cur_vel = 0
        self.cur_err = 0
        self.acc_cmd = 0

    def p_control(self):
        self.P = self.Kp * self.cur_err
        return self.P

    def accel_control(self, tar_vel_, cur_vel_):
        self.tar_vel = tar_vel_
        self.cur_vel = cur_vel_
        self.cur_err = (self.tar_vel - self.cur_vel)
        self.p_control()
        return self.P