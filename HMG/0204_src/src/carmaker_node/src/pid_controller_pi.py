#!/usr/bin/env python
import rospy
import numpy as np

class pid_controller(object):

    def __init__(self):
        self.Kp = 3.0
        self.Ki = 0.01
        self.Kd = 0.05
        self.P = 0
        self.I = 0
        self.D = 0
        # self.Kc = self.Ki * 0.3
        # self.Kt = 0
        # self.awu = 0
        self.tar_vel = 0
        self.cur_vel = 0
        self.cur_err = 0
        self.acc_cmd = 0

        self.dt = 0
        self.last_err = 0
        self.d_error = 0
        self.d_input = 0
        self.last_input = 0



    def p_control(self):
        self.P = self.Kp * self.cur_err
        return self.P

    def pi_control(self):
        self.I = self.I + (self.Ki * self.cur_err * self.dt)
        return self.I


        # self.acc_cmd = self.p_control() + self.I

        # self.I = self.I + (((self.Kp + (self.Ki * self.dt)) * self.cur_err) - (self.Kp * self.last_err))
        # self.last_err = self.cur_err
        # self.I = self.I + (self.cur_err * self.Ki - self.awu * self.Kc)
        # self.acc_cmd = self.p_control() + self.I
        # self.awu = (self.p_control() + self.I) - self.acc_cmd

    def pid_control(self):
        # self.d_error = self.cur_err - self.last_err
        # self.D = self.Kd * (self.d_error / self.dt)
        # self.last_err = self.cur_err

        self.d_input = self.cur_vel - self.last_input
        try:
            self.D = -self.Kd * (self.d_input / self.dt)
        except ZeroDivisionError:
            self.D = 0
        self.last_input = self.cur_vel

        self.acc_cmd = self.p_control() + self.pi_control() + self.D
        return self.acc_cmd




    def accel_control(self, tar_vel_, cur_vel_, dt):
        self.dt = dt
        self.tar_vel = tar_vel_
        self.cur_vel = cur_vel_
        self.cur_err = (self.tar_vel - self.cur_vel)
        # self.p_control()
        # self.pi_control()

        self.pid_control()
        # return self.P

        return self.acc_cmd

    def rate_limiter(self,value,prev_value,max_change):
        delta = value - prev_value
        delta = np.clip(delta,-max_change,max_change)
        return prev_value + delta


"""
Error = setPoint - Input
PTerm = Kp * Error
ITerm += Ki * Error * dt
dinput = input - previnput
DTerm = -Kd *(kinput / dt)
Output = PTerm + ITerm + DTerm
"""