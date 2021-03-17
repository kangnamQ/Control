#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt


#whell_pos
Whell_FL = (3.85, 0.77525, 0.296)
Whell_FR = (3.85, -0.77525, 0.296)
whell_FM = (3.85, 0, 0.296)
Whell_RL = (1.15, 0.77525, 0.296)
Whell_RR = (1.15, -0.77525, 0.296)
whell_RM = (1.15, 0, 0.296)
rear_w = 1.15
front_w = 3.85
WB = (3.85-1.15) # [m] wheel base of vehicle
#WB = 2.7


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.rear_x = self.x + (rear_w * math.cos(self.yaw)) # rear wheel base of vehicle
        self.front_x = self.rear_x + (WB * math.cos(self.yaw)) # front wheel base of vehicle
        self.rear_y = self.y + (rear_w * math.sin(self.yaw)) # rear wheel base of vehicle
        self.front_y = self.rear_y + (WB * math.sin(self.yaw)) # front wheel base of vehicle

        self.car_dx = self.front_x - self.rear_x
        self.car_dy = self.front_y - self.rear_y

        self.turn = 0
        self.dt = 0


    def update(self, a, delta, dt):
        self.dt = dt
        delta_rad = delta * (math.pi / 180)
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / WB * math.tan(delta_rad) * self.dt

        self.v += a * self.dt
        self.rear_x = self.x + (rear_w * math.cos(self.yaw)) # rear wheel base of vehicle
        self.front_x = self.rear_x + (WB * math.cos(self.yaw)) # front wheel base of vehicle
        self.rear_y = self.y + (rear_w * math.sin(self.yaw)) # rear wheel base of vehicle
        self.front_y = self.rear_y + (WB * math.sin(self.yaw)) # front wheel base of vehicle


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


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = np.array(cx)
        self.cy = np.array(cy)
        self.old_nearest_point_index = None

        self.cx_list = []
        self.cy_list = []


    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            for ic in range(len(self.cx)):
                dx = [state.rear_x - self.cx[ic]]
                self.cx_list.append(dx)
                dy = [state.rear_y - self.cy[ic]]
                self.cy_list.append(dy)
            d = np.hypot(self.cx_list, self.cy_list) # hypot : method of triangle
            ind = np.argmin(d) # minimum of d

            self.old_nearest_point_index = ind
            k = 0.1
            # print("old_point : ", self.old_nearest_point_index)
        else:
            k = 0.3
            ind = self.old_nearest_point_index
            # print("else : ", ind, "x : ", self.cx[ind], "y : ",self.cy[ind])

            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])

            while True:
                if state.turn == 1:
                    print("turn")
                    try:
                        distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                                 self.cy[ind + 1])
                        k = 0.1
                    except IndexError:
                        distance_next_index = state.calc_distance(self.cx[(len(self.cx)-1)],
                                                                 self.cy[(len(self.cx)-1)])
                else:

                    if state.v > 49:
                        try:
                            distance_next_index = state.calc_distance(self.cx[ind + 4],
                                                                     self.cy[ind + 4])
                            k = 0.4
                        except IndexError:
                            distance_next_index = state.calc_distance(self.cx[(len(self.cx)-1)],
                                                                     self.cy[(len(self.cx)-1)])

                    elif state.v > 29:
                        try:
                            distance_next_index = state.calc_distance(self.cx[ind + 3],
                                                                     self.cy[ind + 3])
                            k = 0.2
                        except IndexError:
                            distance_next_index = state.calc_distance(self.cx[(len(self.cx)-1)],
                                                                     self.cy[(len(self.cx)-1)])
                    else:
                        try:
                            distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                                     self.cy[ind + 1])
                            k = 0.3
                        except IndexError:
                            distance_next_index = state.calc_distance(self.cx[(len(self.cx)-1)],
                                                                     self.cy[(len(self.cx)-1)])
                            print("a")

                if distance_this_index < distance_next_index:
                    break

                if (ind + 1) < len(self.cx):
                    ind = ind + 1
                if ind >= (len(self.cx)-1):
                    ind = (len(self.cx)-1)
                    break

                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        # # Parameters
        # k = 0.00001  # look forward gain
        Lfc = 2.0  # [m] look-ahead distance
        Lf = k * state.v + Lfc  # update look ahead distance


        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                ind = (len(self.cx)-1)
                break  # not exceed goal
            ind += 1

        Ld = state.calc_distance(self.cx[ind], self.cy[ind])
        return ind, Lf, Ld


    def search_target(self, state):
        Ld_dx = self.target_x - state.rear_x
        Ld_dy = self.target_y - state.rear_y

        car_fr = math.atan2(state.car_dy, state.car_dx)
        car_pr = math.atan2(Ld_dy, Ld_dx)
        alpha_m = (car_fr - car_pr)
        alpha_m_d = alpha * (180 / math.pi)

        Ld_m = math.sqrt((Ld_dx ** 2) + (Ld_dy ** 2))

        car_alpha_m = alpha - state.yaw
        delta_m = math.atan2(2.0 * WB * math.sin(alpha_m) / Ld, 1.0)

        return Ld_m, alpha_m



def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf, Ld = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    # yaw = deg, alpha = radian
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta_rad = math.atan2(2.0 * WB * math.sin(alpha) / Ld, 1.0)

    delta = delta_rad * (180 / math.pi)

    Ks = 0.06 #pure pursuit gain
    delta_gain = delta * Ks
    # steering wheel ang! = deg
    # delta = deg

    Ld_dx = tx - state.rear_x
    Ld_dy = ty - state.rear_y

    car_fr = math.atan2(state.car_dy, state.car_dx)
    car_pr = math.atan2(Ld_dy, Ld_dx)
    alpha_m = (car_pr - car_fr)
    alpha_m_d = alpha_m * (180 / math.pi)

    Ld_m = math.sqrt((Ld_dx ** 2) + (Ld_dy ** 2))

    car_alpha_m = alpha_m - state.yaw
    delta_m_rad = math.atan2(2.0 * WB * math.sin(alpha_m) / Ld_m, 1.0)
    delta_m = delta_m_rad * (180 / math.pi)
    return delta, ind, delta_rad, delta_gain, alpha, alpha_m, car_alpha_m, delta_m, Ld_m



def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)
