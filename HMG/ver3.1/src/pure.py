#!/usr/bin/env python
import numpy as np
import math

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 2.0  # speed proportional gain
dt = 0.1  # [s] time tick

#whell_pos
Whell_FL = (3.85, 0.77525, 0.296)
Whell_FR = (3.85, -0.77525, 0.296)
whell_FM = (3.85, 0, 0.296)
Whell_RL = (1.15, 0.77525, 0.296)
Whell_RR = (1.15, -0.77525, 0.296)
whell_RM = (1.15, 0, 0.296)

WB = (3.85-1.15) # [m] wheel base of vehicle
#WB = 2.7


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw)) #Middle base of vehicle
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw)) #Middle base of vehicle

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


def proportional_control(target, current):
    a = Kp * (target - current)

    return a



class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy) # hypot : method of triangle
            ind = np.argmin(d) # minimum of d
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind


        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
	# steering wheel ang!

    return delta, ind


def main():
    #  target course
    cx = np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]
	# trans km/s

    T = 100.0  # max simulation time
	# ...??

    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)

"""

		// Odometry
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Odometry.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Odometry.Job), CMCRJob_RVStr(rv));
        } else if (Lidar_CycleCount % (int)CMNode.Topics.Pub.Odometry.CycleTime == 0){
            tf2::Quaternion rotation;
            CMNode.Topics.Pub.Odometry.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.Odometry.Msg.header.stamp = ros::Time(SimCore.Time);
            //CMNode.Topics.Pub.Odometry.Msg.pose.covariance =
            //CMNode.Topics.Pub.Odometry.Msg.twist.covariance =
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.x = InertialSensor[0].Pos_0[0];
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.y = InertialSensor[0].Pos_0[1];
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.position.z = InertialSensor[0].Pos_0[2];
            
            rotation.setRPY(Car.Roll, Car.Pitch, Car.Yaw);
            CMNode.Topics.Pub.Odometry.Msg.pose.pose.orientation = tf2::toMsg(rotation);
"""

"""
come data

"""
