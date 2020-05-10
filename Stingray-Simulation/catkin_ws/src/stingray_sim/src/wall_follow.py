#!/usr/bin/env python

"""
Dominic Quintana
Wall_follow source file
"""

import rospy
import os
import numpy
import random
import pickle
from itertools import product
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
"""
Class for wall following robot
"""
class WallFollower:

    def current_model_state(self, md):
        """
        GET state of the model
        """

        pt = md.pose[1].position
        if pt.z > 0.25:
            self.reset()
            self.shouldReset = 1
            return
        pt_arr = numpy.array([pt.x, pt.y, pt.z])
        if self.pt is not None:
            consec = numpy.allclose(self.pt, pt_arr, 0, self.threshold_same)
            if consec:
                self.num_stuck += 1  # if stuck, increment number of times stuck
            if self.num_stuck > self.threshold_consec:
                self.shouldReset = 1
                self.num_stuck = 0
            self.pt = pt_arr
        else:
            self.pt = pt_arr
            return

    def get_distance(self, var):
        l = var.ranges[179] # get left distance
        r = var.ranges[0]   # get right distance
        f = var.ranges[89]  # get front distance
        self.current = [2, 2, 2]
        distances = [l, f, r]
        for i, dist in enumerate(distances):
            for j in range(len(self.thresholds[0])):
                if dist <= self.thresholds[i][j]:
                    self.current[i] = j
                    break
        self.st = tuple(self.current)

    def __init__(self):
        """
        initialize variables, etc.
        """
        rospy.init_node('follower', anonymous=True)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.get_distance)
        # subscriber for the model state
        self.subscriber_mod = rospy.Subscriber('/gazebo/model_states', ModelStates, self.current_model_state)
        self.publisher = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
        self.reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo_unpause_physics', Empty)
        self.rate = rospy.Rate(10) # 10Hz

        self.st = (2, 2, 2)
        self.st_prev = (2, 2, 2)    # previous state
        self.current = [2, 2, 2]
        self.current_action = 1
        self.poses = {0: (0, 2, 5), 1: (0, 2, 0), 2: (0, 2, -5)}
        self.thresholds = {0: [0.4, 1.0], 1: [0.4, 1.4], 2: [0.2, 0.4]}

        self.pt = None
        self.episodes = 0
        self.shouldReset = 0
        self.num_stuck = 0
        self.threshold_consec = 25  # threshold consecutive
        self.threshold_same = 0.0015    # same state threshold
        # set up Q-table variables - [(left or near), (forward or middle), (right or far)]
        actions = [0, 1, 2]
        substate = [0, 1, 2]
        state = [sta for sta in product(substate, substate, substate)]
        cont = 1    # continue
        move_toward = 2
        away = 3
        turn = 4

        self.eps = 0
        self.alpha = 0.2
        self.gamma = 0.75

        self.eps_init = 0.9 # initial epsilon
        self.eps_decay = 0.99

        self.q = {(sta, act): 0 for act in actions for sta in state} # Q-table
        for tup in self.q.keys():
            if tup[1] == 0 and tup[0][2] == 0:
                self.q[tup] = away
            if tup[1] == 2 and tup[0][0] == 0:
                self.q[tup] = away
            if tup[1] == 1 and tup[0][2] == 1:
                self.q[tup] = cont
            if tup[1] == 2 and tup[0][2] == 2:
                self.q[tup] = move_toward
            if tup[0][0] == 2 and tup[0][1] == 2 and tup[0][2] == 2:
                self.q[tup] = cont
            if tup[1] == 0 and tup[0][1] < 2:
                self.q[tup] = turn

    def train(self):
        while not rospy.is_shutdown():
            next = 1
            max_num = 0
            self.pause_physics

            for tup in self.q.keys():
                if tup[0] == self.st and self.q[tup] > max_num:
                    next = tup[1]
                    max_num = self.q[tup]

            decisions = {}
            for tup in self.q.keys():
                if tup[0] == self.st:
                    decisions[tup] = self.q[tup]
            if random.random() < self.eps:
                next = max(decisions, key=decisions.get)[1]
            else:
                next = random.choice(decisions.keys())[1]

            self.st_prev = self.st
            self.current_action = next
            pose = Pose2D()
            pose.x, pose.y, pose.theta = self.poses[next]
            self.publisher.publish(pose)
            self.rate.sleep()

            # find out reward
            reward = 0
            if self.st[0] == 0 or self.st[1] == 0:
                reward = -1
            if self.st[2] == 1 and (self.st[1] > 0 or self.st[1] < 0):
                reward = 1
            # update the Q-table
            current_table = self.q[(self.st_prev, self.current_action)]
            highest_q = max([tup for tup in self.q if tup[0] == self.st], key=self.q.get)
            new_q = current_table + self.alpha * (reward + (self.gamma * self.q[highest_q]) - current_table)
            self.q[(self.st_prev, self.current_action)] = new_q

            self.unpause_physics
            if self.episodes > 9500:
                return
            if self.shouldReset:
                with open(os.path.abspath(os.path.dirname(__file__)) + '/qtable_train', 'wb') as file:
                    pickle.dump(wall_follower.q, file)
                self.eps = self.eps_init * (1 -(self.eps_decay ** self.episodes))
                self.episodes += 1
                #self.reset()
                self.shouldReset = 0
                self.num_stuck = 0


    def wall_follow(self):
        """
        Function for robot to follow wall
        """
        while not rospy.is_shutdown():
            next = 1
            max_num = 0
            for tup in self.q.keys():
                if tup[0] == self.st and self.q[tup] > max_num:
                    next = tup[1]
                    max_num = self.q[tup]

            self.st_prev = self.st
            self.current_action = next
            pose = Pose2D()
            pose.x, pose.y, pose.theta = self.poses[next]
            self.publisher.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    wall_follower = WallFollower()
    option = -1
    while option not in [0, 1]:
        option = int(raw_input("Input 0 for Training; Input 1 for Following"))
        if option != 0 or option != 1:
            continue
    if option == 0:
        try:
            with open(os.path.abspath(os.path.dirname(__file__)) + '/qtable_train', 'rb') as file:
                wall_follower.q = pickle.load(file)
            wall_follower.wall_follow()
        except OSError:
            print "Cannot locate Q-Table"
    else:
        wall_follower.train()
