# -*- coding: utf-8 -*-

import sys
import os
import time
import re
import operator

from numpy import *
from scipy import *
from scipy.spatial import *

from ivutils import *
from viewer import *
from mplan_env import *

class State:
    def __init__(self, parent=None, avec=zeros(6)):
        self.parent = parent
        self.cntrl = None
        self.avec = avec

    # def __repr__(self):
    #     return '<%s %s>'%(self.__class__, self.avec)

    # def __str__(self):
    #     return self.__repr__()

class Node:
    def __init__(self, parent=None, q=zeros(6), x=FRAME()):
        self.parent = parent
        self.q = q
        self.x = x

def random_interval(trajlen):
    # choose 2 nodes randomly
    l = min(trajlen/2, 8)
    while True:
        m = random.randint(0, trajlen-1)
        n = random.randint(0, trajlen-1)
        if abs(m - n) < l:
            continue
        if m > n:
            tmp = m
            m = n
            n = tmp
        #print 'm,n=%d,%d'%(m,n) # try to connect 2 nodes
        return m,n

def inflection_points(traj, thre=0.85):
    trajlen = len(traj)
    for i in range(trajlen-1):
        traj[i].parent = traj[i+1]

    dp = [traj[i].avec-traj[i+1].avec for i in range(len(traj)-1)]
    dp = [dp[i]/linalg.norm(dp[i]) for i in range(len(dp))]
    return [abs(dot(dp[i],dp[i+1])) < thre for i in range(len(dp)-1)]


class CSPlanner():
    def __init__(self, env, cc=None):
        self.maxTime = 1.0
        self.robot = env.get_robot()
        self.env = env
        self.cc = cc
        self.maxIter = 300
        self.epsilon = 50.0
        self.use_sampling_heuristics = False
        self.poses = []

    def initialize(self, joints='rarm'):
        # joints ::= rarm | larm | torso_rarm | torso_larm | torso_arms | all
        for pose in self.poses:
            pose.set_visible(False)
        self.poses = []
        self.cccnt = 0
        self.cctm = self.searchtm = self.optimizetm = 0.0

        self.joints = joints
        if joints == 'rarm':
            self.js = self.robot.joints[3:9]
        elif joints == 'larm':
            self.js = self.robot.joints[9:15]
        elif joints == 'torso_rarm':
            self.js = [self.robot.joints[0]]+self.robot.joints[3:9]
        elif joints == 'torso_larm':
            self.js = [self.robot.joints[0]]+self.robot.joints[9:15]
        elif joints == 'torso_arms':
            self.js = [self.robot.joints[0]]+self.robot.joints[3:15]
        else:
            self.js = self.robot.joints

        self.sllimits = array([j.sllimit for j in self.js])
        self.sulimits = array([j.sulimit for j in self.js])
        self.weights = array([j.weight for j in self.js])

    def random_state(self):
        return State(avec=numpy.random.random_sample(len(self.js))
                     * (self.sulimits-self.sllimits) + self.sllimits)

    def ws_sample_state(self):
        warn('not yet implemented')
        pass

    def print_statistics(self):
        colored_print('total time %f[sec]'%(self.searchtm + self.optimizetm), 'cyan')
        colored_print('search %f[sec]'%self.searchtm, 'cyan')
        colored_print('optimize %f[sec]'%self.optimizetm, 'cyan')
        colored_print('collision check %d[times], %f[sec]'%(self.cccnt, self.cctm), 'cyan')

    def try_connect(self, q_start, q_target, joints='rarm'):
        self.initialize(joints=joints)
        return self.connect([State(avec=array(q_start))], State(avec=array(q_target))) == 'reached'

    def make_plan(self, q_start, q_target, q_targets=[], joints='rarm'):
        self.initialize(joints=joints)
        traj,oneshot = self.search(q_start, q_targets if len(q_targets)>0 else [q_target])
        if traj:
            if oneshot:
                result = traj
            else:
                opttraj = self.optimize_trajectory(traj)
                result = opttraj
        else:
            result = None
        self.print_statistics()
        return result
    
    def search(self, q_start, q_targets):
        self.T_init = [State(avec=array(q_start))]
        self.T_goal = [State(avec=array(q)) for q in q_targets]

        tm_start = time.time()
        isConnected, isFirstTry = self.expandTree()
        if isConnected:
            traj = []
            nd = self.T_init[-1]
            while nd:
                traj.append(nd)
                nd = nd.parent
            traj.reverse()
            nd = self.T_goal[-1]
            while nd:
                traj.append(nd)
                nd = nd.parent

            self.searchtm = time.time() - tm_start
            return traj, isFirstTry
        return None, False

    def expandTree(self):
        T_a = self.T_init
        T_b = self.T_goal

        for i in range(self.maxIter):
            if self.use_sampling_heuristics:
                q_rand = self.ws_sample_state()
            else:
                q_rand = self.random_state()

            if i == 0:
                T_from = self.T_init
                q_to = self.T_goal[0]
                if self.connect(T_from, q_to) == 'reached':
                    return True, True
            elif i < 3:
                if self.joints != 'all' and self.joints != 'torso_arms':
                    l = 150
                    if i == 1:
                        T_from = self.T_goal
                        T_to = self.T_init
                        q = self.T_goal[0]
                        self.robot.set_joint_angles(q.avec, joints=self.joints)
                        l_or_r,use_waist = parse_joints_flag(self.joints)
                        f = self.robot.fk(l_or_r) * FRAME(vec=[l,0,0])
                        q_to = State(avec = self.robot.ik(f, joints=self.joints)[0])
                        if self.connect(T_from, q_to) == 'reached':
                            if self.connect(T_to, q_to) == 'reached':
                                return True, False
                    else:
                        T_from = self.T_init
                        T_to = self.T_goal
                        q = self.T_init[0]
                        self.robot.set_joint_angles(q.avec, joints=self.joints)
                        l_or_r,use_waist = parse_joints_flag(self.joints)
                        f = self.robot.fk(l_or_r) * FRAME(vec=[l,0,0])
                        q_to = State(avec = self.robot.ik(f, joints=self.joints)[0])
                        if self.connect(T_from, q_to) == 'reached':
                            if self.connect(T_to, q_to) == 'reached':
                                return True, False

            else:
                if self.extend(T_a, q_rand) != 'trapped':
                    q_new = T_a[-1]
                    if self.connect(T_b, q_new) == 'reached':
                        return True, False

                T_tmp = T_a
                T_a = T_b
                T_b = T_tmp
                            
        return False, False

    def extend(self, T, q_to):
        q_near = self.nearestNeighbor(q_to, T)
        q_new, dws = self.newState(q_near, q_to)
        if q_new == 'trapped':
            return q_new
        else:
            q_new.parent = q_near
            T.append(q_new)
            if dws < self.epsilon * 1.5:
                return 'reached'
            else:
                return 'advanced'

    def connect(self, T, q_to, maxIter=100):
        state = 'advanced'
        i = 0
        while state == 'advanced':
            # print self.dist(q_from, q_to)
            state = self.extend(T, q_to)
            if i == maxIter:
                break
            i += 1
        return state

    def nearestNeighbor(self, q_in, T):
        #return min(T, key=lambda q: distRn(q.avec, q_in.avec))
        return min(T, key=lambda q: self.weightedDistance(q.avec, q_in.avec))

    def dist(self, q1, q2):
        return distRn(q1.avec, q2.avec)

    def weightedDistance(self, q1, q2):
        return self.weightedNorm(q1 - q2)

    def weightedNorm(self, dq):
        dws = 0.0
        if self.joints == 'all':
            dws1 = 0.0
            for i in range(3,9):
                dws1 += self.weights[i] * abs(dq[i])
            dws2 = 0.0
            for i in range(9,15):
                dws2 += self.weights[i] * abs(dq[i])
            dws = max(dws1, dws2) + self.weights[0] * abs(dq[0])
        elif self.joints == 'torso_arms':
            dws1 = 0.0
            for i in range(1,7):
                dws1 += self.weights[i] * abs(dq[i])
            dws2 = 0.0
            for i in range(7,13):
                dws2 += self.weights[i] * abs(dq[i])
            dws = max(dws1, dws2) + self.weights[0] * abs(dq[0])
        else:
            for i in range(len(dq)):
                dws += self.weights[i] * abs(dq[i])
        return dws

    def newState(self, q_near, q_to):
        v = q_to.avec - q_near.avec
        dws = self.weightedNorm(v)

        if dws < self.epsilon*0.1:
            return 'trapped', dws

        dvec = self.epsilon * v / dws
        st = State(avec = q_near.avec + dvec)
        if self.feasibleState(st):
            return st, dws
        else:
            return 'trapped', dws

    def feasibleState(self, q):
        for i in range(len(q.avec)):
            self.js[i].angle = q.avec[i]
        self.robot.refresh()

        t1 = time.time()
        cstate = self.robot.in_collision()
        t2 = time.time()
        self.cccnt += 1
        self.cctm += t2-t1

        # if cstate:
        #     print 'collision: ',
        #     print self.robot.get_arm_joint_angles()
        return not cstate

    def optimize_trajectory(self, traj):
        t1 = time.time()
        traj = self.smooth(self.random_shortcut(traj, tm=2.0))
        self.optimizetm = time.time() - t1
        return traj

    def random_shortcut(self, traj, tm=3.0):
        t1 = time.time()
        trajlen = len(traj)        
        for i in range(trajlen-1):
            traj[i].parent = traj[i+1]

        while True:
            t2 = time.time()
            if t2 - t1 > tm:
                break
            m,n = random_interval(trajlen)
            trajlen = self.try_shortcut(traj, m, n, trajlen)

        traj2 = []
        q = traj[0]
        while q:
            traj2.append(q)
            q = q.parent

        return traj2

    def try_shortcut(self, traj, m, n, trajlen):
        q = traj[0]
        for j in range(trajlen):
            if j == m:
                q_from = q
            if j == n:
                q_to = q
            q = q.parent

        q0 = q_from
            
        while True:
            q_new, dws = self.newState(q_from, q_to)
            if q_new == 'trapped':
                return trajlen
            else:
                if dws < self.epsilon * 1.5:
                    trajlen = trajlen - (n - m -1)
                    trajlen -= 1
                    while q_to != q0:
                        q = q_from.parent
                        q_from.parent = q_to
                        q_to = q_from
                        q_from = q
                        trajlen += 1
                    # print 'new length = %d'%trajlen
                    return trajlen
                else:
                    q_new.parent = q_from
                    q_from = q_new

    def smooth(self, traj):
        a = 3
        trajlen = len(traj)
        for k in range(5):
            is_shortened = False
            ifpts = inflection_points(traj)
            i = len(ifpts)-2
            while i >= a:
                if ifpts[i]:
                    m = i-a
                    n = i+a
                    #colored_print('k,m,n=%d,%d,%d'%(k,m,n), 'red')
                    newtrajlen = self.try_shortcut(traj, m, n, trajlen)
                    if newtrajlen < trajlen:
                        is_shortened = True                    
                        i -= a
                    else:
                        i -= 1
                    trajlen = newtrajlen
                else:
                    i -= 1

            traj2 = []
            q = traj[0]
            while q:
                traj2.append(q)
                q = q.parent
            traj = traj2
            
            if not is_shortened:
                break

        return traj
