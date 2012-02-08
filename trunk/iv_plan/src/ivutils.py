# -*- coding: utf-8 -*-

import sys
import os
import random
import numpy
import scipy

import ivenv

from geo import *


## how to use Suehiro's geo package
## tools/geo/geo.py
# constructors
# VECTOR()
# VECTOR(vec=[1,2,3])
# I = MATRIX()
# MATRIX(mat=[[1,2,3],[4,5,6],[7,8,9]])
# FRAME(mat=m,vec=v)

# def Rx(th):
#     return MATRIX(angle=th, axis=XAXIS)
# def Ry(th):
#     return MATRIX(angle=th, axis=YAXIS)
# def Rz(th):
#     return MATRIX(angle=th, axis=ZAXIS)

def unzip(x):
    return zip(*x)

XAXIS=VECTOR(1,0,0)
YAXIS=VECTOR(0,1,0)
ZAXIS=VECTOR(0,0,1)

# norms & samplings

def mat2quat(m):
    tr = m[0][0] + m[1][1] + m[2][2] + 1.0
    if (tr >= 1.0):
        s = 0.5 / sqrt(tr)
        w = 0.25 / s
        x = (m[1][2] - m[2][1]) * s
        y = (m[2][0] - m[0][2]) * s
        z = (m[0][1] - m[1][0]) * s
        return [w, x, y, z]
    else:
        if (m[1][1] > m[2][2]):
            _max = m[1][1]
        else:
            _max = m[2][2]

        if (_max < m[0][0]):
            s = sqrt(m[0][0] - (m[1][1] + m[2][2]) + 1.0)
            x = s * 0.5
            s = 0.5 / s;
            y = (m[0][1] + m[1][0]) * s
            z = (m[2][0] + m[0][2]) * s
            w = (m[1][2] + m[2][1]) * s
            return [w, x, y, z]
        elif (_max == m[1][1]):
            s = sqrt(m[1][1] - (m[2][2] + m[0][0]) + 1.0)
            y = s * 0.5
            s = 0.5 / s;
            x = (m[0][1] + m[1][0]) * s
            z = (m[1][2] + m[2][1]) * s
            w = (m[2][0] + m[0][2]) * s
            return [w, x, y, z]
        else:
            s = sqrt(m[2][2] - (m[0][0] + m[1][1]) + 1.0)
            z = s * 0.5
            s = 0.5 / s;
            x = (m[2][0] + m[0][2]) * s
            y = (m[1][2] + m[2][1]) * s
            w = (m[0][1] + m[1][0]) * s
            return [w, x, y, z]


def quat2mat(q):
    qw,qx,qy,qz = q
    sx = qx * qx
    sy = qy * qy
    sz = qz * qz
    cx = qy * qz
    cy = qz * qx
    cz = qx * qy
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return MATRIX(mat=[[1.0-2.0*(sy+sz), 2.0*(cz+wz), 2.0*(cy-wy)],
                       [2.0*(cz-wz),1.0-2.0*(sx+sz),2.0*(cx+wx)],
                       [2.0*(cy+wy),2.0*(cx-wx),1.0-2.0*(sx+sy)]])


def distquat(q1, q2):
    l = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
    return 1.0 - abs(l)

def distSO3(m1, m2):
    return distquat(mat2quat(m1), mat2quat(m2))

def distRn(v1, v2):
    return numpy.linalg.norm(v1-v2)
    # scipy.minkowski_distance(v1, v2, 2)

def distSE3(frm1, frm2, wt=1.0, wr=1.8):
    v1 = frm1.vec
    m1 = frm1.mat
    v2 = frm2.vec
    m2 = frm2.mat
    return wt * distRn(v1, v2) + wr * distSO3(m1, m2)

def weighted_L1dist(v1, v2, w): # weighted L1 distance
    return reduce(lambda y,x: x[0]*abs(x[1]-x[2])+y, zip(w,v1,v2), 0.0)


# def normalize(v):
#     return v / linalg.norm(v)


def sampleUniformEuler():
    theta = 2*pi*random.random() - pi
    phi = acos(1-2*random.random()) + pi/2
    if random.random() < 0.5:
        if phi < pi:
            phi = phi + pi
        else:
            phi = phi - pi
    ita = 2*pi*random.random() - pi
    return [theta,phi,ita]

def sampleBoxR3(minx,maxx,miny,maxy,minz,maxz):
    return [random.uniform(minx, maxx),
            random.uniform(miny, maxy),
            random.uniform(minz, maxz)]

def sampleSE3():
    r = sampleUniformEuler()
    p = sampleBoxR3(WSMINX,WSMAXX,WSMINY,WSMAXY,WSMINZ,WSMAXZ)
    return FRAME(xyzabc=p+r)

def sampleSE3_with_z_constraint(R):
    theta = 2*pi*random.random() - pi
    m = MATRIX(angle=theta, axis=ZAXIS)
    p = sampleBoxR3(WSMINX,WSMAXX,WSMINY,WSMAXY,WSMINZ,WSMAXZ)
    return FRAME(mat=R*m, vec=p)

##
##

def parse_joints_flag(flag):
    if flag == 'rarm':
        use_waist = False
        arm = 'right'
    elif flag == 'torso_rarm':
        use_waist = True
        arm = 'right'
    elif flag == 'larm':
        use_waist = False
        arm = 'left'
    elif flag == 'torso_larm':
        use_waist = True
        arm = 'left'
    else:
        warn('joints %s is not supported'%flag)
        return None
    return arm, use_waist


##
## For fast nearest neighbor search
## not yet used
##

class DynamicKdTrees:
    def __init__(self):
        self.cache = []
        self.trees = []

    def query(self, point):
        mind = 1000000.0 # using squared distance is faster
        for p in self.cache:
            d = minkowski_distance(p, point)
            if d < mind:
                mind = d
                nnp = p
        print 'nearest in list: ', nnp
        d, idx = self.trees[0].query(point)
        if d < mind:
            mind = d
            nnp = self.trees[0].data[idx]
        print 'nearest in tree: ', nnp
        return mind, nnp

    def insert(self, point):
        if len(self.cache) < 20:
            self.cache.append(point)
        else:
            if self.trees == []:
                print 'creage a new tree'
                self.trees.append(cKDTree(self.cache))
            else:
                print 'extend a new tree'
                aa = self.trees[0].data.tolist()
                aa.extend(self.cache)
                self.trees[0] = cKDTree(aa)

        # for i in length(self.trees):
        #     if tree.n < 



##
## Simple primitive collision tests
##

#
# Only two primitive tests are implemented.
# 1, test between two line segments (equivalent to capsules)
# 2, test between a line segment (capsule) and AABB surface (not volume)
#

class CBody:
    def __init__(self):
        pass

class CCapsule(CBody):
    def __init__(self, pos=VECTOR(), axis=VECTOR(), radius=60.0):
        self.pos = pos
        self.axis = axis
        self.radius = radius

class CAABB(CBody):
    def __init__(self, bounds):
        self.bounds = bounds
        
class CollisionChecker:
    def __init__(self):
        pass

class SimpleCollisionChecker(CollisionChecker):

    def isCollisionFree(self, obj1, obj2):
        # collision test functions
        # return True if there is no collision
        if obj1.__class__ == CCapsule:
            if obj2.__class__ == CCapsule:
                return self.testTwoCapsules(obj1, obj2)
            elif obj2.__class__ == CAABB:
                return self.testCapsuleAABB(obj1, obj2)
            else:
                raise CCException('undefined primitive test pair')
        else:
            raise CCException('undefined primitive test pair')

    def testTwoCapsules(self, c1, c2):
        d = self.distanceLineSegments(c1.pos, c1.axis, c2.pos, c2.axis)
        return (c1.radius + c2.radius) < d

    def testCapsuleAABB(self, c, b):
        u = c.axis
        p0 = c.pos
        minx,maxx,miny,maxy,minz,maxz = b.bounds
        r = c.radius

        epsilon = 1e-3
        if abs(u[0]) > epsilon:
            tc = (minx-p0[0])/u[0]
            if 0 <= tc and tc <= 1:
                pc = p0 + tc*u
                if (miny-r <= pc[1] and pc[1] <= maxy+r
                    and minz-r <= pc[2] and pc[2] <= maxz+r):
                    return False
            tc = (maxx-p0[0])/u[0]
            if 0 <= tc and tc <= 1:
                pc = p0 + tc*u
                if (miny-r <= pc[1] and pc[1] <= maxy+r
                    and minz-r <= pc[2] and pc[2] <= maxz+r):
                    return False
        else:
            if abs(p0[0] - minx) < r or abs(p0[0] - maxx) < r:
                return False
            
        if abs(u[1]) > epsilon:
            tc = (miny-p0[1])/u[1]
            if 0 <= tc and tc <= 1:
                pc = p0 + tc*u
                if (minx-r <= pc[0] and pc[0] <= maxx+r
                    and minz-r <= pc[2] and pc[2] <= maxz+r):
                    return False
            tc = (maxy-p0[1])/u[1]
            if 0 <= tc and tc <= 1:
                pc = p0 + tc*u
                if (minx-r <= pc[0] and pc[0] <= maxx+r
                    and minz-r <= pc[2] and pc[2] <= maxz+r):
                    return False
        else:
            if abs(p0[1] - miny) < r or abs(p0[1] - maxy) < r:
                return False

        if abs(u[2]) > epsilon:
            tc = (minz-p0[2])/u[2]
            if 0 <= tc and tc <= 1:
                pc = p0 + tc*u
                if (minx-r <= pc[0] and pc[0] <= maxx+r
                    and miny-r <= pc[1] and pc[1] <= maxy+r):
                    return False
            tc = (maxz-p0[2])/u[2]
            if 0 <= tc and tc <= 1:
                pc = p0 + tc*u
                if (minx-r <= pc[0] and pc[0] <= maxx+r
                    and miny-r <= pc[1] and pc[1] <= maxy+r):
                    return False
        else:
            if abs(p0[2] - minz) < r or abs(p0[2] - maxz) < r:
                return False

        return True

    def distanceLineSegments(self, p0, u, q0, v):
        w0 = p0 - q0
        p1 = p0+u
        q1 = q0+v
        a = u.dot(u)
        b = u.dot(v)
        c = v.dot(v)
        d = u.dot(w0)
        e = v.dot(w0)
        if a*c - b*b > 1e-6:
            sc = (b*e - c*d)/(a*c - b*b)
            tc = (a*e - b*d)/(a*c - b*b)
        else:
            sc = 0
            tc = d / b

        pc = p0 + sc*u
        qc = q0 + tc*v

        if 0 <= sc and sc <= 1 and 0 <= tc <= 1:
            return (pc - qc).abs() # distance between 2 lines
        else:
            if sc < 0:
                s0 = 0
                t0 = e/c
                if 0 <= t0 and t0 <= 1:
                    return (p0-(q0+t0*v)).abs()
                else:
                    return min((p0-q0).abs(), (p0-q1).abs())
            if sc > 1:
                s0 = 1
                t0 = (e+b)/c
                if 0 <= t0 and t0 <= 1:
                    return (p1-(q0+t0*v)).abs()
                else:
                    return min((p1-q0).abs(), (p1-q1).abs())
            if tc < 0:
                t0 = 0
                s0 = - (d/a)
                if 0 <= s0 and s0 <= 1:
                    return ((p0+s0*u)-q0).abs()
                else:
                    return min((p0-q0).abs(), (p1-q0).abs())
            if tc > 1:
                t0 = 1
                s0 = (b-d)/a
                if 0 <= s0 and s0 <= 1:
                    return ((p0+s0*u)-q1).abs()
                else:
                    return min((p0-q1).abs(), (p1-q1).abs())


class CCError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


# pts = []
# for i in range(200):
#     pts.append(array([random.uniform(S1limit[0], S1limit[1]),
#                       random.uniform(S2limit[0], S2limit[1]),
#                       random.uniform(S3limit[0], S3limit[1]),
#                       random.uniform(E1limit[0], E1limit[1]),
#                       random.uniform(E2limit[0], E2limit[1]),
#                       random.uniform(W1limit[0], W1limit[1]),
#                       random.uniform(W2limit[0], W2limit[1])]))


# tree = CachedKdTrees()
# for i in range(100):
#     tree.insert([random.uniform(S1limit[0], S1limit[1]),
#                  random.uniform(S2limit[0], S2limit[1]),
#                  random.uniform(S3limit[0], S3limit[1]),
#                  random.uniform(E1limit[0], E1limit[1]),
#                  random.uniform(E2limit[0], E2limit[1]),
#                  random.uniform(W1limit[0], W1limit[1]),
#                  random.uniform(W2limit[0], W2limit[1])])

#tree.query([zeros(7)])


def rad2deg_trajectory(traj):
    def rad2deg_config(q):
        return map(rad2deg, q)
    path,tms = unzip(traj)
    return zip(map(rad2deg_config, path), tms)


###

colors = {
    'clear' : '\033[0m',
    'black' : '\033[30m',
    'red' : '\033[31m',
    'green' : '\033[32m',
    'yellow' : '\033[33m',
    'blue' : '\033[34m',
    'purple' : '\033[35m',
    'cyan' : '\033[36m',
    'white' : '\033[37m'
    }

def colored_print(msg, color):
    print '%s%s%s'%(colors[color], msg, colors['clear'])
    
def warn(msg):
    print '%s%s%s'%(colors['red'], msg, colors['clear'])

