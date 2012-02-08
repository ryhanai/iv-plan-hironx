#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import *
from numpy import *
import operator

#from set_env import *
import libcpqp as cpqp

def build_model(debug=False):
    b1 = cpqp.PQP_Model()
    b2 = cpqp.PQP_Model()

    if debug:
        print("loading tris into PQP_Model objects...")

    a = 1.0
    b = 0.2
    n1 = 50
    n2 = 50
    count = 0

    b1.BeginModel(8)
    b2.BeginModel(8)

    for uc in range(n1):
        for vc in range(n2):
            u1 = (2.0*pi*uc)/n1
            u2 = (2.0*pi*(uc+1))/n1
            v1 = (2.0*pi*vc)/n2
            v2 = (2.0*pi*(vc+1))/n2

            p1 = [(a-b*cos(v1))*cos(u1),(a-b*cos(v1))*sin(u1),b*sin(v1)]
            p2 = [(a-b*cos(v1))*cos(u2),(a-b*cos(v1))*sin(u2),b*sin(v1)]
            p3 = [(a-b*cos(v2))*cos(u1),(a-b*cos(v2))*sin(u1),b*sin(v2)]
            p4 = [(a-b*cos(v2))*cos(u2),(a-b*cos(v2))*sin(u2),b*sin(v2)]

            b1.AddTri(p1,p2,p3,count)
            b1.AddTri(p4,p2,p3,count+1)
            b2.AddTri(p1,p2,p3,count)
            b2.AddTri(p4,p2,p3,count+1)
            count += 2

    if debug:
        print("done")
        print("Tri have %d triangles each." % count)
        print("building hierarchies...")
    b1.EndModel()
    b2.EndModel()

    if debug:
        print("done.")

    b1.MemUsage(1)
    b2.MemUsage(2)

    return b1,b2

def m2l(m):
    return reduce(operator.__add__, m.tolist())

def collide(R1, T1, b1, R2, T2, b2, contact_type=cpqp.contact_type.all):
    cres = cpqp.PQP_CollideResult()
    cpqp.PQP_Collide(cres, m2l(R1), T1.tolist(), b1, m2l(R2), T2.tolist(), b2, contact_type)
    return cres

def distance(R1, T1, b1, R2, T2, b2):
    dres = cpqp.PQP_DistanceResult()
    cpqp.PQP_Distance(dres, m2l(R1), T1.tolist(), b1, m2l(R2), T2.tolist(), b2, 0.0, 0.0)
    return dres

def print_collide_result(cres, listPairs=False):
    print "Num BV tests: %d" % cres.NumBVTests()
    print "Num Tri tests: %d" % cres.NumTriTests()
    print "Num contact pairs: %d" % cres.NumPairs()
    if listPairs:
        for i in range(cres.NumPairs()):
            print "contact %4d: tri %4d and tri %4d" % (i, cres.Id1(i), cres.Id2(i))

def print_distance_result(dres):
    print "Num BV tests: %d" % dres.NumBVTests()
    print "Num Tri tests: %d" % dres.NumTriTests()
    print "Distance: %f" % dres.Distance()

def queries(b1, b2):
    R1 = identity(3)
    R2 = identity(3)
    T1 = array([1.0,0.0,0.0])
    T2 = array([0.0,0.0,0.0])
    print "All contact collision query between overlapping tori:"
    print_collide_result(collide(R1, T1, b1, R2, T2, b2))
    print "First contact collision query between overlapping tori:"
    print_collide_result(collide(R1, T1, b1, R2, T2, b2, cpqp.contact_type.first))
    print "Distance query between overlapping tori:"
    print_distance_result(distance(R1, T1, b1, R2, T2, b2))

    R1 = array([[1,0,0],
                [0,0,-1],
                [0,1,0]])
    print "Collision query between interlocked but nontouching tori:"
    print_collide_result(collide(R1, T1, b1, R2, T2, b2, cpqp.contact_type.first))
    print "Distance query between interlocked but nontouching tori:"
    print_distance_result(distance(R1, T1, b1, R2, T2, b2))
    

def main():
    b1,b2 = build_model()
    queries(b1, b2)
    

if __name__ == '__main__':
    main()
    
# each parts should have
#  several faces
#  (R,T) in R^3

# Preparation:
#
# cc = CollisionChecker()
# lnks = r.links()
# cc.add_pair(lnks[0], lnks[2])
# cc.add_pair(lnks[1], lnks[3])
# cc.list_pairs()

# Check:
#
# for lnk in links:
#     lnk.update()
# t_or_f = cc.check_fast()

