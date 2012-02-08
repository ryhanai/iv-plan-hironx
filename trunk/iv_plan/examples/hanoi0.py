# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *
import hironx_motions

def putboxes(x, y, n=3):
    x,y = -200,100
    tbltop = env.get_object('table top')
    z0 = tbltop.vbody.size[2]/2.0
    
    for i in range(n):
        name = 'box'+str(n-i-1)
        env.delete_object(name)
        bdy = visual.box(length=97, height=66, width=57, color=(1,0,1))
        b = PartsObjectWithName(vbody=bdy, name=name)
        reltf = FRAME(xyzabc=[x,y,z0+b.vbody.size[1]*(i+1/2.0),pi/2,0,0])
        env.insert_object(b, reltf, tbltop)

def demo_hanoi(i, start, end):
    prepare_right()
    sync()
    target = 'box'+str(i)
    Tworld_obj = detect(target)
    asols,gsols = pl.reaching_plan(Tworld_obj, use_waist=True)
    (ayaw,a) = asols[0]
    (gyaw,g) = gsols[0]
    r.set_joint_angle(0, ayaw)
    r.set_arm_joint_angles(a)
    sync()
    r.set_joint_angle(0, gyaw)
    r.set_arm_joint_angles(g)
    sync(duration=2.0)
    grasp(affixobj=True, name=target)
    sync(duration=1.5)
    r.set_joint_angle(0, ayaw)
    r.set_arm_joint_angles(a)
    sync(duration=2.0)
    place_hanoi(traj_no=end, name=target)
        
places = {}

def start_hanoi(n=3):
    places['a'] = FRAME(xyzabc=[300,100,733,pi/2,0,0])
    places['b'] = FRAME(xyzabc=[300,-50,733,pi/2,0,0])
    places['c'] = FRAME(xyzabc=[300,-200,733,pi/2,0,0])
    x,y,_ = places['a'].vec
    putboxes(x,y,n)
    st0 = range(n)
    st0.reverse()
    hanoi(n-1, st= {"a":st0, "b":[], "c":[]})

def hanoi(n, start='a', end='b', work='c', st={'a':[2,1,0], 'b':[], 'c':[]}):
    if n >= 0:
        hanoi(n - 1, start, work, end, st)

        print st
        print "%d番目の円盤を%s[%d]から%s[%d]へ" % (n, start, len(st[start]), end, len(st[end]))
        r.set_joint_angles(hironx_motions.stretch_right_pose)
        sync(duration=2.0)

        #平行移動
        f_prepare = r.fk()
        f_prepare.vec[0] = places[start].vec[0] 
        f_prepare.vec[1] = places[start].vec[1]
        wyaw,avec = r.ik(f_prepare, use_waist=True)[0]
        r.set_joint_angle(0,wyaw)
        r.set_arm_joint_angles(avec)
        sync(duration=2.0)
        
        #物体へアプローチ
        f_start = FRAME(places[start])
        z_start = (len(st[start])-1) * 66
        f_start.vec[2] += z_start

        print 'FROM: ',
        print f_start

        asols,gsols = pl.reaching_plan(f_start, grasp_from_side=True, use_waist=True)
        (ayaw,a) = asols[0]
        (gyaw,g) = gsols[0]
        r.set_joint_angle(0,ayaw)
        r.set_arm_joint_angles(a)
        sync(duration=2.0)
        r.set_joint_angle(0, gyaw)
        r.set_arm_joint_angles(g)
        sync(duration=2.0)
        grasp(width=53,name='box'+str(n))
        sync(duration=1.0)

        # 持ち上げる
        f1 = r.fk()
        f1.vec[2] += 150
        wyaw,avec = r.ik(f1,use_waist=True)[0]
        r.set_joint_angle(0,wyaw)
        r.set_arm_joint_angles(avec)
        sync(duration=2.0)

        f2 = FRAME(places[end])
        f2.vec[2] += len(st[end]) * 66 + 5

        print 'TO: ',
        print f2

        asols,gsols = pl.reaching_plan(f2, grasp_from_side=True, use_waist=True)
        (ayaw,a) = asols[0]
        (gyaw,g) = gsols[0]
        r.set_joint_angle(0,ayaw)
        r.set_arm_joint_angles(a)
        sync(duration=2.5)
        r.set_joint_angle(0, gyaw)
        r.set_arm_joint_angles(g)
        sync(duration=2.5)
        
        release(width=80, name='box'+str(n))
        sync(duration=1.5)

        #アプローチ姿勢に一度戻る
        r.set_joint_angle(0,ayaw)
        r.set_arm_joint_angles(a)
        sync(duration=2.5)
        
        st[start].pop()
        st[end].append(n)
        print st

        hanoi(n - 1, work, end, start, st)

colored_print('1: start_hanoi()', 'blue')
