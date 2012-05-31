#!/usr/bin/env python
# -*- coding: utf-8 -*-

def demo():
    i = 0
    while i < 2:
        prepare_right()
        sync()
        target = 'box'+str(i)
        try:
            Tworld_obj = detect(target)
            asols,gsols = pl.reaching_plan(Tworld_obj, use_waist=True)
        except:
            continue
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
        place(traj_no=i, name=target)

        i += 1


def place(traj_no=0, name='box0'):
    r.set_joint_angle(0, 0.25)
    r.set_arm_joint_angles(r.ik(motions.pltraj[traj_no][0])[0])
    sync()
    r.set_arm_joint_angles(r.ik(motions.pltraj[traj_no][1])[0])
    sync(duration=3.0)
    r.set_arm_joint_angles(r.ik(motions.pltraj[traj_no][2])[0])
    sync(duration=2.0)
    release(unfixobj=True, name=name)
    sync(duration=1.5)
    r.set_arm_joint_angles(r.ik(motions.pltraj[traj_no][1])[0])
    sync(duration=2.0)
    prepare_right()
    sync(duration=3.0)


def gen_traj():
    traj = CoordinateObjects('trajectory')

    # 手先軌道を作る
    ys = arange(-150, -350, -20)
    xs = [50*sin(pi*(y-150)/100)-250 for y in ys]
    z = 900
    for x,y in zip(xs,ys):
        f = FRAME(mat=MATRIX(angle=-pi/2,axis=VECTOR(vec=[0,1,0])), vec=[x+500,y,z])
        traj.append(f)
    env.insert_object(traj, FRAME(), env.get_world())

    # 関節角軌道を作る
    qs = []
    prepare_right()
    for wpfrm in traj.coords:
        js = r.ik([wpfrm.where()])[0]
        r.set_arm_joint_angles(js)
        qs.append(r.get_joint_angles())

    return qs

def play_traj(qs, waitkey=False, duration = 0.5, interval = 0.5):
    # 軌道を再生する
    isFirst = True
    for q in qs:
        print q
        r.set_joint_angles(q)
        if isFirst:
            sync(duration=3.0) # 最初は時間を長めにとる
            isFirst = False
        else:
            sync(duration=duration, wait=False, waitkey=waitkey)
            #time.sleep(interval)

def demo2():
    '''左手でアプローチするデモ'''
    r.set_joint_angles(r.poses['prepare'])
    sync()
    try:
        Tworld_obj = detect()
        asols,gsols = pl.reaching_plan(Tworld_obj,
                                       arm='left', use_waist=True)
    except:
        return
    (ayaw,a) = asols[0]
    (gyaw,g) = gsols[0]
    r.set_joint_angle(0, ayaw)
    r.set_arm_joint_angles(a, arm='left')
    sync()
    r.set_joint_angle(0, gyaw)
    r.set_arm_joint_angles(g, arm='left')
    sync(duration=2.0)
    grasp(hand='left')
    sync(duration=1.5)
    r.set_joint_angle(0, ayaw)
    r.set_arm_joint_angles(a, arm='left')
    sync(duration=2.0)

def demo3():
    '''横からペットボトルを掴む決め打ちの動作'''
    prepare_right()
    sync()

    f = FRAME(xyzabc=[250,-200,840,0,0,-pi/2])
    wyaw1,avec1 = r.ik(f, use_waist=True)[0]
    r.set_joint_angle(0, wyaw1)
    r.set_arm_joint_angles(avec1)
    sync()

    f = FRAME(xyzabc=[250,-150,840,0,0,-pi/2])
    wyaw2,avec2 = r.ik(f, use_waist=True)[0]
    r.set_joint_angle(0, wyaw2)
    r.set_arm_joint_angles(avec2)
    sync(duration=2.0)

    r.grasp(width=58)
    sync(duration=1.5)
    time.sleep(1.0)
    r.grasp(width=80)
    sync(duration=1.5)

    r.set_joint_angle(0, wyaw1)
    r.set_arm_joint_angles(avec1)
    sync(2.0)

    prepare_right()
    sync()


# def move_end_effector(effrm, arm='right', use_waist=False):
#     try:
#         wristfrm = effrm*FRAME(xyzabc=[0,0,170,0,-pi/2,0])
#         sol = r.ik(wristfrm, arm=arm, use_waist=use_waist)[0]
#         if use_waist:
#             r.set_joint_angle(0, sol[0])
#             r.set_arm_joint_angles(sol[1], arm=arm)
#         else:
#             r.set_arm_joint_angles(sol, arm=arm)
#     except:
#         warn('ik failure')
#         return

