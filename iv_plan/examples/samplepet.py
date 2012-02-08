def detect_rhand1():
    if rr:
        bl,bh,bw = 97,66,57
        frms = []
        res = rr.detect(camera='rhand')
        r.set_joint_angles(rr.get_joint_angles())
        for objnum,Tcam_mk in res:
            Twld_mk = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam*Tcam_mk
            print 'world->marker:', Tcam_mk
            frms.append((objnum,Twld_mk))
            # 認識位置の可視化
            name = 'pet'+str(objnum)
            env.delete_object(name)
            bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
            obj = PartsObjectWithName(vbody=bx,name=name)
            env.insert_object(obj, Twld_mk, env.get_world())
        return frms

def get_frm_from_id(n, res):
    return [x[1] for x in res if x[0]==n][0]

def push_and_grasp():
    res = detect_rhand1()
    f4 = get_frm_from_id(4, res)

    # push
    f4a = FRAME(xyzabc=[f4.vec[0]-180,f4.vec[1]-90,880,0,pi+0.1,-0.5])
    f4g = FRAME(xyzabc=[f4.vec[0]-110,f4.vec[1]-55,840,0,pi+0.1,-0.5])
    r.set_arm_joint_angles(r.ik(f4a)[0])
    r.set_hand_joint_angles(zeros(4))
    sync()
    r.set_arm_joint_angles(r.ik(f4g)[0])
    sync(duration=2.0)
    r.set_arm_joint_angles(r.ik(f4a)[0])
    sync()

    f5 = get_frm_from_id(5, res)
    f5a = FRAME(xyzabc=[f5.vec[0]-180,f5.vec[1]-90,880,0,pi+0.1,-0.5])
    f5g = FRAME(xyzabc=[f5.vec[0]-110,f5.vec[1]-55,840,0,pi+0.1,-0.5])
    w,av = r.ik(f5a, use_waist=True)[0]
    r.set_joint_angle(0, w)
    r.set_arm_joint_angles(av)
    r.grasp(width=100)
    sync(duration=2.0)
    w,av = r.ik(f5g, use_waist=True)[0]
    r.set_joint_angle(0, w)
    r.set_arm_joint_angles(av)
    sync(duration=2.0)
    r.grasp(width=58)
    sync(duration=2.0)

    w,av = r.ik(f5a, use_waist=True)[0]
    r.set_joint_angle(0, w)
    r.set_arm_joint_angles(av)
    sync(duration=2.0)

def push_pet_bottles1():
    res = detect_rhand1()
    f4 = get_frm_from_id(4, res)
    f5 = get_frm_from_id(5, res)
    f4a = FRAME(xyzabc=[f4.vec[0]-180,f4.vec[1]-90,880,0,pi+0.1,-0.5])
    f4g = FRAME(xyzabc=[f4.vec[0]-110,f4.vec[1]-55,840,0,pi+0.1,-0.5])
    r.set_arm_joint_angles(r.ik(f4a)[0])
    sync()
    r.set_arm_joint_angles(r.ik(f4g)[0])
    sync(duration=2.0)
    r.grasp(width=58)
    sync(duration=2.0)

    # push grasp
    r.set_arm_joint_angles(r.ik(f4a)[0])
    sync()

    # remove by pushing

    # remove blocking object
    # f = r.fk()
    # f.vec[1] -= 50
    # f.vec[2] += 20
    # r.set_arm_joint_angles(r.ik(f)[0])
    # sync(duration=2.0)
    # f.vec[1] -= 50
    # f.vec[2] -= 19
    # r.set_arm_joint_angles(r.ik(f)[0])
    # sync(duration=2.0)
    # r.grasp(width=100)
    # sync()
    # f.vec[0] -= 50
    # f.vec[1] -= 25
    # f.vec[2] += 40
    # r.set_arm_joint_angles(r.ik(f)[0])
    # sync(duration=2.0)
    # f.vec[0] -= 50
    # f.vec[1] += 140
    # r.set_arm_joint_angles(r.ik(f)[0])
    # sync(duration=2.0)

    # f5a = FRAME(xyzabc=[f5.vec[0]-180,f5.vec[1]-90,880,0,pi+0.1,-0.5])
    # f5g = FRAME(xyzabc=[f5.vec[0]-110,f5.vec[1]-55,840,0,pi+0.1,-0.5])
    # w,av = r.ik(f5a, use_waist=True)[0]
    # r.set_joint_angle(0, w)
    # r.set_arm_joint_angles(av)
    # sync()
    # w,av = r.ik(f5g, use_waist=True)[0]
    # r.set_joint_angle(0, w)
    # r.set_arm_joint_angles(av)
    # sync(duration=2.0)
    # r.grasp(width=58)
    # sync(duration=2.0)

#r.set_joint_angles(prepare_right2)
prepare_right2 = [0.24999878039410275,
                  -0.3999701506333021,
                  1.0999912242536529,
                  -0.242129102211388,
                  -0.55000849205344104,
                  -2.246649011302321,
                  -0.23170880440775701,
                  0.77617486969986105,
                  -0.064163644362093999,
                  0.009992009967667536,
                  0.0,
                  -1.7449976394364506,
                  -0.26498055477880189,
                  0.16399277276356095,
                  -0.055982450484027418,
                  0.78539814154175325,
                  -0.089011788368225098,
                  -0.78539814154175325,
                  0.090757124125957489,
                  -4.3711390063094768e-08,
                  0.013962633907794952,
                  0.0069812973353524654,
                  -0.015707964077591896]



# neck_y = 0.0
hdp_sample = [ 0.65, 0.75, 0.85, 0.95, 1.0, 1.05, 1.1]

# neck_p = 0.8
hdy_sample = [ -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3]

def gen_head_calib_axis(r, rr):
    r.set_joint_angle(1, 0)
    sync()
    hdp_data = []
    for i in range(len(hdp_sample)):
        r.set_joint_angle(2, hdp_sample[i])
        sync()
        time.sleep(1.0)
        hdp_data.append([rr.get_joint_angles(), rr.detect(camera='kinect_rgb')])

    r.set_joint_angle(2, 0.8)
    sync()
    hdy_data = []
    for i in range(len(hdy_sample)):
        r.set_joint_angle(1, hdy_sample[i])
        sync()
        time.sleep(1.5)
        hdy_data.append([rr.get_joint_angles(), rr.detect(camera='kinect_rgb')])
    return [hdp_data, hdy_data]

hdgrid_sample = [[0.7, 0.8, 0.9, 1.0],[-0.25, -0.15, 0, 0.15, 0.25]]

def gen_head_calib_grid(r, rr):
    hdgrid_data = []
    for p in hdgrid_sample[0]:
        r.set_joint_angle(2, p)
        for y in hdgrid_sample[1]:
            r.set_joint_angle(1, y)
            sync()
            time.sleep(1.0)
            hdgrid_data.append([rr.get_joint_angles(), rr.detect(camera='kinect_rgb')])
    return hdgrid_data
