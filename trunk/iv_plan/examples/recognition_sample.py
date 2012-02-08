# -*- coding: utf-8 -*-

def detect(name='box0'):
    if rr:
        Tleye_cb = rr.detect(camera='leye')
        bl,bh,bw = 97,66,57
        Tcb_box = FRAME(xyzabc=[12,-8,-bw/2.0,0,0,pi])
        Tleye_box = Tleye_cb*Tcb_box
        r.set_joint_angles(rr.get_joint_angles())
        print 'leye->target:', Tleye_box
        Twld_box = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_leye*Tleye_box
        print 'world->target:', Twld_box

        # 認識位置の可視化
        env.delete_object(name)
        bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
        obj = PartsObjectWithName(vbody=bx,name=name)
        env.insert_object(obj, Twld_box, env.get_world())

        return Twld_box
    else:
        obj = env.get_object(name)
        if obj:
            frm2 = obj.where()
            print "world->target:", frm2
            return frm2
        else:
            print "not detected"
            return None


def detect_rhand():
    '''ARマーカが貼られた箱の認識(複数対応)'''
    if rr:
        res = rr.detect(camera='rhand')
        bl,bh,bw = 97,66,57
        Tmk_box = FRAME(xyzabc=[0,0,-bh/2.0,pi/2,0,0])
        frms = []
        r.set_joint_angles(rr.get_joint_angles())
        for objnum,Tcam_mk in res:
            Tcam_box = Tcam_mk*Tmk_box
            print 'rhand->target:', Tcam_box
            Twld_box = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam*Tcam_box
            print 'world->target:', Twld_box
            frms.append((objnum,Twld_box))
            # 認識位置の可視化
            name = 'box'+str(objnum)
            env.delete_object(name)
            bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
            obj = PartsObjectWithName(vbody=bx,name=name)
            env.insert_object(obj, Twld_box, env.get_world())

        return frms
    else:
        # box*という名前の物体を検出する
        # *の部分がマーカ番号
        def detected(obj):
            x,y,z = obj.where().vec
            return z > 700 and re.match('box*', obj.name)

        detected_objs = [x for x in env.get_objects() if detected(x)]
        return [(int(re.sub('box', '', x.name)), x.where()) for x in detected_objs]


def detect_rhand2():
    '''ARマーカが貼られた箱の認識(複数対応)'''
    if rr:
        res = rr.detect(camera='rhand')
        bl,bh,bw = 97,66,57
        Tmk_box = FRAME(xyzabc=[0,0,-bw/2.0,0,0,0])
        frms = []
        r.set_joint_angles(rr.get_joint_angles())
        for objnum,Tcam_mk in res:
            Tcam_box = Tcam_mk*Tmk_box
            print 'rhand->target:', Tcam_box
            Twld_box = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam*Tcam_box
            print 'world->target:', Twld_box
            frms.append((objnum,Twld_box))
            # 認識位置の可視化
            name = 'box'+str(objnum)
            env.delete_object(name)
            bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
            obj = PartsObjectWithName(vbody=bx,name=name)
            env.insert_object(obj, Twld_box, env.get_world())

        return frms
    else:
        # box*という名前の物体を検出する
        # *の部分がマーカ番号
        def detected(obj):
            x,y,z = obj.where().vec
            return z > 700 and re.match('box*', obj.name)

        detected_objs = [x for x in env.get_objects() if detected(x)]
        return [(int(re.sub('box', '', x.name)), x.where()) for x in detected_objs]


def look_for_boxes(name='box0'):
    '''右手ハンドの向きを変えて箱（マーカ）を探す'''
    f0 = r.fk()
    objfrms = [None,None]
    for i in range(1,2)+range(2,-4,-1):
        f = f0 * FRAME(xyzabc=[0,0,0,0,0,pi/16*i])
        js = r.ik(f)[0]
        r.set_arm_joint_angles(js)
        sync(duration=1.5)

        for objnum, objfrm in detect_rhand():
            print 'marker %d found'%objnum
            if objnum < 2:
                objfrms[objnum] = objfrm
                print objfrms
            if objfrms[0] and objfrms[1]:
                return objfrms
    return None


def look_for_boxes2(num):
    '''右手ハンドの向きを変えて箱（マーカ）を探す'''
    f0 = r.fk()
    objfrms = [None,None]
    for i in range(1,2)+range(2,-4,-1):
        f = f0 * FRAME(xyzabc=[0,0,0,0,0,pi/16*i])
        js = r.ik(f)[0]
        r.set_arm_joint_angles(js)
        sync(duration=1.5)

        for objnum, objfrm in detect_rhand2():
            print 'marker %d found'%objnum
            if objnum == num:
                print objfrm
                return objfrm
    return None
