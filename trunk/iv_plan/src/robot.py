# -*- coding: utf-8 -*-

import os
import re
import operator
from subprocess import Popen, PIPE

from numpy import *
from ivutils import *
from viewer import *
import wrl_loader

from pqp_if import *

def get_AABB(vs, padding=2.0):
    xlb = ylb = zlb = inf
    xub = yub = zub = -inf

    if vs.__class__ == list:
        for v in vs:
            if v[0] < xlb:
                xlb = v[0]
            if v[0] > xub:
                xub = v[0]
            if v[1] < ylb:
                ylb = v[1]
            if v[1] > yub:
                yub = v[1]
            if v[2] < zlb:
                zlb = v[2]
            if v[2] > zub:
                zub = v[2]
    else:
        if vs.vbody.__class__ == visual.box:
            xlb = vs.vbody.x - vs.vbody.length/2.0
            ylb = vs.vbody.y - vs.vbody.height/2.0
            zlb = vs.vbody.z - vs.vbody.width/2.0
            xub = vs.vbody.x + vs.vbody.length/2.0
            yub = vs.vbody.y + vs.vbody.height/2.0
            zub = vs.vbody.z + vs.vbody.width/2.0
        elif vs.vbody.__class__ == visual.cylinder:
            r = vs.vbody.radius
            z = vs.vbody.axis[2]
            xlb = -r
            xub = r
            ylb = -r
            yub = r
            zlb = 0
            zub = z
        else:
            warn('unknown shape')
            return None
    return [[xlb-padding,xub+padding],
            [ylb-padding,yub+padding],
            [zlb-padding,zub+padding]]

def gen_collision_body(obj):
    tris = [(0,1,2), (2,3,0),
            (0,4,1), (4,5,1),
            (1,5,2), (5,6,2),
            (2,3,6), (3,7,6),
            (0,4,7), (0,7,3),
            (4,5,6), (4,6,7)]

    def gen_cbody_from_AABB(aabb):
        [[x0,x1],[y0,y1],[z0,z1]] = aabb
        lx = x1-x0
        ly = y1-y0
        lz = z1-z0
        pts = [[x0+lx,y0,   z0+lz],
               [x0+lx,y0+ly,z0+lz],
               [x0,   y0+ly,z0+lz],
               [x0,   y0,   z0+lz],
               [x0+lx,y0,   z0   ],
               [x0+lx,y0+ly,z0   ],
               [x0,   y0+ly,z0   ],
               [x0,   y0,   z0   ]]

        b = cpqp.PQP_Model()
        b.BeginModel(8)
        for i in range(len(tris)):
            tri = tris[i]
            b.AddTri(pts[tri[0]],pts[tri[1]],pts[tri[2]],i)
        b.EndModel()
        b.MemUsage(1)
        return b

    def points_from_AABB(aabb):
        [[x0,x1],[y0,y1],[z0,z1]] = aabb
        lx = x1-x0
        ly = y1-y0
        lz = z1-z0
        pts = [[x0+lx,y0,   z0+lz],
               [x0+lx,y0+ly,z0+lz],
               [x0,   y0+ly,z0+lz],
               [x0,   y0,   z0+lz],
               [x0+lx,y0,   z0   ],
               [x0+lx,y0+ly,z0   ],
               [x0,   y0+ly,z0   ],
               [x0,   y0,   z0   ]]
        return pts

    def gen_cbody(obj):
        if obj.vbody.__class__ == visual.extrusion:
            fcs = obj.vbody.get_faces()[0]
            fcs = fcs.reshape((fcs.size/3/3, 3, 3))
            b = cpqp.PQP_Model()
            b.BeginModel(8)
            for i,(p1,p2,p3) in enumerate(fcs):
                b.AddTri(p1,p2,p3,i)
            b.EndModel()
            b.MemUsage(1)
            return b
        else:
            return gen_cbody_from_AABB(get_AABB(obj))

    def gen_cbody_link(l, simple=True):
        pts = []
        for tf,shp in l.shapes:
            invmat = -tf.mat
            if shp.vbody.__class__ == visual.faces:
                # transform all the 'pos' and add them to pts
                for pos in shp.vbody.pos:
                    pts.append(invmat*VECTOR(vec=pos.tolist())+tf.vec)
                    # pts.append(pos)
            elif shp.vbody.__class__ == visual.box or shp.vbody.__class__ == visual.cylinder:
                aabb = get_AABB(shp)
                pts = points_from_AABB(aabb)
                for tri in tris:
                    pts.append(invmat*VECTOR(vec=pts[tri[0]])+tf.vec)
                    pts.append(invmat*VECTOR(vec=pts[tri[1]])+tf.vec)
                    pts.append(invmat*VECTOR(vec=pts[tri[2]])+tf.vec)

        if len(pts) == 0:
            return
        if simple:
            aabb = get_AABB(pts)
            print l.name,
            print aabb
            return gen_cbody_from_AABB(aabb)
        else:
            b = cpqp.PQP_Model()
            b.BeginModel(8)
            for i in range(len(pts)/3):
                b.AddTri(pts[3*i],pts[3*i+1],pts[3*i+2],i)
            b.EndModel()
            b.MemUsage(1)
            return b

    if obj.__class__ == LinkObject:
        return gen_cbody_link(obj)
    elif obj.__class__ == PartsObjectWithName:
        try:
            return gen_cbody(obj)
        except:
            return None
    elif obj.__class__ == KinbodyObject:
        return gen_cbody(obj)        

def in_collision_pair(obj1, obj2, cache):
    if obj1.cb == None or obj2.cb == None:
        return False

    try:
        R1,T1 = cache[obj1]
    except:
        f1 = obj1.where()
        R1 = array(f1.mat[:3][:3])
        T1 = array(f1.vec)
        cache[obj1] = (R1,T1)

    try:
        R2,T2 = cache[obj2]
    except:
        f2 = obj2.where()
        R2 = array(f2.mat[:3][:3])
        T2 = array(f2.vec)
        cache[obj2] = (R2,T2)

    cres = collide(R1, T1, obj1.cb, R2, T2, obj2.cb, cpqp.contact_type.first)

    if cres.NumPairs() > 0:
        print '%s <=> %s'%(obj1.name, obj2.name)
        # print_collide_result(cres)
        return True
    else:
        return False

def in_collision_pair_parts(obj1, obj2, cache):
    if in_collision_pair(obj1, obj2, cache):
        return True
    for cobj1 in obj1.children:
        if isinstance(cobj1, KinbodyObject):
            for cobj2 in obj2.children:
                if in_collision_pair(cobj1, cobj2, cache):
                    return True
            if in_collision_pair(cobj1, obj2, cache):
                return True
    for cobj2 in obj2.children:
        if isinstance(cobj2, KinbodyObject):
            if in_collision_pair(cobj2, obj1, cache):
                return True
    return False


class VRobot(JointObject):
    def __init__(self, wrldir, mainfile, scale, robotname, base2waist=FRAME(vec=VECTOR(vec=[0,0,800]))):
        JointObject.__init__(self, 0, robotname, [0,0,1], FRAME())
        # self.wrldir = os.getcwd() + re.sub('[^\/]*$', '', wrlfile)
        # print ('wrl directory = ' + self.wrldir)
        loader = wrl_loader.WrlLoader()
        name, joints, links = loader.load(wrldir, mainfile=mainfile)

        self.joints = joints
        self.links = links
        self.link = LinkObject(self, name='BASE_Link')
        self.links.insert(0, self.link)
        joints[0].parent.affix(self, base2waist)

        self.gen_link_collision_body()
        self.cobj_pairs = []
        self.sensors = []

        self.grabbed_obj = {'right': None, 'left': None}

        self.init_clink_pairs()
        self.reset_pose()

    def __repr__(self):
        return '<VRobot %s>'%self.name

    def __str__(self):
        return self.__repr__()

    def add_collision_pair(self, obj1, obj2):
        if obj1.cb == None:
            obj1.cb = gen_collision_body(obj1)
        for cobj in obj1.children:
            if isinstance(cobj, KinbodyObject) and (not cobj.cb):
                cobj.cb = gen_collision_body(cobj)
        if obj2.cb == None:
            obj2.cb = gen_collision_body(obj2)
        for cobj in obj2.children:
            if isinstance(cobj, KinbodyObject) and (not cobj.cb):
                cobj.cb = gen_collision_body(cobj)

        if not (obj1,obj2) in self.cobj_pairs:
            self.cobj_pairs.append((obj1, obj2))

    def remove_collision_pair(self, obj1, obj2):
        self.cobj_pairs = [(x,y) for x,y in self.cobj_pairs if not ((x == obj1 and y == obj2) or (x == obj2 and y == obj1))]

    def in_collision(self, check_all=False):
        cache = {}
        for l1,l2 in self.clink_pairs:
            if in_collision_pair_parts(l1, l2, cache):
                return True
        for obj1, obj2 in self.cobj_pairs:
            if in_collision_pair_parts(obj1, obj2, cache):
                return True
        return False

    def gen_link_collision_body(self):
        for lnk in self.get_links():
            lnk.cb = gen_collision_body(lnk)

    def set_joint_angle(self, id, th, flush=True, check_collision=False):
        self.joints[id].angle = th

        if check_collision:
            th_old = self.get_joint_angle(id)

        if flush:
            self.refresh()

        if check_collision and self.in_collision():
            warn('collision detected')

            self.joints[id].angle = th_old
            if flush:
                self.refresh()

    def refresh(self):
        for j in self.joints:
            newth = j.angle
            jaxis = j.jaxis
            ltrans = j.reltrans
            obj = j
            jtrans = FRAME(mat=MATRIX(axis=jaxis,angle=newth))
            reltrans = ltrans * jtrans
            obj.set_trans(reltrans)

    def get_joint_angle(self, id):
        return self.joints[id].angle

    def reset_pose(self):
        self.prepare()

    def prepare(self, width=None):
        self.set_joint_angles(self.poses['prepare'])
        if width:
            self.grasp(width=width, hand='right')
            self.grasp(width=width, hand='left')

    def go_pos(self, x, y, theta):
        self.set_trans(FRAME(xyzabc=[x,y,0,0,0,theta]))

    def get_link(self, name):
        return [x for x in self.get_links() if x.name == name][0]

    def get_joint(self, name):
        return [x for x in self.get_joints() if x.name == name][0]

    def get_sensor(self, name):
        return [x for x in self.get_sensors() if x.name == name][0]

    def get_links(self):
        return self.links

    def get_joints(self):
        return self.joints

    def get_sensors(self):
        return self.sensors

