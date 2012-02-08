# -*- coding: utf-8 -*-

from viewer import *
import re

class NameCollisionError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return self.msg


class PlanningEnvironment:
    def __init__(self):
        self.scene_objects = []
        self.collidable_objects = []
        self.robot = None

    def get_object(self, name):
        objs = [x for x in self.scene_objects if x.name == name]
        # 2 objects with a same name is not permitted
        if objs:
            return objs[0]
        else:
            return None

    def get_objects(self, pattern='.*'):
        r = re.compile(pattern)
        return [x for x in self.scene_objects if r.match(x.name) != None]

    def get_world(self):
        return self.get_object('world')

    def insert_object(self, obj, frame, parent=None):
        # check the names of existing objects
        if obj.name and self.get_object(obj.name):
            raise NameCollisionError(obj.name + ' cannot be used for 2 objects')

        self.scene_objects.append(obj)

        if parent:
            parent.children.append(obj)
            obj.affix(parent, frame)

    def insert_robot(self, robot):
        self.robot = robot
        self.insert_object(robot, FRAME(), parent=self.get_world())

    def get_robot(self):
        return self.robot

    def delete_objects(self, pattern):
        r = re.compile(pattern)
        objs = self.get_objects(pattern)
        for obj in objs:
            if self.get_object(obj.name):
                self.delete_object(obj.name)

    def delete_object(self, name_or_obj):
        def delete_object_aux(obj):
            print 'delete ', obj
            if obj == None:
                return

            # delete all children
            if isinstance(obj, CoordinateObjectWithName):
                for cobj in obj.children:
                    delete_object_aux(cobj)

            try:
                obj.vframe.set_visible(False)
            except:
                pass

            self.robot.cobj_pairs = [p for p in self.robot.cobj_pairs if p[0] != obj and p[1] != obj]
            self.scene_objects.remove(obj)

        if type(name_or_obj) == str:
            obj = self.get_object(name_or_obj)
        else:
            obj = name_or_obj

        if obj == None:
            return

        delete_object_aux(obj)
        obj.parent.children.remove(obj)
        
        # delete_candidate = [obj]
        # for o in self.scene_objects:
        #     if o.ancestor(obj) and o != obj:
        #         delete_candidate.append(o)

        # for o in delete_candidate:
        #     if isinstance(o, CoordinateObject):
        #         o.vframe.set_visible(False)
        #         #o.vbody.frame.set_visible(False)
        #         # remove from collision list
        #         self.robot.cobj_pairs = [p for p in self.robot.cobj_pairs if p[0] != o and p[1] != o]
                
        #         self.scene_objects.remove(o)
        #         # del self.scene_objects[name]


    def show_frames(self):
        print 'not yet implemented'

    def eval_sctree(self, ast):
        try:
            nm = ast['name']
        except:
            nm = None

        if ast['type'] == 'coordinate':
            obj = CoordinateObjectWithName(nm)
        else:
            shape = ast['shape']
            if shape != None:
                col = ast['color']
                mat = ast['material']        
                if mat == 'wood':
                    mat = visual.materials.wood
                elif mat == 'rough':
                    mat = visual.materials.rough
                else:
                    mat = None

            if shape == 'box':
                w,d,h = ast['dimension']
                body = visual.box(length=d, height=w, width=h,
                                  color=col, material=mat)
            elif shape == 'cylinder':
                l,r = ast['dimension']
                body = visual.cylinder(axis=(0,0,l), radius=r, color=col)
            else:
                body = None

            if ast['type'] == 'kinbody':
                obj = KinbodyObject(vbody=body, name=nm)
            else:
                obj = PartsObjectWithName(vbody=body, name=nm)

        for child in ast['children']:
            cobj = self.eval_sctree(child[0])
            self.insert_object(cobj, FRAME(xyzabc = child[1]), parent=obj)

        obj.vframe.resize(1.0)
        return obj

    def load_scene(self, sctree):
        self.insert_object(self.eval_sctree(sctree), FRAME())

    def add_collidable_object(self, obj):
        self.collidable_objects.append(obj)

    def putbox(self, name='box0', vaxis='x', pose2d=None):
        '''シミュレータ内で箱を机上に置く。位置はランダムに決定される。vaxis="y"で側面を上に向けて置く'''
        if pose2d:
            x,y,theta  = pose2d
        else:
            x = random.uniform(-300,-50)
            y = random.uniform(-200,200)
            theta = random.uniform(0,2*pi)
        self.delete_object(name)
        bl,bh,bw=97,66,57
        bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
        obj = PartsObjectWithName(vbody=bx, name=name)
        tbltop = self.get_object('table top') # テーブル上面
        thickness = tbltop.vbody.size[2]
        if vaxis == 'x':
            relfrm = FRAME(xyzabc=[x,y,(thickness+bw)/2,0,0,theta])
        elif vaxis == 'y':
            relfrm = FRAME(xyzabc=[x,y,(thickness+bh)/2,pi/2,theta,0])
        elif vaxis == 'z':
            relfrm = FRAME(xyzabc=[x,y,(thickness+bh)/2,0,-pi/2,0])*FRAME(xyzabc=[0,0,0,theta,0,0])
        else:
            print 'vaxis is wrong'
        return self.insert_object(obj, relfrm, tbltop)
