#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
import operator
import pprint

from numpy import *

from wrl_parser import *
from viewer import *
from ivutils import *


class WrlLoader():
    def __init__(self, scale=1000.0):
        self.scale = scale

    def load(self, wrldir, mainfile='main.wrl'):
        self.wrldir = wrldir
        ast = parse_wrl(wrldir+'/'+'main.wrl', 'toplevel')

        humanoid_nodes = filter(lambda x: x['type'] == 'Humanoid', ast)
        if len(humanoid_nodes) != 1:
            print 'There should be exactly one humanoid definition'
        else:
            return self.eval_humanoid(humanoid_nodes[0])

    def eval_humanoid(self, humanoid_node):
        wenv = { 'joints': [], 'links': [] }
        name = humanoid_node['name']
        nd = humanoid_node['humanoidBody'][0]

        obj,_ = self.eval_joint_node(nd, wenv)
        joints = range(len(wenv['joints']) - 1)
        links = []

        # sort joints in the order of joint ID to access quickly
        for j in wenv['joints']:
            links.append(j.link)
            if j.id != None:
                joints[j.id] = j

        return name, joints, links

    def eval_shape(self, shape_node):
        col = [1.0,1.0,1.0]
        fvs = []
        geotyp = None
        body = None

        material = filter(lambda x: x['type'] == 'Material', shape_node)[0]
        col = material['diffuseColor']

        for prop in shape_node:
            geotyp = prop['type']
            if geotyp == 'Coordinate':
                vs = prop['point']
                fidx = prop['coordIndex']
                vs = self.scale * array(vs)

                for i in fidx:
                    if i != -1:
                        fvs.append(vs[i])

                body = visual.faces(pos=fvs, color=col)
                body.make_normals()
                body.smooth()

            elif geotyp == 'Sphere':
                R = prop['radius']
                R = self.scale * R
                body = visual.sphere(pos=(0,0,0), radius=R, color=col)
            elif geotyp == 'Box':
                L,H,W = prop['size']
                L = self.scale * L
                H = self.scale * H
                W = self.scale * W
                body = visual.box(pos=(0,0,0), axis=(1,0,0), color=col,
                                  length=L, height=H, width=W)
            elif geotyp == 'Cylinder':
                R = prop['radius']
                R = self.scale * R
                H = prop['height']
                H = self.scale * H
                body = visual.cylinder(axis=(0,0,H), radius=R, color=col)

        obj = PartsObjectWithName(vbody=body)
        return obj

    def eval_inline(self, inline):
        url = inline['Inline'][0]['url']
        wrlpath = self.wrldir + '/' + url
        print 'Including', wrlpath

        tf = parse_wrl(wrlpath)[0]

        #
        # Currently lists must come after the 'children' keyword
        # need to handle the case:
        # children Shape { ... } in PA10/j3.wrl
        #

        return self.eval_transform(tf)

    def eval_transform(self, tf):
        if tf.has_key('rotation'):
            rot = tf['rotation']
        else:
            rot = [0,0,1,0]

        if tf.has_key('translation'):
            trans = tf['translation']
        else:
            trans = [0,0,0]

        reltrans = FRAME(vec=self.scale*VECTOR(vec=trans),
                         mat=MATRIX(axis=VECTOR(vec=rot[:-1]),
                                    angle=rot[3]))

        children = []
        for child in tf['children']:
            if child.has_key('Shape'):
                children.append((reltrans, self.eval_shape(child['Shape'])))
            elif child.has_key('Inline'):
                children += [(reltrans*tr,shp) for tr,shp in self.eval_inline(child)]
            else:
                children += [(reltrans*tr,shp) for tr,shp in self.eval_transform(child)]

        return children

    def eval_segment_node(self, seg):
        name = seg['name']

        print 'Segment: ' + name

        children = []
        for child in seg['children']:
            if child.has_key('Inline'):
                children += self.eval_inline(child)
            elif child['type'] == 'Transform':
                children += self.eval_transform(child)
                
        return children

    def eval_joint_node(self, jnt, wenv):
        def getval(dict, key, default_value=0.0):
            return dict[key][0] if dict.has_key(key) else default_value

        jaxis = jnt['jointAxis']
        if not jaxis:
            jaxis = [0,0,1] # default axis is 'Z'
        trans = jnt['translation']
        if not trans:
            trans = [0,0,0]
        rot = jnt['rotation']            
        if not rot:
            rot = [0,0,1,0]
        jaxis = VECTOR(vec=jaxis)
        trans = self.scale * VECTOR(vec=trans) # [m] => [mm]
        rot = MATRIX(axis=VECTOR(vec=rot[:-1]), angle=rot[3])
        reltrans = FRAME(vec=trans, mat=rot)
        jobj = JointObject(jnt['jointId'], jnt['name'],
                           jaxis, reltrans,
                           getval(jnt,'ulimit'), getval(jnt,'llimit'),
                           getval(jnt,'uvlimit'), getval(jnt,'lvlimit'))
        wenv['joints'].append(jobj)

        children = jnt['children']

        jchildren = filter(lambda x: x['type'] == 'Joint', children)
        for jchild in jchildren:
            childobj,creltrans = self.eval_joint_node(jchild, wenv)
            childobj.affix(jobj, creltrans)
        schildren = filter(lambda x: x['type'] == 'Segment', children)

        tr_shps = reduce(operator.__add__,
                         [self.eval_segment_node(x) for x in schildren])
        for tr,shp in tr_shps:
            shp.affix(jobj, tr)

        # use the name of the first segment as the name of the link
        jobj.link = LinkObject(jobj, name=schildren[0]['name'], shapes=tr_shps)

        return jobj, reltrans


if __name__ == '__main__':
    loader = WrlLoader()
    if len(sys.argv) == 2:
        if sys.argv[1] == 'pa10':
            obj = loader.load('../externals/models/PA10_2/')
        elif sys.argv[1] == 'rh2':
            obj = loader.load('../externals/models/RH2/')
        elif sys.argv[1] == 'hironx':
            obj = loader.load('../externals/models/HIRO_110219/')

        pprint.pprint(obj)
    else:
        print './wrl_loader <robot_name>'
