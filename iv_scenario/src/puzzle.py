# -*- coding: utf-8 -*-

# set up environment for ROS/RTM
# RTM nameserver is read from rtc.conf file in the current directory

from ivenv import *
import scene_objects
from hironx import *
from mplan_env import *
from csplan import *

import hironx_params
import hironxsys

real_robot = False
if real_robot:
    rr = hironxsys.HiroNxSystem(nameserver, hironx_params.portdefs)
else:
    rr = None

env = PlanningEnvironment()
env.load_scene(scene_objects.table_scene())

r = HiroNx(ivenv.ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/')
env.insert_robot(r)
r.go_pos(-150, 0, 0)
r.prepare()

pl = CSPlanner(env)

import toplevel_extension
toplevel_extension.setup_toplevel_extension(r,env,rr,pl)
from toplevel_extension import *


# definitions of piece geometry
def cube(name, color, l):
    return {'type':'kinbody', 'name':name, 'shape':'box',
            'color':color, 'material':None,
            'dimension':[l-1, l-1, l-1], 'children':[]}

def generate_piece(nametag, color, cubeposes, l=30.0):
    l2 = l/2
    xyzabcs = cubeposes
    cbs = [(cube(name=nametag+str(i), color=color, l=l),xyzabc) for i,xyzabc in enumerate(xyzabcs)]
    return env.eval_sctree({'type':'parts', 'name':'piece-'+nametag, 'shape':None, 'children':cbs})

init_frames = {}

def setup_puzzle_scene():
    l = 30.0
    tbl = env.get_object('table')
    tbltop = env.get_object('table top')
    ttz = tbltop.where().vec[2] + tbltop.vbody.size[2]/2
    z = ttz + l/2

    # generate 7 pieces
    # green piece
    env.insert_object(generate_piece('green', (0.0, 0.392, 0.0),
                                     [(0,0,0,0,0,0),(0,0,l,0,0,0),(l,0,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-160,-80,z,0,0,-pi/2]), tbl)
    # yellow green piece
    env.insert_object(generate_piece('yellow-green', (0.0, 1.0, 0.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(l,0,0,0,0,0)]),
                      FRAME(xyzabc=[-130,70,z+l,-pi/2,pi,0])*FRAME(xyzabc=[0,0,0,0,-0.5,0]), tbl)
    # red piece
    env.insert_object(generate_piece('red', (1.0, 0.0, 0.0),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(2*l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-160,-180,z+l,-pi/2,0,0])*FRAME(xyzabc=[0,0,0,0,pi/3,0]), tbl)
    # purple piece
    env.insert_object(generate_piece('purple', (0.6, 0.196, 0.8),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(0,0,l,0,0,0)]),
                      FRAME(xyzabc=[-10,-130,z,0,-pi/2,0])*FRAME(xyzabc=[0,0,0,-1,0,0]), tbl)
    # aqua piece
    env.insert_object(generate_piece('aqua', (0.0, 1.0, 1.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(0,2*l,0,0,0,0),(l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-200,-280,z,pi/2,0.3,0]), tbl)
    # brown piece
    env.insert_object(generate_piece('brown', (0.545, 0.27, 0.075),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-60,-260,z,0,0,-1.0]), tbl)
    # yellow piece
    env.insert_object(generate_piece('yellow', (1.0, 1.0, 0.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(0,2*l,0,0,0,0),(l,0,0,0,0,0)]),
                      FRAME(xyzabc=[-250,0,z,0,0,-pi/2]), tbl)


    # remember the initial positions of the pieces in world frame
    for p in env.get_objects('^piece'):
        init_frames[p.name] = FRAME(p.where())

    # register collision pairs and graspable objects
    ps = env.get_objects('^piece')
    for p in ps:
        r.add_collision_object(p)
        for p2 in ps:
            if p != p2:
                r.add_collision_pair(p, p2)
        env.add_collidable_object(p)
    env.add_collidable_object(env.get_object('table'))


def reset_puzzle():
    for p in env.get_objects('^piece'):
        p.locate(init_frames[p.name], world=True)

w1 = (80, 24)
w2 = (100, 54)
w3 = (110, 84)
l = 30

grspposs = {
    'piece-green': (FRAME(xyzabc=[0,0,l,0,0,pi/2]), w1),
    'piece-yellow-green': (FRAME(xyzabc=[0,0,0,-pi/2,pi,0]), w1),
    'piece-red': (FRAME(xyzabc=[0.5*l,0,0,pi/2,0,pi/2]), w2),
    'piece-purple': (FRAME(xyzabc=[l,0,0,0,pi/2,pi/2]), w1),
    'piece-aqua': (FRAME(xyzabc=[0,2*l,0,-pi/2,0,-pi/2]), w1),
    'piece-brown': (FRAME(xyzabc=[0.5*l,l,-2,0,0,pi/2]), w2),
    'piece-yellow': (FRAME(xyzabc=[0,l,-0.5*l,0,0,pi]), w3)
    }


def grasp_plan(obj):
    '''given an object, compute handlink frame and distance between fingers to pick the piece'''
    d = 30

    f, (awidth,gwidth) = grspposs[obj.name]

    gfrm = obj.where()*f*(-r.Trwrist_ef)
    afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
    return afrm,gfrm,awidth,gwidth


def place_plan(obj, hand='right'):
    l = 30
    d = 30

    tbl = env.get_object('table')
    tbltop = env.get_object('table top')
    ttz = tbltop.where().vec[2] + tbltop.vbody.size[2]/2
    z = ttz + l/2

    # reference frame
    rfrm = FRAME(xyzabc=[190,-80,z,0,0,-pi/2])

    plcposs = {
        'piece-green':FRAME(xyzabc=[0,0,0,0,0,0]),
        'piece-yellow-green':FRAME(xyzabc=[0,2*l,l,-pi/2,pi/2,0]),
        'piece-red':FRAME(xyzabc=[l,0,l,-pi/2,-pi/2,0]),
        'piece-purple':FRAME(xyzabc=[2*l,2*l,0,pi/2,0,pi/2]),
        'piece-aqua':FRAME(xyzabc=[2*l,0,0,pi/2,pi/2,0]),
        'piece-brown':FRAME(xyzabc=[2*l,l,2*l,0,0,pi/2]),
        'piece-yellow':FRAME(xyzabc=[0,0,2*l,0,0,0])
    }

    appvec = {
        'piece-green':FRAME(xyzabc=[0,0,0,0,0,0]),
        'piece-yellow-green':FRAME(xyzabc=[d,d,-d,0,0,0]),
        'piece-red':FRAME(xyzabc=[-d,d,d,0,0,0]),
        'piece-purple':FRAME(xyzabc=[-d,d,d,0,0,0]),
        'piece-aqua':FRAME(xyzabc=[d,-d,-d,0,0,0]),
        'piece-brown':FRAME(xyzabc=[-d,-d,-d,0,0,0]),
        'piece-yellow':FRAME(xyzabc=[d,d,-d,0,0,0])
    }

    handfrm = r.get_link(hand[0].upper()+'ARM_JOINT5_Link').where()
    objfrm = obj.where()

    gfrm = rfrm*plcposs[obj.name]*(-objfrm)*handfrm
    afrm = rfrm*plcposs[obj.name]*(-appvec[obj.name])*(-objfrm)*handfrm

    _, (awidth,gwidth) = grspposs[obj.name]
    return afrm,gfrm,awidth,gwidth

tms = {'app':0.8, 'hand':0.3}

def pick_piece(obj, jts='rarm', hand='right'):
    # determine grasp position & hand width
    afrm,gfrm,awidth,gwidth = grasp_plan(obj)

    r.grasp(awidth, hand=hand)
    sync(duration=tms['hand'])

    # initial configuration
    q0 = r.get_joint_angles(joints=jts)
    # compute goal configuration by solving inverse kimematics
    q1 = r.ik(afrm, joints=jts)[0]

    # connect 2 configurations by RRT-connect
    traj = pl.make_plan(q0, q1, joints=jts)
    # execute the trajectory
    exec_traj(traj, joints=jts)

    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    sync(duration=tms['app'])
    r.grasp(gwidth+10, hand=hand)
    sync(duration=tms['hand'])
    r.grasp(gwidth, hand=hand)
    sync(duration=tms['hand'])
    grab(hand=hand)
    r.set_joint_angles(r.ik(afrm, joints=jts)[0], joints=jts)
    sync(duration=tms['app'])

def place_piece(obj, p=FRAME(xyzabc=[0,0,0,0,0,0]), jts='rarm', hand='right'):
    afrm,gfrm,awidth,gwidth = place_plan(obj, hand)
    q0 = r.get_joint_angles(joints=jts)
    q1 = r.ik(afrm, joints=jts)[0]
    traj = pl.make_plan(q0, q1, joints=jts)
    exec_traj(traj, joints=jts)

    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    sync(duration=tms['app'])
    r.grasp(gwidth+10, hand=hand)
    sync(duration=tms['hand'])
    r.grasp(awidth, hand=hand)
    sync(duration=tms['hand'])
    release(hand=hand)
    r.set_joint_angles(r.ik(afrm, joints=jts)[0], joints=jts)
    sync(duration=tms['app'])

def demo(jts='rarm'):
    hand = 'right' if re.match('.*rarm', jts) else 'left'

    cl = ['yellow-green','red','purple','aqua','brown','yellow']
    for c in cl:
        obj = env.get_object('piece-'+c)
        sync()
        pick_piece(obj, jts=jts, hand=hand)
        place_piece(obj, jts=jts, hand=hand)
    r.prepare()
    sync()


import discrete_puzzle
def parse_solution(sol, step=True):
    def aux(action):
        l = 30.0
        n,p = action
        pos, (ex,ey) = p.qs[n][2]
        ez = cross(ex, ey)
        return p.col, FRAME(mat=transpose(array([ex,ey,ez])).tolist(), vec=(l*pos).tolist())

    acts = [aux(act) for act in sol]
    basefrm = FRAME(xyzabc=[300,0,800,0,0,0])
    for a in acts:
        nm = 'piece-'+a[0]
        f = basefrm*a[1]
        print 'put ', nm, 'at ', f
        env.get_object(nm).locate(f)
        if step:
            raw_input(); print('hit any key')


import operator

def spaces(n):
    return reduce(operator.__add__, map(lambda x: ' ', range(n)))

def format_vertice(v, ident_level=1):
    return spaces(ident_level*2)+'{%f, %f, %f}'%(v[0],v[1],v[2])

def format_vertices(vs, name='vertices'):
    code = reduce(lambda x,y: x+',\n'+y, map(format_vertice, vs))
    return 'double ' + name + '[][3] = {\n' + code + '\n};'

def format_edge(e, ident_level=1):
    return spaces(ident_level*2)+'{%d, %d}'%(e[0],e[1])

def format_edges(es, name='edges'):
    code = reduce(lambda x,y: x+',\n'+y, map(format_edge, es))
    return 'int ' + name + '[][2] = {\n' + code + '\n};'

def generate_piece_defs():
    print format_vertices(piecered['vertices'], 'redvertices')
    print format_edges(piecered['edges'], 'rededges')
    print format_vertices(pieceaqua['vertices'], 'aquavertices')
    print format_edges(pieceaqua['edges'], 'aquaedges')
    print format_vertices(pieceyellow['vertices'], 'yellowvertices')
    print format_edges(pieceyellow['edges'], 'yellowedges')
    print format_vertices(pieceyellowgreen['vertices'], 'yellowgreenvertices')
    print format_edges(pieceyellowgreen['edges'], 'yellowgreenedges')
    print format_vertices(piecebrown['vertices'], 'brownvertices')
    print format_edges(piecebrown['edges'], 'brownedges')

def dump_body(body):
    body.rel_trans
    lx,ly,lz = body.vbody.size
    x = lx/2
    y = ly/2
    z = lz/2
    vertices = [[x,y,z],[x,y,-z],
                [x,-y,z],[x,-y,-z],
                [-x,y,z],[-x,y,-z],
                [-x,-y,z],[-x,-y,-z]]
    edges = [[1,2],[2,4],[4,3],[3,1],
             [5,6],[6,8],[8,7],[7,5],
             [1,5],[2,6],[3,7],[4,8]]

    for vertice in vertices:
        print format_vertice((body.rel_trans*FRAME(vec=vertice)).vec), ','

    # for edge in edges:
    #     print edge[0], '=>', edge[1]

def dump_piece_model(piece):
    name = piece.name
    print 'int ' + name + '[][2] = {'
    bodies = piece.children
    for body in bodies:
        dump_body(body)
    print '};'
    

setup_puzzle_scene()
r.prepare()
colored_print("demo()", 'blue')
