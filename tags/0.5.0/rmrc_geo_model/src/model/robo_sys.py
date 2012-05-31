from time import *
from object_model_v import *
from hand import *
# from arm6dof_tmpl3 import *
# from l_block import *
# from v_pa10 import *

#
#  define and place objects in task environment
#
def create_env() :
    global room, table, place_a, place_b, box, hand, up200, up100, arm
    room = CoordinateObject()
    table_body=visual.box(color=visual.color.blue,width=0.02,length=0.7,height=1.0)
    table_body.pos=(0,0,-0.01)
    table = PartsObject(vbody=table_body)
    table.affix(room,FRAME(xyzabc=[0,0,0.5,0,0,pi/6]))
    place_a = CoordinateObject()
    place_a.affix(table,FRAME(xyzabc=[0.1,0,0,0,0,0]))
    place_b = CoordinateObject()
    place_b.affix(table,FRAME(xyzabc=[-0.1,0,0,0,0,-pi/6]))
    box_body=visual.box(width=0.1, length=0.05, height=0.03)
    box_body.pos=(0,0,0.05)
    box = PartsObject(vbody=box_body)
    box.grip.set_pos(FRAME(xyzabc=[0,0,0.08,0,0,0]))
    box.grip.width = 0.03
#    box.grip.app_pos.set_trans(FRAME(xyzabc=[0,0,0.1,0,0,0]))
    box.affix(table,place_a.where(table))
    up200=FRAME(vec=VECTOR(0,0,0.2))
    up100=FRAME(vec=VECTOR(0,0,0.1))
    hand=create_hand()
    base_body=visual.cylinder(axis=(0,0,0.1),radius=0.1,color=visual.color.red)
    arm=ArmWithHand(None, hand, up200)
    arm.base.set_vbody(base_body)
    arm.base.affix(table,FRAME(vec=VECTOR(-0.2,0.40,0)))
    AxesXYZ.visible_all(False)
    visual.scene.center=table.where().vec
    visual.scene.autoscale=False
    visual.scene.scale=(2.0,2.0,2.0)
#    visual.scene.scale=(1.0,1.0,1.0)
#    arm.ready()

def reset_env() :
    table.unfix()
    table.affix(room,FRAME(xyzabc=[0,0,0.5,0,0,pi/6]))
    box.unfix()
    box.affix(table,place_a.where(table))
#    arm.ready()

#
# task and subtask programs
#
def pick_up(arm,obj) :
    arm.move("hand",up100,obj.grip.app_pos)
    arm.move("hand", obj.grip.app_pos)
    arm.hand.open(obj.grip.width+0.01)
    arm.move("hand", obj.grip)
    arm.hand.close(obj.grip.width)
    obj.unfix()
    obj.affix(arm.hand)
    arm.move(obj,up100,obj)
    
def place_down(arm,obj,place,fix=None) :
    arm.move(obj,up100,place)
    arm.move(obj,place)
    arm.hand.open(obj.grip.width+0.01)
    obj.unfix()
    if fix :
        obj.affix(fix)
    arm.move("hand", obj.grip.app_pos)
    arm.hand.close(0)
    arm.move("hand",up100,obj.grip.app_pos)

def do_task(arm, obj, place, fix) :
    pick_up(arm,obj)
    place_down(arm,obj, place, fix)

create_env()
