from time import *
from object_model_v import *
import new

def create_hand() :
    hand = Hand()
    hand.body=visual.box(size=(0.03,0.06,0.04),color=visual.color.green)
    hand.finger1=visual.box(size=(0.03,0.004,0.05),color=visual.color.green)
    hand.finger2=visual.box(size=(0.03,0.004,0.05),color=visual.color.green)
    hand.body.pos=(0,0,0.07)
    hand.finger1.pos=(0,0.002,0.025)
    hand.finger2.pos=(0,-0.002,0.025)
    hand.body.frame=hand.vframe
    hand.finger1.frame=hand.vframe
    hand.finger2.frame=hand.vframe
    def hand_open(self,width) :
        self.finger1.pos=(0,width/2.0+0.002,0.025)
        self.finger2.pos=(0,-width/2.0-0.002,0.025)
        sleep(0.5)
    hand.open=new.instancemethod(hand_open,hand,hand.__class__)
    def hand_close(self,width=0.0) :
        self.finger1.pos=(0,width/2.0+0.002,0.025)
        self.finger2.pos=(0,-width/2.0-0.002,0.025)
        sleep(0.5)
    hand.close=new.instancemethod(hand_close,hand,hand.__class__)    
    hand.set_trans(FRAME(xyzabc=[0,0,0.09,pi,0,0]))
    return hand
