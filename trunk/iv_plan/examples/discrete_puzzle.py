#!/usr/bin/env python
# -*- coding: utf-8 -*-

##
## discrete_puzzl.py
##
## R.Hanai 2011.15. -
##

from numpy import *
import operator
import time

# unit vectors
exs = [array([1,0,0]),array([-1,0,0])]
eys = [array([0,1,0]),array([0,-1,0])]
ezs = [array([0,0,1]),array([0,0,-1])]

# piece原点: 3^3=27通り
def make_poss():
    poss = []
    for i in range(3):
        for j in range(3):
            for k in range(3):
                poss.append(array([i,j,k]))
    return poss

# piece姿勢: x軸=6通り * y軸=4通り = 24通り
def make_oris():
    oris = []
    for ex_prime in exs:
        for ey_prime in eys+ezs:
            oris.append([ex_prime, ey_prime])
    for ex_prime in eys:
        for ey_prime in ezs+exs:
            oris.append([ex_prime, ey_prime])
    for ex_prime in ezs:
        for ey_prime in exs+eys:
            oris.append([ex_prime, ey_prime])
    return oris

poss = make_poss()
oris = make_oris()

class PhPiece:
    def __init__(self, no, col, shape):
        self.no = no
        self.col = col
        self.shape = shape
        self.qs = self.valid_piece_configs()

    # 各pieceについて可能な配置をオフラインで計算する
    # 27*6*4通りの内，3x3x3の領域外に出るもの，対称性により重複するものを除く
    def valid_piece_configs(self):
        qs = []
        ps = []
        fs = []
        for pos in poss:
            for ori in oris:
                shp = self.put(pos, ori)
                if piece_inside(shp):
                    cube = zeros([3,3,3]).tolist()
                    for i,j,k in shp:
                        cube[i][j][k] = 1
                    if not (cube in ps):
                        qs.append(shp)
                        ps.append(cube)
                        fs.append((pos,ori))
        return zip(qs,ps,fs)

    def put(self, pos, ori):
        ex, ey = ori
        ez = cross(ex, ey)
        T = transpose(array([ex,ey,ez]))
        return [dot(T, p) + pos for p in self.shape]

    def __repr__(self):
        return '<%d %s>'%(self.no, self.col)

    def __str__(self):
        return self.__repr__()

        
def piece_inside(poss):
    def pos_inside(pos):
        return reduce(operator.__and__, map(lambda x: x>=0 and x<=2, pos))
    return reduce(operator.__and__, map(pos_inside, poss))

def put(q, p, f, state, piece):
    newstate = copy(state)
    for pos in q:
        x,y,z = pos
        if state[x,y,z] > 0:
            return None
        else:
            newstate[x,y,z] = piece.no
    # pruning heuristics
    return newstate

nsols = 0
sols = []

def print_solution(nsols, sol):
    print 'solution: %d '%nsols
    for i,p in sol:
        print ' ', p,
        print p.qs[i][2]

class SolutionsFound(Exception):
    def __init__(self):
        pass

def solve(pieces, state, parsol=[], maxsolutions=10, debug=False):
    global nsols, sols
    if pieces == []:
        nsols += 1
        sols.append(parsol)
        print_solution(nsols, parsol)
        if nsols >= maxsolutions:
            raise SolutionsFound()
        return None
    piece = pieces[0]
    for i,(q,p,f) in enumerate(piece.qs):
        newstate = put(q, p, f, state, piece)
        if not newstate == None:
            if debug:
                print 'put: ', piece
                print newstate
                raw_input()
            parsol.append((i,piece))
            solve(pieces[1:], newstate, parsol, maxsolutions=maxsolutions, debug=debug)
            parsol.pop()

# definition of pieces
pieces = [PhPiece(1, 'brown', [[0,0,0],[0,1,0],[1,0,0],[1,1,0]]),
         PhPiece(2, 'aqua', [[0,0,0],[0,1,0],[0,2,0],[1,1,0]]),
         PhPiece(3, 'yellow', [[0,0,0],[0,1,0],[0,2,0],[1,0,0]]),
         PhPiece(4, 'red', [[0,0,0],[1,0,0],[1,1,0],[2,1,0]]),
         PhPiece(5, 'green', [[0,0,0],[0,1,0],[1,0,0],[0,0,1]]),
         PhPiece(6, 'purple', [[0,0,0],[1,0,0],[1,1,0],[0,0,1]]),
         PhPiece(7, 'yellow-green', [[0,0,0],[0,1,0],[1,0,0]])]
# pieces = [PhPiece(1, 'brown', [[0,0,0],[0,1,0],[1,0,0],[1,1,0]]),
#          PhPiece(2, 'aqua', [[0,0,0],[0,1,0],[0,2,0],[1,1,0]]),
#          PhPiece(3, 'yellow', [[0,0,0],[0,1,0],[0,2,0],[1,0,0]]),
#          PhPiece(4, 'red', [[0,0,0],[1,0,0],[1,1,0],[2,1,0]]),
#          PhPiece(5, 'green', [[0,0,0],[0,1,0],[1,0,0],[0,0,1]]),
#          PhPiece(6, 'purple', [[0,0,0],[1,0,0],[1,1,0],[0,0,1]]),
#          PhPiece(7, 'yellow-green', [[0,0,0],[0,1,0],[1,0,0]])]


def run(n=10, debug=False):
    global nsols, sols
    nsols = 0
    sols = []
    t1 = time.time()
    try:
        solve(pieces, zeros([3,3,3]), parsol=[], maxsolutions=n, debug=debug)
    except:
        pass
    t2 = time.time()
    print (t2-t1),
    print ' [secs]'
