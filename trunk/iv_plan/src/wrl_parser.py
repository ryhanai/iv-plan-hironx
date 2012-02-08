#!/usr/bin/env python
# -*- coding: utf-8 -*-

# depend: python-ply 3.3-1

# The current parser doesn't interpret PROTO definitions.
# Node expressions are parsed in an a priori way.
# Syntax check is loose, resulting in accepting many illegal expressions.
# Undefined tags are not exactly checked.


import sys
import re
import pprint

import ply.yacc as yacc
from wrl_lexer import tokens,new_lexer

import numpy

# from subprocess import Popen,PIPE
# import pickle


def p_toplevel(p):
    '''toplevel : proto_nodes nodes'''
    p[0] = p[2]

def p_proto_nodes(p):
    '''proto_nodes : proto_nodes proto_node
                   | empty'''

def p_nodes(p):
    '''nodes : node nodes
             | node COMMA nodes
             | empty'''
    if len(p) == 2:
        p[0] = []
    elif p[2] == ',':
        p[3].append(p[1])
        p[0] = p[3]
    else:
        p[2].append(p[1])
        p[0] = p[2]

def p_proto_node(p):
    '''proto_node : PROTO IDENT LBRCKT proto_field_defs RBRCKT LBRC impl_exp RBRC'''

def p_proto_field_defs(p):
    '''proto_field_defs : proto_field_def proto_field_defs
                        | empty'''

def p_proto_field_def(p):
    '''proto_field_def : FIELD simple_type_exp ident_and_reserved const_exp
                       | EXPOSEDFIELD simple_type_exp ident_and_reserved const_exp
                       | EVENTIN simple_type_exp ident_and_reserved'''

def p_ident_and_reserved(p):
    '''ident_and_reserved : IDENT
                          | KEYWORD'''
    p[0] = p[1]

def p_simple_type_exp(p):
    '''simple_type_exp : MFNODE
                       | MFFLOAT
                       | MFSTRING
                       | SFNODE
                       | SFROTATION
                       | SFVEC3F
                       | SFFLOAT
                       | SFSTRING
                       | SFINT32
                       | IS'''

def p_impl_exps(p):
    '''impl_exps : impl_exp impl_exps
                 | empty'''

def p_impl_exp(p):
    '''impl_exp : IDENT LBRC impl_fields RBRC'''

def p_impl_fields(p):
    '''impl_fields : impl_field impl_fields
                   | empty'''

def p_impl_field(p):
    '''impl_field : IDENT IS IDENT
                  | IDENT LBRCKT impl_exps RBRCKT'''

# General Transform expression can come here

def p_def_opt(p):
    '''def_opt : DEF IDENT
               | empty'''
    if len(p) == 3:
        p[0] = p[2]
    else:
        p[0] = p[1]

def p_node_qualifier_opt(p):
    '''node_qualifier_opt : KEYWORD
                          | empty'''
    p[0] = p[1]

def p_node(p):
    '''node : node_qualifier_opt def_opt IDENT LBRC nodes RBRC
            | node_qualifier_opt def_opt IDENT const_exp
            | node_qualifier_opt def_opt IDENT LBRCKT nodes RBRCKT
            | USE IDENT'''
    # USE statements
    if p[1] == 'USE':
        p[0] = p[2]

    # just ignore DEF and an identifier

    elif p[3] == 'Joint':
        node = {'type' : p[3], 'name' : p[2],
                'jointAxis' : None, 'jointType' : None,
                'jointId' : None, 'translation' : None,
                'rotation' : None}
        for subnd in p[5]:
            node.update(subnd)
        p[0] = node

    elif p[3] == 'Segment':
        node = {'type' : p[3], 'name' : p[2]}
        for subnd in p[5]:
            node.update(subnd)
        p[0] = node            

    elif p[3] == 'Humanoid':
        node = {'type' : p[3], 'name' : p[2]}
        for subnd in p[5]:
            node.update(subnd)
        p[0] = node

    elif p[3] == 'Transform':
        node = {'type' : p[3]}
        for subnd in p[5]:
            node.update(subnd)
        p[0] = node

    elif p[3] == 'Viewpoint' or p[3] == 'Background' or p[3] == 'NavigationInfo':
        node = {'type' : p[3]}
        for subnd in p[5]:
            node.update(subnd)
        p[0] = node

    # with qualifier
    # 'geometry' | 'appearance' | 'material' | 'coord' | 'normal'

    elif p[1]:
        node = {'type' : p[3]}
        for subnd in p[5]:
            node.update(subnd)
        p[0] = node

    elif p[3] == 'unknown': # used in RH2
        p[0] = {'type' : p[3]}

    else:
        if len(p) == 7: # with enclosing brackets/braces
            p[0] = {p[3] : p[5]}
        else:
            p[0] = {p[3] : p[4]}

def p_const_exp(p):
    '''const_exp : prim_const_exp
                 | LBRCKT RBRCKT
                 | LBRCKT const_exps RBRCKT'''
    if len(p) == 2:
        p[0] = p[1]
    elif len(p) == 3:
        p[0] = []
    else:
        p[2].reverse()
        p[0] = p[2]

def p_const_exps(p):
    '''const_exps : const_exp COMMA const_exps
                  | const_exp COMMA
                  | const_exp'''
    if len(p) <= 3:
        p[0] = [p[1]]
    else:
        p[3].append(p[1])
        p[0] = p[3]

def p_prim_const_exp(p):
    '''prim_const_exp : STRING
                      | NULL
                      | BOOL
                      | numbers
                      '''
    p[0] = p[1]

def p_numbers(p):
    '''numbers : numbers number
               | number'''
    if len(p) == 2:
        p[0] = p[1]
    else:
        if type(p[1]) != list:
            p[1] = [p[1]]
        p[1].append(p[2])
        p[0] = p[1]

def p_number(p):
    '''number : FLOAT
              | INT'''
    p[0] = p[1]

def p_empty(p):
    'empty :'
    pass

def p_error(p):
    print 'Syntax error: ', p


# def include_another_wrl(wrlfile):
#     print 'Including ', wrlfile
#     wrlpath = 'externals/models/HIRO_090605/' + wrlfile
#     proc = Popen(['./wrl_loader.py', wrlpath, 'transform_exp'],
#                  stdout=PIPE,
#                  stderr=PIPE)
#     str = proc.communicate()[0]
#     return str

# def include_another_wrl(wrlfile):
#     wrlpath = 'externals/models/HIRO_090605/' + wrlfile
#     return parse_wrl(wrlpath, 'transform_def')


def parse_wrl(wrlfile, startsym='toplevel', debug=False):
    global start
    start = startsym
    lexer = new_lexer()
    if debug:
        parser = yacc.yacc(start=start)
    else:
        parser = yacc.yacc(start=start, errorlog=yacc.NullLogger())

    with open(wrlfile, 'r') as f:
        return parser.parse(f.read(), lexer=lexer)


if __name__ == '__main__':

    if len(sys.argv) == 3:
        startsym = sys.argv[2]
    else:
        startsym = 'toplevel'

    if len(sys.argv) < 2:
        print 'input file not given'
    else:
        wrlfile = sys.argv[1]
        ast = parse_wrl(wrlfile, startsym)
        pprint.pprint(ast)
