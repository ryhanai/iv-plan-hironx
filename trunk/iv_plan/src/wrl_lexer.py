#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import ply.lex as lex

reserved = {
    'PROTO' : 'PROTO',
    'DEF' : 'DEF',
    
    # field qualifiers
    'field' : 'FIELD',
    'exposedField' : 'EXPOSEDFIELD',
    'eventIn' : 'EVENTIN',

    # simple type names
    'MFNode' : 'MFNODE',
    'MFFloat' : 'MFFLOAT',
    'MFString' : 'MFSTRING',
    'SFNode' : 'SFNODE',
    'SFRotation' : 'SFROTATION',
    'SFVec3f' : 'SFVEC3F',
    'SFFloat' : 'SFFLOAT',
    'SFString' : 'SFSTRING',
    'SFInt32' : 'SFINT32',
    'IS' : 'IS',

    # 'Transform' : 'TRANSFORM',
    # 'Group' : 'GROUP',
    # 'Box' : 'BOX',
    # 'Inline' : 'INLINE',
    # 'NavigationInfo' : 'NAVIGATIONINFO',
    # 'Background' : 'BACKGROUND',
    # 'Viewpoint' : 'VIEWPOINT',
    # 'Shape' : 'SHAPE',

   'geometry' : 'KEYWORD',
   'appearance' : 'KEYWORD',
   'material' : 'KEYWORD',
   'coord' : 'KEYWORD',
   'normal' : 'KEYWORD',
   'color' : 'KEYWORD',

   'USE' : 'USE',
   'NULL' : 'NULL',
   }

def unique(ls):
   return sorted(set(ls), key=ls.index) 

tokens = [
    'IDENT',
    'FLOAT',
    'INT',
    'BOOL',
    'STRING',
    'COMMENT_LINE',
    'NEWLINE',
    'LBRCKT',
    'RBRCKT',
    'LBRC',
    'RBRC',
    'COMMA',
    'DOT',
    ] + unique(list(reserved.values()))


def t_FLOAT(t):
    r'-?((\d+\.\d*(([eE][\+-]?\d+)|f|F)?)|(\.\d+(([eE][\+-]?\d+)|f|F)?)|(\d+([eE][\+-]?\d+)[fF]?))'
    try:
         t.value = float(t.value)
    except ValueError:
         lexer_warn("Line %d: Number %s is too large!" % (t.lineno,t.value))
         t.value = 0
    return t

def t_BOOL(t):
    r'TRUE|FALSE'
    t.value = (t.value == 'TRUE')
    return t

def t_INT(t):
    r'-?(0x[0-9a-fA-F]+|\d+)'
    try:
        t.value = int(t.value)
    except ValueError:
        lexer_warn("Line %d: Number %s is too large!" % (t.lineno,t.value))
        t.value = 0
    return t

def t_STRING(t):
    r'\"(\\\"|[^\"])*\"'
    t.value = t.value[1:-1]
    return t

def t_IDENT(t):
    r'[a-zA-Z_]\w*'
    t.type = reserved.get(t.value, 'IDENT')
    return t

def t_COMMENT_LINE(t):
    r'\#.*\n'

def t_NEWLINE(t):
    r'[\r\n]+'
    t.lineno += len(t.value)


t_ignore = ' \t'
t_LBRCKT = r'\['
t_RBRCKT = r'\]'
t_LBRC = r'\{'
t_RBRC = r'\}'
t_COMMA = r','
t_DOT = r'\.'


def t_error(t):
    print 'Illegal character "%s"' % t.value[0]
    # t.skip(1)


def test_lexing(infile):
    # data = '''
    #   PROTO Joint [
    #    exposedField SFVec3f center 0.12 0 -3e-007
    #   ]
    # '''

    lexer = lex.lex()
    
    f = open(infile, 'r')
    data = f.read()
    f.close()
    
    lexer.input(data)
    
    while True:
        tok = lexer.token()
        if not tok: break # no more input
        print tok
        # print tok.type, tok.value
        # print tok.lineno, tok.lexpos
        # if tok.value == 'DEF':
        #     nametok = lexer.token()
        #     typtok =lexer.token()
        #     print nametok, typtok



def new_lexer():
    return lex.lex()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'input file not given'
    else:
        test_lexing(sys.argv[1])
        
