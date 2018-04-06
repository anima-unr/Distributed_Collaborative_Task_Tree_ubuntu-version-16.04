#!/usr/bin/env python

import sys
import pdb
import pickle
'''
Description: A program to parse the Task Expression Language
Output => A preorder tree traversal and a YAML file
'''

Symbol = str
List = list
Number = (int, float)

def tokenize(chars):
  '''
  Teokenize the input stream of characters based on the language
  '''
  return chars.replace('(', ' ( ').replace(')', ' ) ').split()

def read_from_tokens(tokens):
  if len(tokens) == 1:
    raise SyntaxError('Unexpected EOF while reading')
  token = tokens.pop(0)
  if '(' == token:
    L = []
    while tokens[0] != ')':
      L.append(read_from_tokens(tokens))
    tokens.pop(0)
    return L
  elif ')' == token:
    raise SyntaxError('Unexpected [)] symbol')
  else:
    return atom(token)

def atom(token):
  try: return int(token)
  except ValueError:
    try: return float(token)
    except ValueError:
      return Symbol(token)

def parse(program):
  '''
  Parse the input character stream and output syntax tree
  '''
  return read_from_tokens(tokenize(program))
class Procedure(object):
  "A user-defined Scheme procedure."
  def __init__(self, parms, body, env):
    self.parms, self.body, self.env = parms, body, env
  def __call__(self, *args): 
    return eval(self.body, Env(self.parms, args, self.env))

class Env(dict):
  "An environment: a dict of {'var':val} pairs, with an outer Env."
  def __init__(self, parms=(), args=(), outer=None):
    self.update(zip(parms, args))
    self.outer = outer
  def find(self, var):
    "Find the innermost Env where var appears."
    return self if (var in self) else self.outer.find(var)

def repl(prompt='lis.py> '):
  "A prompt-read-eval-print loop."
  while True:
    input_ = raw_input(prompt)
    if input_:
      val = eval(parse(input_))
      if val is not None: 
        print(schemestr(val))

def schemestr(exp):
  "Convert a Python object back into a Scheme-readable string."
  if  isinstance(exp, list):
    return '(' + ' '.join(map(schemestr, exp)) + ')' 
  else:
    return str(exp)

def standard_env():
  import math, operator as op
  env = Env()
  env.update(vars(math)) # sin, cos, sqrt, pi, ...
  env.update({
    '+':op.add, '-':op.sub, '*':op.mul, '/':op.div, 
    '>':op.gt, '<':op.lt, '>=':op.ge, '<=':op.le, '=':op.eq, 
    'abs':     abs,
    'append':  op.add,  
    'apply':   apply,
    'begin':   lambda *x: x[-1],
    'car':     lambda x: x[0],
    'cdr':     lambda x: x[1:], 
    'cons':    lambda x,y: [x] + y,
    'eq?':     op.is_, 
    'equal?':  op.eq, 
    'length':  len, 
    'list':    lambda *x: list(x), 
    'list?':   lambda x: isinstance(x,list), 
    'map':     map,
    'max':     max,
    'min':     min,
    'not':     op.not_,
    'null?':   lambda x: x == [], 
    'number?': lambda x: isinstance(x, Number),   
    'procedure?': callable,
    'round':   round,
    'symbol?': lambda x: isinstance(x, Symbol),
  })
  return env
global_env = standard_env()

# def eval(x, env=global_env):
#   "Evaluate an expression in an environment."
#   if isinstance(x, Symbol):      # variable reference
#     return env[x]
#   elif not isinstance(x, List):  # constant literal
#     return x                
#   elif x[0] == 'quote':          # (quote exp)
#     (_, exp) = x
#     return exp
#   elif x[0] == 'if':             # (if test conseq alt)
#     (_, test, conseq, alt) = x
#     exp = (conseq if eval(test, env) else alt)
#     return eval(exp, env)
#   elif x[0] == 'define':         # (define var exp)
#     (_, var, exp) = x
#     env[var] = eval(exp, env)
#   else:                          # (proc arg...)
#     proc = eval(x[0], env)
#     args = [eval(arg, env) for arg in x[1:]]
#     return proc(*args)


def DepthFirstOrder(tree, i=0):
  if isinstance(tree, list):
    output = []
    for elem in tree:
      output += DepthFirstOrder(elem)
    return output
  else:
    return [tree]

def eval(x, env=global_env):
  "Evaluate an expression in an environment."
  if isinstance(x, Symbol):      # variable reference
    return env.find(x)[x]
  elif not isinstance(x, List):  # constant literal
    return x                
  elif x[0] == 'quote':          # (quote exp)
    (_, exp) = x
    return exp
  elif x[0] == 'if':             # (if test conseq alt)
    (_, test, conseq, alt) = x
    exp = (conseq if eval(test, env) else alt)
    return eval(exp, env)
  elif x[0] == 'define':         # (define var exp)
    (_, var, exp) = x
    env[var] = eval(exp, env)
  elif x[0] == 'set!':           # (set! var exp)
    (_, var, exp) = x
    env.find(var)[var] = eval(exp, env)
  elif x[0] == 'lambda':         # (lambda (var...) body)
    (_, parms, body) = x
    return Procedure(parms, body, env)
  else:                          # (proc arg...)
    proc = eval(x[0], env)
    args = [eval(arg, env) for arg in x[1:]]
    return proc(*args)

if __name__ == '__main__':
  # eval(parse("(define x 10)"))
  # print eval(parse("(+  4 (+ 4 (+ 4 x)))"))
  # output preorder tree traversal

  # Perform Depth first search
  string = "(THEN_0_1_001 PLACE_3_1_002 (AND_3_1_003 (OR_3_1_004 PLACE_3_1_009 PLACE_3_1_010 PLACE_3_1_011) PLACE_3_1_005 PLACE_3_1_006 (THEN_0_1_007 PLACE_3_1_012 PLACE_3_1_013) PLACE_3_1_008))"
  parse_str = parse(string)
  order = dict()
  for i, node in enumerate(DepthFirstOrder(parse_str)):
    order[node] = i
  with open('table_setting_demo_node_graph_order.txt', 'w') as file:
    file.write(pickle.dumps(order))