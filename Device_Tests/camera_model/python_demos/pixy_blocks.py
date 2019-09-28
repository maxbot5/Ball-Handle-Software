# -*- coding: utf-8 -*- 
from __future__ import print_function
import pixy 
from ctypes import *
from pixy import *

# Pixy2 Python SWIG get blocks example 

blocks = BlockArray(100)
frame = 0

#paramter für umrechnung pixel  mm
h_camera = 200 #mm
r_camera =  55 #mm

alpha = 30 #deg
gamma = 60 #deg
delta_x = 47 #deg
detla_y = 75 #deg

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

'''
def pixy_conver_init(h,r,alpha,gamma,delta_x,delta_y):
  z_camera = sin(alpha)*r
  X_min = (z_camera+h)/tan(delta-(gamma/2))
  X_max = (z_camera#h)/tan(delta+(gamma/2))
  delta_X = X_max - X_min
'''
def pixy_start():
  print("Pixy2 Python SWIG Example -- Get Blocks")

  pixy.init ()
  pixy.change_prog ("color_connected_components");

def get_pixy():
  count = pixy.ccc_get_blocks (100, blocks)

  if count > 0:
#   print('frame %3d:' % (frame))
 #  frame = frame + 1
   for index in range (0, count):
    # print('[BLOCK: SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].m_signature, blocks[index].m_x, blocks[index].m_y, blocks[index].m_width, blocks[index].m_height))
     X_ball = blocks[index].m_x
     Y_ball = blocks[index].m_y
     print('(X|Y)=  ',X_ball,'|',Y_ball)
   

pixy_start()
while 1:
#for i in range (0,200):
  get_pixy()
