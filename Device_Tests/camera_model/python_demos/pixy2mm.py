
# -*- coding: utf-8 -*- 
from __future__ import print_function
#import pixy 
from ctypes import *
#from pixy import *
import math as ma

test_data = 120,100 #test input
#hight = 495-114 #mm

#ball parameter
r_ball = 114.8 #mm

'PIXY Parameter'
#pixy-cam image size in pixy-coordination
delta_X_pixy = 207
delta_Y_pixy = 315

#angles
ang_cam_tilt = ma.radians(33)
ang_rev_cam_tilt =  ma.radians(90)-ang_cam_tilt
ang_cam_flare_X = ma.radians(47) #Öffnungswinkel pixy cam
ang_cam_flare_Y = ma.radians(75)
delta_ang_X = ma.radians(47) #deg
ang_ball_ratio = delta_X_pixy / delta_ang_X

'PIXY Parameter'
#pixy-cam position parameter
'vorläufige Parameter!!!'
ang_offset = ma.radians(30) #1.0472  # 60 degree
h_cam_offset = 463 #floor to camera-mountings rotation axle
s_cam_offset =  77 #midpoint robot to camera mounting on x axle
r_cam_rot = 55 #radius of camera rotation circle on xy-level

#absolute position of camera on robot
X_cam_pos = 0
Z_cam_pos = 0
#pixy-cam image parameter
Xmin_image_cam = 0
Xmax_image_cam = 0
X_offset_image_cam = 0
Ymin_image_cam = 0
Ymax_image_cam = 0
delta_X_image_cam = 0
delta_Y_image_cam = 0
X_factor_cam = 1
Y_factor_cam = 1
X_ball_ego = 0 #mm
Y_ball_ego = 0 #mm



# Pixy2 Python SWIG get blocks example 
'''
blocks = BlockArray(100)
frame = 0

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

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
	#Anpassung des Pixy-Koordinatensystems an das Roboterkoordinatensystem 
      print("X_raw|Y_raw) ",blocks[index].m_y,blocks[index].m_x)
      X_ball = 103.5 - (blocks[index].m_y) # - 103.5) #*(-1) #Y_pixy_max / 2 
      Y_ball = (blocks[index].m_x - 157.5) #*(-1) #X_pixy_max / 2
      print('(X|Y)=  ',X_ball,'|',Y_ball)
   return X_ball,Y_ball
'''
   
# pixy-cam position from midpoint robot in relation to tilt angle alpha
def cam_position(alpha, h, s, r):

    return (ma.sin(alpha)*r) + h, (ma.cos(alpha)*r) + s

def image_position(Z_cam, X_cam, gamma, delta_X, delta_Y):
    global Xmin_image_cam #X_offset_image_cam
    global X_offset_image_cam
    Xmax_image_cam = Z_cam / ma.tan(gamma - (delta_X / 2))
    Xmin_image_cam = Z_cam / ma.tan(gamma + (delta_X / 2))
    #cam_offset in  image on X axle:
    X_offset_image_cam = (Xmax_image_cam + Xmin_image_cam)/2

    print("Xmax, Xmin: ", Xmax_image_cam, Xmin_image_cam)
    Ymax_image_cam = Xmax_image_cam / ma.tan(delta_Y / 2)
    Ymin_image_cam = Xmin_image_cam / ma.tan(delta_Y / 2)

    return Xmax_image_cam - Xmin_image_cam, Ymax_image_cam - Ymin_image_cam

#configuration of pixy-cam results before working
def cam_config():
    Z_cam_pos, X_cam_pos = cam_position(ang_cam_tilt, h_cam_offset, s_cam_offset, r_cam_rot)
    x, y = image_position(Z_cam_pos, X_cam_pos, ang_rev_cam_tilt, ang_cam_flare_X, ang_cam_flare_Y)
    print("Cam Pos ", Z_cam_pos, X_cam_pos)
    print("Img Pos ", x, y)

    return (x / delta_X_pixy), (y / delta_Y_pixy)

#process new data from pixy-cam
def cam_process(raw_data):
    x,y = raw_data
    print("Xmin: ", Xmin_image_cam)
    ang_ball = ang_ball_ratio * x
    #X_value = X gemessen + kleinstmöglicher X gemessen Wert + cam vor midpoint robot + bild des balls offset zum midpoint ball
    return (x * X_factor_cam) + X_offset_image_cam + s_cam_offset + (r_ball * ma.cos(ang_ball)), (y * Y_factor_cam)
 
'MAIN'

#pixy_start()
X_factor_cam, Y_factor_cam = cam_config()
X_ball_ego, Y_ball_ego = cam_process(test_data) #get_pixy())

'DEBUG OUTPUT'
print("Factors: ",X_factor_cam, Y_factor_cam)
print("Länge in mm:", X_ball_ego, Y_ball_ego)
#print(math.tan(math.radians(45)))

