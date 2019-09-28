# -*- coding: utf-8 -*- 
# from __future__ import print_function
# import pixy
# from ctypes import *
# from pixy import *
import math as ma
import numpy as np

test_data = 68, 150  # test input
# hight = 495-114 #mm

# ball parameter
r_ball = 114.8  # mm

'PIXY Parameter'
# pixy-cam image size in pixy-coordination
delta_X_pixy = 207
delta_Y_pixy = 315
dt = 60 #Hz Abtastfrequenz

# angles
ang_cam_tilt = ma.radians(33)
ang_rev_cam_tilt = ma.radians(90) - ang_cam_tilt
ang_cam_flare_X = ma.radians(47)  # Öffnungswinkel pixy cam
ang_cam_flare_Y = ma.radians(75)
delta_ang_X = ma.radians(47)  # deg
ang_ball_ratio = delta_X_pixy / delta_ang_X

'''PIXY Parameter'''
# pixy-cam position parameter
'vorläufige Parameter!!!'
ang_offset = ma.radians(30)  # 1.0472  # 60 degree
h_cam_offset = 463  # floor to camera-mountings rotation axle
s_cam_offset = 77  # midpoint robot to camera mounting on x axle
r_cam_rot = 55  # radius of camera rotation circle on xy-level

# absolute position of camera on robot
X_cam_pos = 0
Z_cam_pos = 0
# pixy-cam image parameter
Xmin_image_cam = 0
Xmax_image_cam = 0
X_offset_image_cam = 0
Ymin_image_cam = 0
Ymax_image_cam = 0
delta_X_image_cam = 0
delta_Y_image_cam = 0
X_factor_cam = 1
Y_factor_cam = 1
P_ball_X = 0  # mm
P_ball_Y = 0  # mm
V_ball_X = 0
V_ball_Y= 0
P_ball_old = (0.0, 0.0)

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
    return (ma.sin(alpha) * r) + h, (ma.cos(alpha) * r) + s


def image_position(Z_cam, X_cam, gamma, delta_X, delta_Y):
    global Xmin_image_cam  # X_offset_image_cam
    global X_offset_image_cam
    Xmax_image_cam = Z_cam / ma.tan(gamma - (delta_X / 2))
    Xmin_image_cam = Z_cam / ma.tan(gamma + (delta_X / 2))
    # cam_offset in  image on X axle:
    X_offset_image_cam = (Xmax_image_cam + Xmin_image_cam) / 2

    print("Xmax, Xmin: ", Xmax_image_cam, Xmin_image_cam)
    Ymax_image_cam = Xmax_image_cam / ma.tan(delta_Y / 2)
    Ymin_image_cam = Xmin_image_cam / ma.tan(delta_Y / 2)

    return Xmax_image_cam - Xmin_image_cam, Ymax_image_cam - Ymin_image_cam


# configuration of pixy-cam results before working
def cam_config():
    Z_cam_pos, X_cam_pos = cam_position(ang_cam_tilt, h_cam_offset, s_cam_offset, r_cam_rot)
    x, y = image_position(Z_cam_pos, X_cam_pos, ang_rev_cam_tilt, ang_cam_flare_X, ang_cam_flare_Y)
    print("Cam Pos ", Z_cam_pos, X_cam_pos)
    print("Img Pos ", x, y)

    return (x / delta_X_pixy), (y / delta_Y_pixy)


# process new data from pixy-cam
def cam_process(raw_data):
    x, y = raw_data
    # print("Xmin: ", Xmin_image_cam)
    ang_ball = ang_ball_ratio * x
    # X_value = X gemessen + kleinstmöglicher X gemessen Wert + cam vor midpoint robot + bild des balls offset zum midpoint ball
    return (x * X_factor_cam) + X_offset_image_cam + s_cam_offset + (r_ball * ma.cos(ang_ball)), (y * Y_factor_cam)


def offset_start(init_data):
    init_data = []

    print("offset calc start...")
    for count in range(0, 200):
        init_data.append(cam_process(test_data))
    offset = np.array(init_data)
    return np.median(offset, axis=0)


'MAIN'
# Config Parameter
# pixy_start()
X_factor_cam, Y_factor_cam = cam_config()
offset_pixy = offset_start(test_data)
print("offset_pixy: ", offset_pixy)
# Execute:
'DEBUG OUTPUT'
print("Factors: ", X_factor_cam, Y_factor_cam)


def motion_modell(p_old, p_new, dt):
    p_new_X, p_new_Y= p_new
    v_new_X = (p_new_X - p_old[0]) * dt
    v_new_Y = (p_new_Y - p_old[1]) * dt
    return p_new_X, p_new_Y, v_new_X, v_new_Y


Position_Error = []
Vel_Error = []
P_ball_old = offset_start(test_data) #offset wert ist startwert
for i in range(0, 6):  # range(start,end,step)

    P_ball_X, P_ball_Y, V_ball_X, V_ball_Y = motion_modell(P_ball_old, cam_process(test_data), dt)
    P_ball_old = P_ball_X, P_ball_Y
    print("Pos in mm:", P_ball_X, P_ball_Y, "V in mm/s: ", V_ball_X, V_ball_Y)
    P_ball = P_ball_X,P_ball_Y
    Position_Error.append(P_ball)
    V_ball = V_ball_X, V_ball_Y
    Vel_Error.append(V_ball)

import matplotlib.pyplot as plt

plt.plot(range(len(Position_Error)), np.array(Position_Error), label='Pos')
plt.plot(range(len(Vel_Error)), np.array(Vel_Error), label='Vel')
plt.legend()
plt.show()
