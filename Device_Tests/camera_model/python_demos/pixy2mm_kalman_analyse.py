# -*- coding: utf-8 -*- 
#from __future__ import print_function
#import pixy
#from ctypes import *
#from pixy import *
import math as ma
import numpy as np

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

'''PIXY Parameter'''
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
X_ball_filtered = 0
Y_ball_filtered = 0
'Kalman Parameter'
dt = 1.0/60.0 #pixys freq = 60 Hz
F = np.array([[1, dt, 0], [0, 1, dt],[0, 0, 1]]) #state transition model, A 
H = np.array([1, 0, 0]).reshape(1, 3) #transponieren #observation model C
"""
das ist meine C matrix für den Ausgang, also müsste das mittlere die geschwindigkeit sein
"""
q = 0.05
Q = np.array([[q, q, 0], [q, q, 0], [0, 0, 0]])
R = np.array([0.05]).reshape(1, 1) #observation noise
    

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
'''
class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x) #das gemessne x aus der Matrix
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
   
def kalman_pixy(kf,measure):
    kf.predict()
    kf.update(measure)
    P = kf.x[0] #Position
    V = kf.x[1] #Velocity
    a = kf.x[2] #acceleration
    return P, V  
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
    #print("Xmin: ", Xmin_image_cam)
    ang_ball = ang_ball_ratio * x
    #X_value = X gemessen + kleinstmöglicher X gemessen Wert + cam vor midpoint robot + bild des balls offset zum midpoint ball
    return (x * X_factor_cam) + X_offset_image_cam + s_cam_offset + (r_ball * ma.cos(ang_ball)), (y * Y_factor_cam)

def offset_start(init_data):
    init_data = []

    print("offset calc start...")
    for count in range (0,200):
        init_data.append(cam_process(test_data))
    offset = np.array(init_data)
    return np.median(offset, axis=0)
'MAIN'
#Config Parameter
#pixy_start()
X_factor_cam, Y_factor_cam = cam_config()

offset_pixy = offset_start(test_data)
print("offset_pixy: ", offset_pixy)
kf_pixy = KalmanFilter(F = F, H = H, Q = Q, R = R)
#kf_pixy = KalmanFilter(F = F, B =offset_pixy, H = H, Q = Q, R = R)
#Execute:
'DEBUG OUTPUT'
print("Factors: ",X_factor_cam, Y_factor_cam)

#print("Filtered Data: ", X_ball_filtered, Y_ball_filtered)
#print(math.tan(math.radians(45)))
Position_Error_X = []
Position_Error_Y = []
for i in range (0,6):  #range(start,end,step)

    #X_ball_ego, Y_ball_ego = cam_process(test_data) #get_pixy())
    P_ball_ego = cam_process(test_data)  # get_pixy())
    print("Länge in mm:", X_ball_ego, Y_ball_ego)
    P_ball_filtered, V_ball_filtered = kalman_pixy(kf_pixy, cam_process(test_data))
    print("Filtered: ", P_ball_filtered, V_ball_filtered)
    '''
    P_X, P_Y = P_ball_filtered
    X_error = P_X - X_ball_ego
    Y_error = P_Y - Y_ball_ego
    print("Error: X: ", X_error,"Y: ", Y_error)
    Filter_Error_X.append(X_error)
    Filter_Error_Y.append(Y_error)
    '''
    #alternative
    #absolute
    #Filter_Error_Y.append(P_ball_filtered[1] - P_ball_ego[1])
    #Filter_Error_X.append(P_ball_filtered[0] - P_ball_ego[0])
    #relative in percent
    Position_Error_X.append(100 -(100 / P_ball_filtered[0] * P_ball_ego[0]))
    Position_Error_Y.append(100- (100 / P_ball_filtered[1] * P_ball_ego[1]))
    print("Error: X: ", Position_Error_X[-1], "Y: ",Position_Error_Y[-1] )


import matplotlib.pyplot as plt
plt.plot(range(len(Position_Error_X)), np.array(Position_Error_X), label = 'Position Error X')
plt.plot(range(len(Position_Error_Y)), np.array(Position_Error_Y), label = 'Position Error Y')
plt.legend()
plt.show()