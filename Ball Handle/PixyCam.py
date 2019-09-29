# -*- coding: utf-8 -*-
# from __future__ import print_function
import time
from ctypes import *

import numpy as np
import pixy
from classes import Debug, KalmanFilter
from pixy import *

test_data = 150, 100

# ball status
Ball_Status = 0
FAR_BALL = 1
NEAR_BALL = 2
HAVE_BALL = 3
CONQUER_BALL = 4

r_ball = 114.8  # mm
origin = (0, 0)
ideal_line = 200  # distance of ball midpoint from robot for dribbling
ball_distance_max = 385  # border for making decision if have oder not

'''
Important Information:
Execute with sudo!!!
'''

class Blocks(Structure):
    _fields_ = [("m_signature", c_uint),
                ("m_x", c_uint),
                ("m_y", c_uint),
                ("m_width", c_uint),
                ("m_height", c_uint),
                ("m_angle", c_uint),
                ("m_index", c_uint),
                ("m_age", c_uint)]


class PixyCam(Debug, Blocks, KalmanFilter):
    def __init__(self, sim_mode=False, ball_dis_max=ball_distance_max):
        self.debug = Debug('pixy')
        self.sim_mode = sim_mode
        self.offset_X = 0
        self.offset_Y = 0

        if sim_mode == False:
            self.start()
            self.blocks = BlockArray(100)
            #self.offset_calc() # NOT USED AT THE MOMENT

        self.raw = self.read_raw()
        self.r_ball = 114.8  # mm


        'PIXY Parameter'
        # pixy-cam image size in pixy-coordination
        self.delta_X_pixy = 207
        self.delta_Y_pixy = 315

        self.dt = 50  # Hz Abtastfrequenz
        # angles
        self.ang_cam_tilt = np.radians(33)
        self.ang_rev_cam_tilt = np.radians(90) - self.ang_cam_tilt
        self.ang_cam_flare_X = np.radians(47)  # Öffnungswinkel pixy cam in robot cs
        self.ang_cam_flare_Y = np.radians(75)
        self.delta_ang_X = np.radians(47)  # deg
        self.ang_ball_ratio = self.delta_X_pixy * 0.5 / self.delta_ang_X
        # pixy-cam position parameter
        'vorläufige Parameter!!!'
        self.ang_offset = np.radians(30)  # degree
        self.h_cam_offset = 463  # -114.2#463  # floor to camera-mountings rotation axle
        self.s_cam_offset = 77  # midpoint robot to camera mounting on x axle
        self.r_cam_rot = 55  # radius of camera rotation circle on xy-leve
        # absolute position of camera on robot
        self.X_cam_pos = 0
        self.Z_cam_pos = 0
        # pixy-cam image parameter
        self.Xmin_image_cam = 0
        self.Xmax_image_cam = 0
        self.X_offset_image_cam = 0
        self.Ymin_image_cam = 0
        self.Ymax_image_cam = 0
        self.delta_X_image_cam = 0
        self.delta_Y_image_cam = 0
        self.X_factor_cam, self.Y_factor_cam = self.cam_config()
        self.P_ball_X = 0  # mm
        self.P_ball_Y = 0  # mm
        self.V_ball_X = 0
        self.V_ball_Y = 0
        self.P_ball_old = (0.0, 0.0)
        self.P_ball_new = 0
        self.Ball_distance_Max = ball_dis_max
        self.detect_count = 0

        self.kf = self.filter_config()
        self.P_ball_X_kf=0
        self.P_ball_Y_kf=0
        self.V_ball_X_kf=0
        self.V_ball_Y_kf=0
        self.Ball_Status=0



    def start(self):
        # print("Pixy2 Python SWIG Example -- Get Blocks")
        pixy.init()
        pixy.change_prog("color_connected_components")

    def read_raw(self):
        global Ball_Status

        if self.sim_mode == True:
            X_ball, Y_ball = test_data
        else:
            count = pixy.ccc_get_blocks(100, self.blocks)
            if count > 0:
                for index in range(0, count):
                    X_ball = self.blocks[index].m_x
                    Y_ball = self.blocks[index].m_y
                    #if X_ball is not None:
                    #print('(X|Y)=  ', X_ball, '|', Y_ball)
                    return X_ball, Y_ball

    def transform_coordinates(self, datain):
        # adapt pixy coordinates to robot coordiantes (0|0) is in the middle of the pic now
        X_ball, Y_ball = datain
        X_ball = -(Y_ball - (self.delta_Y_pixy)/2)  # Y_pixy_max / 2 = 103.5
        Y_ball = (X_ball - self.delta_X_pixy/2)   # X_pixy_max / 2 = 157.5
        #print("X|Y _ball transformed:",X_ball, Y_ball)
        return X_ball, Y_ball

    def offset_calc(self):
        init_data = []
        #print("offset calc start...")
        for count in range(0, 200):
            init_data.append(self.read_raw())
        offset = np.array(init_data)
        #print("offset array:",offset)
        #Median
        self.offset_X = np.median(offset[0], axis=0)
        self.offset_Y = np.median(offset[1], axis=0)
        # Standartabweichung
        #self.std_deviation_X = np.std(offset[0], axis=0)
        #self.std_deviation_Y = np.std(offset[1], axis=0)

    # pixy-cam position from midpoint robot in relation to tilt angle alpha
    def cam_position(self, alpha, h, s, r):
        return (np.sin(alpha) * r) + h, (np.cos(alpha) * r) + s

    def image_position(self, Z_cam, X_cam, gamma, delta_X, delta_Y):
        Xmax_image_cam = Z_cam / np.tan(gamma - (delta_X / 2))
        self.Xmin_image_cam = Z_cam / np.tan(gamma + (delta_X / 2))
        # cam_offset in  image on X axle: midpoint of image on x axle in mm
        self.X_offset_image_cam = X_cam + (Xmax_image_cam + self.Xmin_image_cam) / 2

        #print("Xmax, Xmin: ", Xmax_image_cam, self.Xmin_image_cam)
        #print(" Z_cam, gamma, delta_X, delta_Y", Z_cam, gamma, delta_X, delta_Y)
        Ymax_image_cam = Xmax_image_cam / np.tan(delta_Y / 2)
        Ymin_image_cam = self.Xmin_image_cam / np.tan(delta_Y / 2)
        #print("Ymax, Ymin: ", Ymax_image_cam, Ymin_image_cam)

        return Xmax_image_cam - self.Xmin_image_cam, Ymax_image_cam - Ymin_image_cam

    def filter_config(self):
        # paramter for kalman filter
        dt = 1/self.dt
        # state transition model, A
        F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
        H = np.array([1, 0, 0]).reshape(1, 3)  # transponieren #observation model C
        q = 0.05
        Q = np.array([[q, q, 0], [q, q, 0], [0, 0, 0]])  # process noise
        R = np.array([0.2]).reshape(1, 1)  # observation noise 0.8
        return KalmanFilter(F=F, H=H, Q=Q, R=R)

    def kalman_filter(self, z):
        # das ist meine C matrix für den Ausgang, also müsste das mittlere die geschwindigkeit sein
        np.dot(self.kf.H, self.kf.predict())
        self.kf.update(z)
        #print("kalmanfilter: ", self.kf.x[0], self.kf.x[1], self.kf.x[2])
        return self.kf.x[0], self.kf.x[1]

    # configuration of pixy-cam results before working
    def cam_config(self):
        self.Z_cam_pos, self.X_cam_pos = self.cam_position(self.ang_cam_tilt, self.h_cam_offset, self.s_cam_offset,
                                                           self.r_cam_rot)
        x, y = self.image_position(self.Z_cam_pos, self.X_cam_pos, self.ang_rev_cam_tilt, self.ang_cam_flare_X,
                                   self.ang_cam_flare_Y)
        #print("Cam Pos ", self.Z_cam_pos, self.X_cam_pos)
        #print("Img size ", x, y)

        return (x / self.delta_X_pixy), (y / self.delta_Y_pixy)

    # process new data from pixy-cam
    def cam_process(self, raw_data):
        x, y = raw_data
        #print("Xmin: ", self.Xmin_image_cam)
        ang_ball = self.ang_ball_ratio * x
        #print("Factory X,Y", self.X_factor_cam, self.Y_factor_cam)
        #print("X_offset_image_cam", self.X_offset_image_cam)
        x_rel = x * self.X_factor_cam
        #print("X pos relativ", x_rel)
        #print("ang_ration", self.ang_ball_ratio)
        #print("offset des blickwinkels zum ball:", self.r_ball * np.cos(ang_ball))
        # X_value = X gemessen + kleinstmöglicher X gemessen Wert + cam vor midpoint robot + bild des balls offset zum midpoint ball
        # x_result = x_rel + self.X_cam_pos + abs(self.r_ball * np.cos(ang_ball))
        # THIS WORKS:
        x_result = self.X_cam_pos + self.Xmin_image_cam + x_rel + abs(self.r_ball * np.cos(ang_ball))


        '''
        if x_result > self.X_offset_image_cam:
            x_result = x_result + (self.r_ball * np.cos(ang_ball))
        else:
            x_result = x_result - (self.r_ball * np.cos(ang_ball))
        '''

        '''
        REMEMBER: pixys picture is in real a trapez, that results in an dependency of the ymax value 
        from the current x position!       
        '''
        y_max_current = (x * self.X_factor_cam) / np.tan(self.ang_cam_flare_Y * 0.5)
        y_result = y * (y_max_current / self.delta_Y_pixy)

        #print("X|Y result: ", x_result, y_result)
        return x_result, y_result

    def motion_modell(self, p_old, p_new, dt):
        # self.P_ball_old = p_new
        p_new_X, p_new_Y = p_new
        #print("p_new", p_new)
        v_new_X = (p_new[1] - p_old[0]) * dt
        v_new_Y = (p_new[0] - p_old[1]) * dt
        # self.P_ball_old = p_new

        # print("p_new_X, p_new_Y, v_new_X, v_new_Y, P_OLD", p_new_X, p_new_Y, v_new_X, v_new_Y, self.P_ball_old)
        return v_new_X, v_new_Y

    def process(self):


        'MAIN'
        self.P_ball_new = self.read_raw()
        # print("P_new_ball", self.P_ball_new)
        if self.P_ball_new == None:
            self.Ball_Status = FAR_BALL
            self.detect_count = 0
            #return
        else:

            self.detect_count = self.detect_count + 1
            # change cs from pixy to midpoint image is (0|0)
            self.P_ball_new = self.transform_coordinates(self.P_ball_new)
            self.P_ball_new = self.cam_process(self.P_ball_new)
            # Position of ball in mm:
            self.P_ball_X, self.P_ball_Y = self.P_ball_new

            #KalmanFilter:
            self.q_dot = self.P_ball_X, self.P_ball_Y, self.V_ball_X, self.V_ball_Y
            (self.P_ball_X_kf, self.P_ball_Y_kf), (self.V_ball_X_kf, self.V_ball_Y_kf) = self.kalman_filter(self.P_ball_new)
            '''
            # need 2 measurments for calculate the velocity because kalman needs 4 iteration before
            if self.detect_count >= 2 and self.detect_count <= 4:
                self.V_ball_X, self.V_ball_Y = self.motion_modell(self.P_ball_old,
                                                                  self.P_ball_new, self.dt)
            else:
            '''
            self.P_ball_X, self.P_ball_Y, self.V_ball_X, self.V_ball_Y = self.P_ball_X_kf, self.P_ball_Y_kf, self.V_ball_X_kf, self.V_ball_Y_kf

            # test for ball status
            if self.V_ball_X <= 0 and self.P_ball_X <= self.Ball_distance_Max:
                self.Ball_Status = HAVE_BALL

            self.P_ball_old = self.P_ball_X, self.P_ball_Y
            #print("Pos in mm:", self.P_ball_X, self.P_ball_Y, "V in mm/s: ", self.V_ball_X, self.V_ball_Y)
            #print("Ball Status:", Ball_Status)
            self.Ball_Status = NEAR_BALL
        return self.Ball_Status, self.P_ball_X, self.P_ball_Y, self.V_ball_X, self.V_ball_Y

'''

def test_pixy(save=False, draw=False):
    print("stat testing...")
    cam = PixyCam(sim_mode=False)
    t_ref = int(round(time.time() * 1000))

    # while KeyboardInterrupt is not True:
    for i in range(0,300):
        try:
            cam.run()
            cam.debug.P_X, cam.debug.P_Y, cam.debug.V_X, cam.debug.V_Y = cam.P_ball_X, cam.P_ball_Y, cam.V_ball_X, cam.V_ball_Y
            cam.debug.excecute(t_ref)
        # time.sleep(1)
        except KeyboardInterrupt:
            pass
    if save:
        cam.debug.save()
    if draw:
        cam.debug.draw()
        return


# if __name__== "__main":
test_pixy(save=False, draw=False)
'''