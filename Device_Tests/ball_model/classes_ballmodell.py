# -*- coding: utf-8 -*-
from __future__ import print_function

import csv
import os
import time

import matplotlib.pyplot as plt
import numpy as np

# import pixy
# import smbus
# from pixy import *

'''
CONFIG
'''


class State(object):  # Motion State Q (of ball or robot)
    def __init__(self, p_x, p_y, phi_z, v_x, v_y, w_z):
        self.P_X = p_x
        self.P_Y = p_y
        self.phi_Z = phi_z
        self.V_X = v_x
        self.V_Y = v_y
        self.w_Z = w_z
        self.q = (self.P_X, self.P_Y, self.phi_Z)
        self.q_dot = (self.V_X, self.V_Y, self.w_Z)
        self.Q = (self.q, self.q_dot)


class KalmanFilter(object):
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None):
        if (F is None or H is None):
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

    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)  # das gemessne x aus der Matrix
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


class Debug(object):
    def __init__(self, name):  # , p_X, p_Y, v_X, v_Y, w_Z):
        self.Position_Error = []
        self.Vel_Error = []
        self.P_X = 0
        self.P_Y = 0
        self.V_X = 0
        self.V_Y = 0
        self.w_Z = 0
        self.name = name
        self.init_folder()
        self.fileName = self.init_file(self.name)
        self.Data = []
        self.count = 0

    def init_folder(self):
        # csv-files saved in own folder
        # from: https://www.tutorialspoint.com/python/python_files_io.htm
        try:
            os.chdir("csvData")
        except:
            os.mkdir("csvData")
            os.chdir("csvData")

    def init_file(self, sensorName):
        fileName = str(sensorName) + '_' + str(time.strftime("%y%m%d")) + '_' + str(
            time.strftime("%H%M%S")) + '.csv'
        print("fileName: ", fileName)
        return fileName

    def save(self, Data):
        # --------------------
        # --- LOGGER-Part ----
        # --------------------
        npdata = np.array(Data)
        np.savetxt(self.fileName, npdata, delimiter=",")
        file = csv.writer(open(str(self.fileName), "w"))
        file.writerows(Data)

    def draw(self):
        plt.plot(range(len(self.Position_Error)), np.array(self.Position_Error), label='Position')
        plt.plot(range(len(self.Vel_Error)), np.array(self.Vel_Error), label='Velocity')
        plt.legend()
        plt.show()

    def run(self):
        print("Pos in mm:", self.P_X, self.P_Y, "V in mm/s: ", self.V_X, self.V_Y)
        pos = self.P_X, self.P_Y
        vel = self.V_X, self.V_Y, self.w_Z
        self.Position_Error.append(pos)
        self.Vel_Error.append(vel)

        try:
            now = int(round(time.time() * 1000))  # datetime.strptime('%S.%f') #datetime.now(%s)
            self.Data[self.count] = (
            str(now), str(self.P_X), str(self.P_Y), str(self.V_X), str(self.V_Y), str(self.w_Z))
        except KeyboardInterrupt:
            self.save(self.Data)
            self.draw()


'''
class Imu(object):
    def __init__(self, port):
        self.debug = Debug('imu')
        self.kf = self.filter_config()
        self.raw = self.read_raw()
        self.offset = self.offset_calc()
        self.port = port
        self.bus = smbus.SMBus(2)  # bus = smbus.SMBus(0) fuer Revision 1
        self.address = 0x68  # via i2cdetect
        self.power_mgmt_1 = 0x6b
        self.ACCEL_CONFIG = 0x1C  # Reg 28
        self.ACCEL_CONFIG2 = 0x1D  # Reg 29
        self.imu_config()

    def filter_config(self):
        # paramter for kalman filter
        dt = 1.0 / 50.0
        # state transition model, A
        F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
        H = np.array([0, 0, 1]).reshape(1, 3)  # transponieren #observation model C
        q = 0.05
        Q = np.array([[q, q, 0], [q, q, 0], [0, 0, 0]])  # process noise
        R = np.array([0.8]).reshape(1, 1)  # observation noise
        return KalmanFilter(F=F, H=H, Q=Q, R=R)

    def imu_config(self):
        # Aktivieren, um das Modul ansprechen zu koennen
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)  # full power mode
        # bus.write_byte_data(address, power_mgmt_2, 0b00001111)  #disabele=1, disabled accel_z, gyro_x bis _z
        # setzt Accelerometer Full Scale Select (hier auf +-2g)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0b00100000)
        # setzt den Tiefpass-Filter
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG2,
                                 0b00000100)  # entspricht dem Wert 4, also 19,8 ms ~50Hz

    def read_raw(self):
        def read_word(reg):
            h = self.bus.read_byte_data(self.address, reg)
            l = self.bus.read_byte_data(self.address, reg + 1)
            value = (h << 8) + l
            return value
        def read_word_2c(reg):
            val = read_word(reg)
            if (val >= 0x8000):
                return -((65535 - val) + 1)
            else:
                return val

        beschleunigung_xout = read_word_2c(0x3b)
        beschleunigung_yout = read_word_2c(0x3d)
        gyroskop_zout = read_word_2c(0x47)

        beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0  # value from sensor documentation
        beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
        gyroskop_zout_skaliert = gyroskop_zout / 131

        return beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, gyroskop_zout_skaliert

    def offset_calc(self):
        init_data = []
        print("offset calc start...")
        for count in range(0, 200):
            init_data.append(self.read_raw())
        offset = np.array(init_data)
        return np.median(offset, axis=0)

    def kalman_filter(self, z):
        # das ist meine C matrix für den Ausgang, also müsste das mittlere die geschwindigkeit sein
        np.dot(self.H, self.kf.predict())
        self.kf.update(z)
        return self.kf.x[0], self.kf.x[1], self.kf.x[2]

    def run(self):
        return self.kalman_filter(self.read_raw() - self.offset)


class Blocks(Structure):
    _fields_ = [("m_signature", c_uint),
                ("m_x", c_uint),
                ("m_y", c_uint),
                ("m_width", c_uint),
                ("m_height", c_uint),
                ("m_angle", c_uint),
                ("m_index", c_uint),
                ("m_age", c_uint)]
'''
'''
class PixyCam(Debug, Blocks):
    def __init__(self):
        self.debug = Debug('pixy')
        self.start()
        self.blocks = BlockArray(100)

        self.raw = self.read_raw()
        self.offset = self.offset_calc()
        self.r_ball = 114.8  # mm

        'PIXY Parameter'
        # pixy-cam image size in pixy-coordination
        self.delta_X_pixy = 207
        self.delta_Y_pixy = 315
        self.dt = 60  # Hz Abtastfrequenz
        # angles
        self.ang_cam_tilt = np.radians(33)
        self.ang_rev_cam_tilt = np.radians(90) - self.ang_cam_tilt
        self.ang_cam_flare_X = np.radians(47)  # Öffnungswinkel pixy cam
        self.ang_cam_flare_Y = np.radians(75)
        self.delta_ang_X = np.radians(47)  # deg
        self.ang_ball_ratio = self.delta_X_pixy / self.delta_ang_X
        # pixy-cam position parameter
        'vorläufige Parameter!!!'
        self.ang_offset = np.radians(30)  # degree
        self.h_cam_offset = 463  # floor to camera-mountings rotation axle
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
        self.X_factor_cam = 1
        self.Y_factor_cam = 1
        self.P_ball_X = 0  # mm
        self.P_ball_Y = 0  # mm
        self.V_ball_X = 0
        self.V_ball_Y = 0
        self.P_ball_old = (0.0, 0.0)
        # self.pixy_start()
        self.X_factor_cam, self.Y_factor_cam = self.cam_config()

    def start(self):
        # print("Pixy2 Python SWIG Example -- Get Blocks")
        pixy.init()
        pixy.change_prog("color_connected_components")

    def read_raw(self):
        count = pixy.ccc_get_blocks(100, self.blocks)
        if count > 0:
            for index in range(0, count):
                X_ball = self.blocks[index].m_x
                Y_ball = self.blocks[index].m_y
                # print('(X|Y)=  ', X_ball, '|', Y_ball)
                return X_ball, Y_ball

    def offset_calc(self):
        init_data = []
        print("offset calc start...")
        for count in range(0, 200):
            init_data.append(self.read_raw())
        offset = np.array(init_data)
        return np.median(offset, axis=0)

    # pixy-cam position from midpoint robot in relation to tilt angle alpha
    def cam_position(self, alpha, h, s, r):
        return (np.sin(alpha) * r) + h, (np.cos(alpha) * r) + s

    def image_position(self, Z_cam, gamma, delta_X, delta_Y):
        Xmax_image_cam = Z_cam / np.tan(gamma - (delta_X / 2))
        Xmin_image_cam = Z_cam / np.tan(gamma + (delta_X / 2))
        # cam_offset in  image on X axle:
        self.X_offset_image_cam = (self.Xmax_image_cam + self.Xmin_image_cam) / 2

        print("Xmax, Xmin: ", self.Xmax_image_cam, self.Xmin_image_cam)
        Ymax_image_cam = self.Xmax_image_cam / np.tan(delta_Y / 2)
        Ymin_image_cam = self.Xmin_image_cam / np.tan(delta_Y / 2)

        return Xmax_image_cam - Xmin_image_cam, Ymax_image_cam - Ymin_image_cam

    # configuration of pixy-cam results before working
    def cam_config(self):
        self.Z_cam_pos, self.X_cam_pos = self.cam_position(self.ang_cam_tilt, self.h_cam_offset, self.s_cam_offset,
                                                           self.r_cam_rot)
        x, y = self.image_position(self.Z_cam_pos, self.X_cam_pos, self.ang_cam_flare_X, self.ang_cam_flare_Y)
        print("Cam Pos ", self.Z_cam_pos, self.X_cam_pos)
        print("Img Pos ", x, y)

        return (x / self.delta_X_pixy), (y / self.delta_Y_pixy)

    # process new data from pixy-cam
    def cam_process(self, raw_data):
        x, y = raw_data
        # print("Xmin: ", Xmin_image_cam)
        ang_ball = self.ang_ball_ratio * x
        # X_value = X gemessen + kleinstmöglicher X gemessen Wert + cam vor midpoint robot + bild des balls offset zum midpoint ball
        return (x * self.X_factor_cam) + self.X_offset_image_cam + self.s_cam_offset + (
                    self.r_ball * np.cos(ang_ball)), (y * self.Y_factor_cam)

    def motion_modell(self, p_old, p_new, dt):
        p_new_X, p_new_Y = p_new
        v_new_X = (p_new_X - p_old[0]) * dt
        v_new_Y = (p_new_Y - p_old[1]) * dt
        return p_new_X, p_new_Y, v_new_X, v_new_Y

    def run(self):
        'MAIN'
        P_ball_X, P_ball_Y, V_ball_X, V_ball_Y = self.motion_modell(self.P_ball_old, self.cam_process(self.read_raw()),
                                                                    self.dt)
        self.P_ball_old = self.P_ball_X, self.P_ball_Y
        # print("Pos in mm:", P_ball_X, P_ball_Y, "V in mm/s: ", V_ball_X, V_ball_Y)

        return P_ball_X, P_ball_Y, V_ball_X, V_ball_Y
'''

class Pwm(object):
    def __init__(self, freq, dutycycle, port):
        self.frq = freq
        self.duty = dutycycle
        self.port = port


class Wheel(Pwm):
    def __init__(self, freq, dutycycle, port, radius, v_min, v_max, v_now, voltage, n_now, p_x, p_y):
        self.PWM = Pwm(freq, dutycycle, port)
        self.r = radius
        self.V_min = v_min
        self.V_max = v_max
        self.V_now = v_now
        self.U = voltage
        self.pwm_factor = 1 / (self.V_max - self.V_min) * self.U
        self.n_now = n_now
        self.P_X = p_x
        self.P_Y = p_y


class Servo(Pwm):
    def __init__(self, freq, dutycycle, port, radius, ang_min, ang_max, ang_norm, ang_now, ang_tang, voltage, p_x, p_y,
                 ang_set):
        self.PWM = Pwm(freq, dutycycle, port)
        self.r = radius
        self.ang_min = ang_min
        self.ang_max = ang_max
        self.ang_norm = ang_norm
        self.ang_now = ang_now
        self.ang_tang = ang_tang
        self.U = voltage
        self.P_X = p_x
        self.P_Y = p_y
        self.ang_set = ang_set
        self.pwm_factor = 1 / (self.ang_max - self.ang_min) * self.U
