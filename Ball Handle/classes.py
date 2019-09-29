# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import time

import numpy as np

# GPIO.setup("P8_13", GPIO.OUT)

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


# from classes import Debug
class Debug(object):
    def __init__(self, name):  # , p_X, p_Y, v_X, v_Y, w_Z):
        self.Position_Error = []
        self.Vel_Error = []
        self.P_X = 0
        self.P_Y = 0
        self.phi_Z = 0
        self.V_X = 0
        self.V_Y = 0
        self.w_Z = 0
        self.name = name
        self.init_folder()
        self.fileName = self.init_file(self.name)
        self.Data = []

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

    def save(self):
        # --------------------
        # --- LOGGER-Part ----
        # --------------------
        print("Save File")
        npdata = np.array(self.Data)
        np.savetxt(self.fileName, npdata, delimiter=",")
        print(self.fileName)

    def draw(self):
        import matplotlib.pyplot as plt
        print("Drawing...")
        npdata = np.array(self.Data)
        # t_ref = npdata[1][0]
        t = npdata[:, 0]
        p_x = npdata[:, 1]

        # print("draw array", self.Data[1:], self.Data[:1])
        print("time", npdata[:, 0:1])
        print("P_X", np.array(npdata[:, 1:2]))
        plt.plot(npdata[1:, 0:1], np.array(npdata[1:, 1:2]), label='Position X')
        plt.plot(npdata[1:, 0:1], np.array(npdata[1:, 2:3]), label='Position Y')
        plt.plot(npdata[1:, 0:1], np.array(npdata[1:, 3:4]), label='Angle phi')
        plt.plot(npdata[1:, 0:1], np.array(npdata[1:, 4:5]), label='Velocity X')
        plt.plot(npdata[1:, 0:1], np.array(npdata[1:, 5:6]), label='Velocity Y')
        plt.plot(npdata[1:, 0:1], np.array(npdata[1:, 6:7]), label='Angular Vel omega')
        plt.legend()
        plt.show()

    def excecute(self, t_ref):
        # print("Pos in mm:", self.P_X, self.P_Y, "V in mm/s: ", self.V_X, self.V_Y)
        # pos = self.P_X, self.P_Y, self.phi_Z
        # vel = self.V_X, self.V_Y, self.w_Z
        # self.Position_Error.append(pos)
        # self.Vel_Error.append(vel)
        now = int(round(time.time() * 1000))  # datetime.strptime('%S.%f') #datetime.now(%s)

        self.Data.append([now - t_ref, self.P_X, self.P_Y, self.phi_Z, self.V_X, self.V_Y, self.w_Z])
        #print("Data: ", self.Data)


'''
class Pwm(object):
    def __init__(self, freq, start_duty, dutycycle, port):
        self.freq = freq
        self.duty = dutycycle
        self.port = port
        self.Servo_1 = PWM.start(port, start_duty, freq)
'''

'''
class Wheel(GPIO):
    def __init__(self, freq, start_duty, dutycycle, port, radius, v_min, v_max, v_now, voltage, n_now, p_x, p_y):
        self.freq = freq
        self.duty = dutycycle
        self.port = port
        # self.Wheel_1 = Pwm.start(port, start_duty, freq)
        self.r = radius
        self.V_min = v_min
        self.V_max = v_max
        self.V_now = v_now
        self.U = voltage
        self.pwm_factor = 1 / (self.V_max - self.V_min) * self.U
        self.n_now = n_now
        self.P_X = p_x
        self.P_Y = p_y
        self.pwmpin
        self.enablepin
        self.directionpin
        self.start(port, start_duty, freq)

    def motor_init(self):
        # GPIO initialisieren
        self.setmode(self.BCM)
        self.setup(self.pwmpin, self.OUT)
        self.setup(self.enablepin, self.OUT)
        self.setup(self.directionpin, self.OUT)

        # PWM-Frequenz auf 53.6 kHz setzen
        self.PWM(self.pwmpin, 53600)

        # PWM starten, Servo auf 0 Grad
        self.start(0)

    # Drehrichtung eingeben
    def setdir(self, dir):
        GPIO.output(self.directionpin, dir)

    # Umrechnung Grad in Tastverhaeltnis
    def setmotor(self, vel):
        if vel < 0:
            vel = 0
        elif vel > 4000:
            vel = 4000
        pwm = 0.025 * vel
        self.ChangeDutyCycle(pwm)

    try:
        GPIO.output(enablepin, 1)
        # Endlosschleife Servoansteuerung
        while True:
            dir = raw_input("Richtung eingeben (0,1): ")
            dir = int(dir)
            setdir(dir)
            vel = raw_input("Geschwindigkeit eingeben (0 - 4000): ")
            vel = float(vel)
            setmotor(vel)

    except KeyboardInterrupt:
        # Abbruch mit [Strg][C],
        # Servo auf 0 Grad, PWM beenden
        motor.stop()
        GPIO.cleanup()

class Servo(object):
    def __init__(self, freq, start_duty, dutycycle, port, radius, ang_min, ang_max, ang_norm, ang_now, ang_tang,
                 voltage, p_x, p_y,
                 ang_set):
        # self.PWM = Pwm(self, freq, dutycycle, port)
        self.duty = dutycycle
        SERVO_L = "P9_14"
        SERVO_R = "P9_16"
        pwm_min = 5
        pwm_max = 10
        freq = 50  # Hz
        start_duty = 0
        self.r = radius
        self.ang_min = ang_min
        self.ang_max = ang_max
        self.ang_norm = ang_norm
        self.ang_now = ang_now
        self.ang_tang = ang_tang
        self.U = voltage
        # self.P_X = p_x
        #self.P_Y = p_y
        self.ang_set = ang_set
        self.pwm_factor = 1 / (self.ang_max - self.ang_min) * self.U
        self.offset_servo_L = 0
        self.offset_servo_R = 0
        self.Servo_1 = Pwm.start(port, start_duty, freq)

    def servo_init(self):
        # Servo_1 = PWM.start(SERVO_L, start_duty, freq)
        # Servo_2 = PWM.start(SERVO_R, start_duty, freq)

        servo_L_ok = False
        servo_R_ok = False
        ready = input("Ready to init servo? (y|n)")
        if ready is 'y':
            while (servo_L_ok is not True or servo_R_ok is not True):

                self.servo_run(Servo_L_zero + Servo_L_ideal + offset_servo_L,
                               Servo_R_zero + Servo_R_ideal + offset_servo_R)
                check_L = input("Is Servo L correct oriented? (y|n)")
                if check_L is 'n':
                    new_L = int(input("Current orientation is 54°, set new orientation for left servo: "))
                    offset_servo_L = new_L - Servo_L_ideal
                    print("offset_L: ", offset_servo_L)
                elif check_L is 'y':
                    servo_L_ok = True
                check_R = input("Is Servo R correct oriented? (y|n)")
                if check_R is 'n':
                    new_R = int(input("Current orientation is 54°, set new orientation for right servo: "))
                    offset_servo_R = new_R - Servo_R_ideal
                    print("offset_R: ", offset_servo_R)
                elif check_R is 'y':
                    servo_R_ok = True

    def servo_run(self, ang_left, ang_right):  # der blanke winkel, um den sich der servo dreht

        def setServo_L(angle):
            if angle < 0:
                angle = 0
            elif angle > 180:
                angle = 180
            duty = angle / 36.0 + self.pwm_min
            if duty > self.pwm_max:
                duty = self.pwm_max
            # PWM.set_duty_cycle(SERVO_L, duty)
            print("duty L", duty)

        def setServo_R(angle):
            if angle < 0:
                angle = 0
            elif angle > 180:
                angle = 180
            duty = angle / 36.0 + pwm_min
            if duty > pwm_max:
                duty = pwm_max
            # PWM.set_duty_cycle(SERVO_R, duty)
            print("duty R", duty)

        setServo_L(ang_left + offset_servo_L)
        setServo_R(ang_right + offset_servo_R)

'''
if __name__ == '__main__':
    pass