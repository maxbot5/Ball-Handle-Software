# -*- coding: utf-8 -*-
import time
import numpy as np
from classes import Debug, KalmanFilter
import smbus

bus = smbus.SMBus(2)  # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68  # via i2cdetect
power_mgmt_1 = 0x6b
ACCEL_CONFIG = 0x1C  # Reg 28
ACCEL_CONFIG2 = 0x1D  # Reg 29

class Imu(Debug, KalmanFilter):
    def __init__(self, sim_mode=False):
        self.debug = Debug('imu')
        self.sim_mode = sim_mode
        self.kf = self.filter_config()
        self.raw = self.read_raw()
        self.offset = self.offset_calc()
        #self.port = port

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
        bus.write_byte_data(address, power_mgmt_1, 0)  # full power mode
        # bus.write_byte_data(address, power_mgmt_2, 0b00001111)  #disabele=1, disabled accel_z, gyro_x bis _z
        # setzt Accelerometer Full Scale Select (hier auf +-2g)
        bus.write_byte_data(address, ACCEL_CONFIG, 0b00100000)
        # setzt den Tiefpass-Filter
        bus.write_byte_data(address, ACCEL_CONFIG2,
                                 0b00000100)  # entspricht dem Wert 4, also 19,8 ms ~50Hz
        #print("IMU config ready..")

    def read_word(self, reg):
        h = bus.read_byte_data(address, reg)
        l = bus.read_byte_data(address, reg + 1)
        # h = bus.read_byte_data(self.address, reg)
        # l = bus.read_byte_data(self.address, reg + 1)
        value = (h << 8) + l
        return value

    def read_word_2c(self, reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def read_raw(self):

        if self.sim_mode == True:
            return 100, 200, 20
        else:
            beschleunigung_xout = self.read_word_2c(0x3b)
            beschleunigung_yout = self.read_word_2c(0x3d)
            gyroskop_zout = self.read_word_2c(0x47)

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
        print("finished calc..")
        #print("offset:",offset)
        return np.median(offset, axis=0)

    def kalman_filter(self, z):
        # das ist meine C matrix für den Ausgang, also müsste das mittlere die geschwindigkeit sein
        np.dot(self.kf.H, self.kf.predict())
        self.kf.update(z)
        #print("kalmanfilter: ", self.kf.x[0], self.kf.x[1], self.kf.x[2])
        return self.kf.x[1]

    def process(self):
        return self.kalman_filter(self.read_raw() - self.offset)

'''
def test_imu(save=False, draw=False):
    print("stat testing...")
    imu = Imu(sim_mode=False)
    t_ref = int(round(time.time() * 1000))

    if imu.sim_mode:
        for i in range(0, 1000):
            try:
                imu.debug.V_X, imu.debug.V_Y, imu.debug.w_Z = imu.run()
                imu.debug.excecute(t_ref)
                time.sleep(0.1)
            except KeyboardInterrupt:
                break
    else:
        while KeyboardInterrupt is not True:
            try:
                imu.debug.V_X, imu.debug.V_Y, imu.debug.w_Z = imu.run()
                imu.debug.excecute(t_ref)
            except KeyboardInterrupt:
                break
    if save:
        imu.debug.save()
    if draw:
        imu.debug.draw()
    return


# if __name__== "__main":
test_imu(save=True)
'''
