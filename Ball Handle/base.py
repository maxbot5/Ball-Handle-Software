import time
import numpy as np
import Adafruit_BBIO.PWM as PWM
from PixyCam import PixyCam
from imu import Imu
from servo import Servo
from wheel import Wheel
from classes import State
import constants
import threading
#import logging
from queue import LifoQueue

count =0
'''
Objects
'''
# Sensors
imu = Imu('P5_4', sim_mode=False)

cam = PixyCam(sim_mode=False)

# Objects
ball_measure = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)
ball_set = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)

robot = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)

# Actuators
servo_left = Servo(sim_mode=False, radius=65, name='left', port="P9_14", ang_min=-10, ang_max=85,
                         ang_crit=SERVO_ANG_CRIT_LEFT,
                         ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_LEFT, pwm_min=5.4, pwm_max=9.5, start_duty=8,
                         ang_offset=SERVO_ANG_OFFSET_LEFT, p_x=194, p_y=-63.8)

servo_right = Servo(sim_mode=False, radius=65,name='right', port="P9_16", ang_min=-85, ang_max=10,
                          ang_crit=SERVO_ANG_OFFSET_RIGHT,
                          ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_RIGHT, pwm_min=6, pwm_max=9.5,
                          ang_offset=SERVO_ANG_OFFSET_RIGHT, p_x=194, p_y=63.8)

wheel_left = Wheel(pin_en=PIN_EN_WHEEL_LEFT, pin_dir=PIN_DIR_WHEEL_LEFT, pin_pwm=PIN_PWM_WHEEL_LEFT)
wheel_right = Wheel(pin_en=PIN_EN_WHEEL_RIGHT, pin_dir=PIN_DIR_WHEEL_RIGHT, pin_pwm=PIN_PWM_WHEEL_RIGHT)

# estimate real ball motion in relation to the ground
def observer():
    pass
#decide how to handle the ball
def controller():
    global ball_status_new
    global ball_status_old
    def accept_ball():
        #1. set servos to the position that the ball hit the ball handle in front
        #2. set wheels to spin the ball in the mirror ang of the incoming direction
        pass
    def dribbel_ball():
        #1. fix the servos to ideal ball handle position
        servo_left.ang_set = servo_left.dribbel_ang
        servo_right.ang_set = servo_right.ang_set
        #2. balance the ball in front of the ball handle
        return
    if ball_status_new is FAR_BALL:
        return
    if ball_status_new is NEAR_BALL:
        accept_ball()
    if ball_status_new is HAVE_BALL:
        dribbel_ball()
    pass


class Input(threading.thread):
    def __init__(self):
        pass
    def run(self):
        while(True):
            if not imu.Queue.full() : #and int-pin imu auslesen
                #imu.process()  Muss noch erstellt werden!! queue in die imu class einfügen und direkt befüllen im process
                #ImuQueue.put()
            if not cam.Queue.full():
                cam.Queue.put(cam.process())

class Processing(threading.thread):
    def __init__(self):
        pass
    def run(self):
        while(True):
            #read sensordata
            if not imu.Queue.empty() : #and int-pin imu auslesen
                robot.V_X, robot.V_Y, robot.w_Z = imu.Queue.get()
            if not cam.Queue.empty():
                ball_measure.V_X, ball_measure.V_Y = cam.Queue.get()

            servo_left.ang_set, servo_right.ang_set, wheel_left.V_set, wheel_right.V_set = controller(observer(ball_measure.V_X,ball_measure.V_Y,robot.V_X, robot.V_Y))

            # set actuatordata
            if not servo_left.Queue.full():
                servo_left.Queue.put(servo_left.ang_set)
            if not servo_right.Queue.full():
                servo_right.Queue.put(servo_right.ang_set)
            if not wheel_left.Queue.full():
                wheel_left.Queue.put(wheel_left.V_set)
            if not wheel_right.Queue.full():
                wheel_right.Queue.put(wheel_right.V_set)


class Output(threading.thread):
    def __init__(self):
        pass
    def run(self):
        while(True):
            if not servo_left.Queue.empty() :
                servo_left.process(servo_left.Queue.get())
            if not servo_right.Queue.empty():
                servo_left.process(servo_left.Queue.get())
            if not wheel_left.Queue.empty():
                wheel_left.process(wheel_left.Queue.get())
            if not wheel_right.Queue.empty():
                wheel_right.process(wheel_right.Queue.get())


