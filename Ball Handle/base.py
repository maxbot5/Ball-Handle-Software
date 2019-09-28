from PixyCam import PixyCam
import imu
import time
import numpy as np
import servo
import wheel
from classes import State
import Adafruit_BBIO.PWM as PWM
import threading
#import logging
from queue import LifoQueue

'''
Constants
'''
# parameter for visualization
ON = 1
OFF = 0
SHOW_WITH_VEL = ON
VISUSUALIZATION = ON

# ball status
# parameter for interception
r_ball = 114.8  # mm
origin = (0, 0)
ideal_line = 375
#ball_distance_max = 200
impact_point = (375, 0)

WAIT_FOR_SERVO = 0.2 #sek = 100 ms

# wheel parameter
R_WHEEL = 40
SLIP_IDEAL = 0.2
SLIP_CRITICAL = 0.4
SLIP_THRESHOLD = 100  # up this speed the sliping bound will occure
I_M2W = 26 / 7
F_MOTOR = 53600
VOLTAGE_WHEEL = 3.3
PIN_PWM_WHEEL_LEFT = "P8_13"
PIN_DIR_WHEEL_LEFT = "P8_11"
PIN_EN_WHEEL_LEFT = "P8_9"
PIN_PWM_WHEEL_RIGHT = "P8_19"
PIN_DIR_WHEEL_RIGHT = "P8_17"
PIN_EN_WHEEL_RIGHT = "P8_15"
V_left = 0
V_right = 0
# direction
CCW = 0
CW = 1
# robot
V_ROB_MAX = 2000
V_ROB_MIN = -2000

# Servo constants
SERVO_ANG_OFFSET_LEFT =  20  # np.deg2rad(-40) #60
SERVO_ANG_OFFSET_RIGHT =  -30  # np.deg2rad(40) #35
SERVO_PORT_LEFT = "P9_14"
SERVO_PORT_RIGHT = "P9_16"
SERVO_FREQ = 50
SERVO_RADIUS = 55
SERVO_VOLTAGE = 5  # Volt
SERVO_ANG_CRIT_LEFT = 45  # np.deg2rad(-85)
SERVO_ANG_CRIT_RIGHT = 45  # np.deg2rad(85)
SERVO_ANG_START = 60
SERVO_THRESHHOLD = 50

SERVO_ANG_DRIBBEL_LEFT = -54
SERVO_ANG_DRIBBEL_RIGHT = 54

count =0
'''
Objects
'''
# Sensors
imu = imu.Imu('P5_4', sim_mode=False)

cam = PixyCam(sim_mode=False)

# Objects
ball_measure = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)
ball_set = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)

robot = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)

# Actuators
servo_left = servo.Servo(sim_mode=False, radius=65, name='left', port="P9_14", ang_min=-10, ang_max=85,
                         ang_crit=SERVO_ANG_CRIT_LEFT,
                         ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_LEFT, pwm_min=5.4, pwm_max=9.5, start_duty=8,
                         ang_offset=SERVO_ANG_OFFSET_LEFT, p_x=194, p_y=-63.8)
'''
WERTE nochmal amn 3D-Modell nachmessen!!!!!!
'''
servo_right = servo.Servo(sim_mode=False, radius=65,name='right', port="P9_16", ang_min=-85, ang_max=10,
                          ang_crit=SERVO_ANG_OFFSET_RIGHT,
                          ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_RIGHT, pwm_min=6, pwm_max=9.5,
                          ang_offset=SERVO_ANG_OFFSET_RIGHT, p_x=194, p_y=63.8)

wheel_left = wheel.Wheel(pin_en=PIN_EN_WHEEL_LEFT, pin_dir=PIN_DIR_WHEEL_LEFT, pin_pwm=PIN_PWM_WHEEL_LEFT)
wheel_right = wheel.Wheel(pin_en=PIN_EN_WHEEL_RIGHT, pin_dir=PIN_DIR_WHEEL_RIGHT, pin_pwm=PIN_PWM_WHEEL_RIGHT)

'''
Functions Helping
'''

