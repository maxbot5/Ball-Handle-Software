from PixyCam import PixyCam
import imu
import time
import numpy as np
import servo
import wheel
from classes import State
import Adafruit_BBIO.PWM as PWM

'''
Constants
'''
# parameter for visualization
ON = 1
OFF = 0
SHOW_WITH_VEL = ON
VISUSUALIZATION = ON

# TO DO: evtl eigene Klassen fuer Ball und Roboter???
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


def impact_Y():  # cutting point between ball motion and y-axle through ideal dribbel point
    #return -(((ball_measure.P_X - impact_point[0]) * -(ball_measure.V_Y / ball_measure.V_X)) + ball_measure.P_Y)
    return -(-(ball_measure.P_X - impact_point[0]) * ball_measure.V_Y / ball_measure.V_X + ball_measure.P_Y)
def cart2pol(beg, end):  # transform cartesian coordiantes to polar
    rel = (end[0] - beg[0], end[1] - beg[1])
    mag = np.hypot(rel[0], rel[1])
    # print("mag= ", mag, beg, end)
    # print("rel= ", rel)
    # proof of cases
    if rel[0] < 0:
        if rel[1] < 0:
            return mag, np.arctan(rel[1] * (1 / rel[0]) - np.pi)
        elif rel[1] >= 0:
            return mag, np.arctan(rel[1] * (1 / rel[0]) + np.pi)
    elif rel[0] == 0:
        if rel[1] < 0:
            return mag, -(np.pi * 0.5)
        elif rel[1] > 0:
            return mag, (np.pi * 0.5)
    elif rel[0] > 0:
        return mag, np.arctan(rel[1] * (1 / rel[0]))


def setPoint_ball():  # setpoint for ball movement (include model from robot motion and relativ ball motion)
    '''
    Berechnung des Geschwindigkeitsbetrags des Balls in kartesischen Koordinaten V(X,Y)
    Zur Zeit befindet sich der ideale Dribbelpunkt vor dem Roboter in X-Richrung ohne Abweichung in Y-Richtung
    Überlegung: stattdessen aktuellen Dribbelpunkt verwenden!

    (akutelle Umsetzung) Um den Ball im idealen Dribbelpunkt zu dribbeln, werden
    die folgenden Ballgeschwindigkeiten benötigt:
    '''
    # Hinweis: eigentlich -Vball am Ende
    '''
    V_X = robot.V_X + np.cos(np.deg2rad(robot.w_Z)) * np.sqrt(
        ball_measure.P_X * ball_measure.P_X + ball_measure.P_Y * ball_measure.P_Y) - ball_measure.V_X
    V_Y = robot.V_Y + np.sin(np.deg2rad(robot.w_Z)) * np.sqrt(
        ball_measure.P_X * ball_measure.P_X + ball_measure.P_Y * ball_measure.P_Y) - ball_measure.V_Y
    '''
    vy = robot.V_X
    vx = robot.V_Y
    if ball_measure.V_X > 50:
        vx = robot.V_X - ball_measure.V_X
    if ball_measure.V_Y > 100 or ball_measure.V_Y < -100:
        vy = robot.V_Y - ball_measure.V_Y

    return vx, vy


def tangent_point(a, M_X, M_Y, P_X, P_Y):  # calculate the wheel position on servos motion circle
    # a = servo-radius
    # b = halber Abstand d zwischen Ballmittelpunkt und Servomittelpunkt

    c, tangent_ang = cart2pol((M_X, M_Y), (P_X, P_Y))
    b = (c * 0.5)
    #print("a,b,c, ang: ", a, b, c, np.rad2deg(tangent_ang))

    x = np.sqrt(((c * c) + (a * a) - (b * b)) / (2 * c))
    y = (np.sqrt(np.absolute((a * a) - (x * x))))

    dx, dy = P_X - M_X, P_Y - M_Y

    S_X1 = M_X + x * (dx / c) - y * (dy / c)
    S_Y1 = M_Y + x * (dy / c) + y * (dx / c)
    S_X2 = M_X + x * (dx / c) + y * (dy / c)
    S_Y2 = M_Y + x * (dy / c) - y * (dx / c)

    #print("Schnittpunkte x1, y1, x2, y2", S_X1, S_Y1, S_X2, S_Y2)

    # der ball darf nicht weiter als der servodrehpunkt kommen, also ist M_Y das seitliche Maximum
    # der ball kann niemals hinter dem servo sein, also ist P_X der äußerste Punkt auf dieser achse
    #print("tangent_ang:",tangent_ang)
    return tangent_ang, S_X1, S_Y1, S_X2, S_Y2


def wheel_velocity(datain):
    ball_mag, ball_ang = datain
    v_left = -(ball_mag * (np.cos(-servo_left.ang_dribbel + ball_ang) + np.sin(servo_left.ang_dribbel + ball_ang)))
    v_right = (ball_mag * (np.cos(-servo_right.ang_dribbel + ball_ang) + np.sin(-servo_right.ang_dribbel + ball_ang)))
    #print("wheel velocity x|Y", v_left, v_right)
    #print("Ball |V|:",ball_mag, "Ball Ang:", ball_ang)
    return v_left, v_right


'''
Functions  Main
'''
def update_sensors():
    cam.run()
    #ball_measure.P_X, ball_measure.P_Y, ball_measure.V_X, ball_measure.V_Y = np.round(cam.P_ball_X,0),np.round(cam.P_ball_Y,0), np.round(cam.V_ball_X,0), np.round(cam.V_ball_Y,0)
    ball_measure.P_X, ball_measure.P_Y, ball_measure.V_X, ball_measure.V_Y = cam.P_ball_X, cam.P_ball_Y, cam.V_ball_X, cam.V_ball_Y
    robot.V_X, robot.V_Y, robot.w_Z = imu.run()
    #print("round:",ball_measure.P_X, ball_measure.P_Y, ball_measure.V_X, ball_measure.V_Y)
    #robot.V_X, robot.V_Y, robot.w_Z = 500, 0, 0
    robot.q_dot = robot.V_X, robot.V_Y, robot.w_Z


def execute_actuators():
    servo_left.run()
    #time.sleep(WAIT_FOR_SERVO*2)
    servo_right.run()
    #time.sleep(WAIT_FOR_SERVO*2)
    wheel_left.run()
    wheel_right.run()
    return

def observer():
    global impact_point
    '''
    INPUT
    '''
    #print("get input...")
    #update_sensors()
    '''
    CALCULATION
    '''
    #print("IMPACT point:", impact_point)
    #Servo Action#
    '''
    if ball_measure.V_X > SERVO_THRESHHOLD:  # and ball_measure.P_X > 400:

        impact_point = impact_point[0], impact_Y()
        print("IMPACT at:",impact_point)
        if impact_point[1] > 30 or impact_point[1] < -30:
            ball_set.q_dot = setPoint_ball()

            servo_left.ang_tang, wheel_left.P_X, wheel_left.P_Y, x3, y3 = tangent_point(servo_left.r, servo_left.P_X,
                                                                                            servo_left.P_Y,
                                                                                            impact_point[0], impact_point[1])
            servo_right.ang_tang, x2, y2, wheel_right.P_X, wheel_right.P_Y = tangent_point(servo_right.r, servo_right.P_X,
                                                                                               servo_right.P_Y,
                                                                                               impact_point[0], impact_point[1])
                # set servo
            servo_left.ang_set = np.rad2deg(servo_left.ang_tang)  #-90
            servo_right.ang_set = np.rad2deg(servo_right.ang_tang)  #+90
            print("Ang_set Left:", servo_left.ang_set)
            print("Ang_set Right", servo_right.ang_set)

    '''
    # set wheel
    ball_set.q_dot = setPoint_ball()

    wheel_left.V_set, wheel_right.V_set = wheel_velocity(cart2pol(ball_set.q_dot, impact_point))
    #else:
    servo_left.ang_set = servo_left.ang_dribbel
    servo_right.ang_set = servo_right.ang_dribbel

    global count
    count = count+1
    if count >= 100 :
        print("V_Ball  messen X|Y",ball_measure.V_X,ball_measure.V_Y)
        print("V_Ball soll X|Y", ball_set.V_X, ball_set.V_Y)
        print("V_Robot X|Y, w", robot.V_X, robot.V_Y, robot.w_Z)
        print("WHEEL LEFT ", wheel_left.V_set, "WHEEL RIGHT", wheel_right.V_set)
        print("Wheel LEFT rpm", wheel_left.n_new, "Wheel RIGHT rpm", wheel_right.n_new)
        count = 0

    execute_actuators()

#for count in  range(0,100):
def init_observer():
    print("init observer...")
    servo_left.ang_set = -36
    servo_right.ang_set = 36
    servo_left.run()
    servo_right.run()
    global impact_point
    offset = cam.offset_X,cam.offset_Y
    impact_point = cam.cam_process(cam.transform_coordinates(offset))
    #print("impact point", impact_point)
    #print("Init complete...")
    #time.sleep(2)

#init_observer()
#impact_point = (230,0)

while True:
#for i in range(0,500):
    try:
        update_sensors()
        #print("ball vel:",ball_measure.V_X,ball_measure.V_Y)
        #if ball_measure.V_X and ball_measure.V_Y is not 0:
        observer()
            #print("wheels Velocity ", wheel_left.V_set, wheel_right.V_set)
        #print("servos angle ", servo_left.ang_set, servo_right.ang_set)
    except KeyboardInterrupt:
        PWM.stop(SERVO_PORT_LEFT)
        PWM.stop(SERVO_PORT_RIGHT)
        PWM.stop(PIN_PWM_WHEEL_LEFT)
        PWM.stop(PIN_PWM_WHEEL_RIGHT)
        PWM.cleanup()
        time.sleep(2)
        break
