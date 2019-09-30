import time
import numpy as np
#import Adafruit_BBIO.PWM as PWM
#from PixyCam import PixyCam
#from imu import Imu
#from servo import Servo
#from wheel import Wheel
from classes import State
import constants as cons
import threading
#import logging
import sys
from queue import LifoQueue

count =0
ball_status_new = 0
ball_status_old = 0

stop_threads = False

BUF_SIZE = 1
imuQueue = LifoQueue(BUF_SIZE)
camQueue = LifoQueue(BUF_SIZE)
wheelQueue = LifoQueue(BUF_SIZE)
servoQueue = LifoQueue(BUF_SIZE)
'''
wheel_leftQueue = LifoQueue(BUF_SIZE)
wheel_rightQueue = LifoQueue(BUF_SIZE)
servo_leftQueue = LifoQueue(BUF_SIZE)
servo_rightQueue = LifoQueue(BUF_SIZE)
'''

class Input(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # create Sensor objects
        #self.imu = Imu('P5_4', sim_mode=False)
        #self.cam = PixyCam(sim_mode=False)

    def run(self):
        global dead

        print("Start Input...")
        while(not killpill):
            if not imuQueue.full() : #and int-pin imu auslesen
                #imu.process()  Muss noch erstellt werden!! queue in die imu class einfügen und direkt befüllen im process
                imuQueue.put((1000, 0, 0)) #TEST DATA
            if not camQueue.full():
                #camQueue.put(camQueue.process())
                camQueue.put((2,400,0,100,0))  #TEST DATA

class Processing(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # Objects
        self.ball_measure = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)
        self.ball_set = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)
        self.robot = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)
        self.ang_set_left = 0
        self.ang_set_right = 0
        self.V_set_left = 0
        self.V_set_right = 0
        self.impact_point = (0,0)
        self.ang2impact_left = 0
        self.ang2impact_right = 0

    def cart2pol(self, beg, end):  # transform cartesian coordiantes to polar
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

    # estimate real ball motion and important information about it in relation to the ground
    def observer(self):
        print("Start observer...")
        #1. calculating impact point and the dribbel position
        def impact_Y():  # cutting point between ball motion and y-axle through ideal dribbel point
            # return -(((ball_measure.P_X - impact_point[0]) * -(ball_measure.V_Y / ball_measure.V_X)) + ball_measure.P_Y)
            if self.ball_measure.V_Y == 0 or self.ball_measure.V_X == 0:
                return -(self.ball_measure.P_Y)
            else:
                return -(-(self.ball_measure.P_X - self.impact_point[0]) * self.ball_measure.V_Y / self.ball_measure.V_X + self.ball_measure.P_Y)

        def tangent_point(a, M_X, M_Y, P_X, P_Y):  # calculate the wheel position on servos motion circle
            # a = servo-radius
            # b = halber Abstand d zwischen Ballmittelpunkt und Servomittelpunkt

            c, tangent_ang = self.cart2pol((M_X, M_Y), (P_X, P_Y))
            b = (c * 0.5)
            # print("a,b,c, ang: ", a, b, c, np.rad2deg(tangent_ang))

            x = np.sqrt(((c * c) + (a * a) - (b * b)) / (2 * c))
            y = (np.sqrt(np.absolute((a * a) - (x * x))))

            dx, dy = P_X - M_X, P_Y - M_Y

            Wheel_X1 = M_X + x * (dx / c) - y * (dy / c)
            Wheel_Y1 = M_Y + x * (dy / c) + y * (dx / c)
            Wheel_X2 = M_X + x * (dx / c) + y * (dy / c)
            Wheel_Y2 = M_Y + x * (dy / c) - y * (dx / c)

            # print("Schnittpunkte x1, y1, x2, y2", S_X1, S_Y1, S_X2, S_Y2)
            # der ball darf nicht weiter als der servodrehpunkt kommen, also ist M_Y das seitliche Maximum
            # der ball kann niemals hinter dem servo sein, also ist P_X der äußerste Punkt auf dieser achse
            # print("tangent_ang:",tangent_ang)

            left_abs, ang_set_left= self.cart2pol((M_X,M_Y), (Wheel_X1,Wheel_Y1))
            left_right, ang_set_right = self.cart2pol((M_X,M_Y), (Wheel_X2,Wheel_Y2))

            return ang_set_left, ang_set_right

        self.impact_point = (cons.DRIBBEL_POINT_X, impact_Y())
        p_x_left, p_y_left = cons.SERVO_POS_LEFT
        p_x_right, p_y_right = cons.SERVO_POS_RIGHT
        imp_x, imp_y = self.impact_point
        self.ang2impact_left, dump = tangent_point(cons.SERVO_RADIUS, p_x_left, p_y_left, imp_x, imp_y)
        dump, self.ang2impact_right = tangent_point(cons.SERVO_RADIUS, p_x_right, p_y_right, imp_x, imp_y)
        print("Observer: impact_point", self.impact_point, "tangent_angle", self.ang2impact_left, self.ang2impact_right)

        #2. Ball motion in relation to the ground based on relativ motion to the robot and robots own motion
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
            #current ball motion in relation to the ground, because we can only influence the motion to the robot directly.
            #Thats why we need the robot motion
            vy = self.robot.V_Y
            vx = self.robot.V_X
            if abs(self.ball_measure.V_X) > 50:
                vx = self.robot.V_X - self.ball_measure.V_X
            if abs(self.ball_measure.V_Y) > 50:
                vy = self.robot.V_Y - self.ball_measure.V_Y

            return vx, vy

        self.ball_set.V_X, self.ball_set.V_Y = setPoint_ball()
        print("Observer: self.ball_set.V_X, self.ball_set.V_Y", self.ball_set.V_X, self.ball_set.V_Y)
    # decide how to handle the ball
    def controller(self):
        global ball_status_new
        global ball_status_old
        print("Start controller...")

        def wheel_velocity(ball_mag, ball_ang):

            print("controller: wheel velocity: ball_mag, ball_ang",  ball_mag, np.rad2deg(ball_ang))
            v_left = -(ball_mag * (
                        np.cos(-self.ang_set_left + ball_ang) + np.sin(self.ang_set_left + ball_ang)))
            v_right = (ball_mag * (
                        np.cos(-self.ang_set_right + ball_ang) + np.sin(-self.ang_set_right + ball_ang)))
            # print("wheel velocity x|Y", v_left, v_right)
            # print("Ball |V|:",ball_mag, "Ball Ang:", ball_ang)
            return v_left, v_right

        def accept_ball():
            print("accept_ball")
            # 1. set servos to the position that the ball hit the ball-handle in front
            self.ang_set_left = np.rad2deg(self.ang2impact_left)
            self.ang_set_right = np.rad2deg(self.ang2impact_right)
            # 2. set the wheels to spin in robots motion
            V_Ball_mag, ang = self.cart2pol((self.ball_set.V_X, self.ball_set.V_Y), self.impact_point)
            mag, Ball_ang = self.cart2pol((self.ball_set.P_X, self.ball_set.P_Y), self.impact_point)
            self.V_set_left, self.V_set_right = wheel_velocity(V_Ball_mag, Ball_ang)
        def dribbel_ball():
            print("dribbel_ball")
            # 1. fix the servos to ideal ball handle position
            self.ang_set_left = cons.ANG_DRIBBEL_LEFT
            self.ang_set_right = cons.ANG_DRIBBEL_RIGHT
            # 2. balance the ball in front of the ball handle by setting wheels spin to the mirror ang of the current ball motion
            self.V_set_left, self.V_set_right = wheel_velocity(
                self.cart2pol((self.ball_set.V_X, self.ball_set.V_Y), self.impact_point))

        if ball_status_new is cons.FAR_BALL:
            ball_status_old = cons.FAR_BALL
            return
        if ball_status_new is cons.NEAR_BALL:
            accept_ball()
            ball_status_old = cons.NEAR_BALL
            return
        if ball_status_new is cons.HAVE_BALL:
            dribbel_ball()
            ball_status_old = cons.HAVE_BALL
            return

    def run(self):
        global dead, ball_status_new
        print("Start Processing...")
        while(not killpill):
            #read sensordata
            if not imuQueue.empty() : #and int-pin imu auslesen
                self.robot.V_X, self.robot.V_Y, self.robot.w_Z = imuQueue.get()
                print("PROCESS: self.robot.V_X, self.robot.V_Y, self.robot.w_Z",self.robot.V_X, self.robot.V_Y, self.robot.w_Z)
            if not camQueue.empty():
                ball_status_new, self.ball_measure.P_X, self.ball_measure.P_Y, self.ball_measure.V_X, self.ball_measure.V_Y = camQueue.get()
                print("PROCESS: self.ball_measure.V_X, self.ball_measure.V_Y", self.ball_measure.V_X, self.ball_measure.V_Y)

            #Execution:
            if not wheelQueue.full():
                self.observer()
                self.controller()
                print("PROCESS: (self.ang_set_left, self.ang_set_right, self.V_set_left, self.V_set_right)", (self.ang_set_left, self.ang_set_right, self.V_set_left, self.V_set_right))
                wheelQueue.put((self.V_set_left, self.V_set_right))
                if not servoQueue.full():
                    servoQueue.put((self.ang_set_left, self.ang_set_right))



class Servos(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        '''
        self.servo_left = Servo(sim_mode=False, radius=65, name='left', port="P9_14", ang_min=-10, ang_max=85,
                                ang_crit=SERVO_ANG_CRIT_LEFT,
                                ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_LEFT, pwm_min=5.4, pwm_max=9.5,
                                start_duty=8,
                                ang_offset=SERVO_ANG_OFFSET_LEFT, p_x=194, p_y=-63.8)

        self.servo_right = Servo(sim_mode=False, radius=65, name='right', port="P9_16", ang_min=-85, ang_max=10,
                                 ang_crit=SERVO_ANG_OFFSET_RIGHT,
                                 ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_RIGHT, pwm_min=6, pwm_max=9.5,
                                 ang_offset=SERVO_ANG_OFFSET_RIGHT, p_x=194, p_y=63.8)
        '''
    def run(self):
        while(not killpill):
            if not servoQueue.empty():
                # self.servo_left.ang_set, self.servo_right.ang_set, self.wheel_left.V_set, self.wheel_right.V_set = outputQueue.get()
                servo_left_ang_set, servo_right_ang_set = servoQueue.get()
                print("servo_left_ang_set, servo_right_ang_set", servo_left_ang_set, servo_right_ang_set)
                # self.servo_left.process(self.servo_left.ang_set)
                # self.servo_right.process(self.servo_right.ang_set)
                time.sleep(0.2)


class Wheels(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        '''
        # Actuators
        self.wheel_left = Wheel(pin_en=PIN_EN_WHEEL_LEFT, pin_dir=PIN_DIR_WHEEL_LEFT, pin_pwm=PIN_PWM_WHEEL_LEFT)
        self.wheel_right = Wheel(pin_en=PIN_EN_WHEEL_RIGHT, pin_dir=PIN_DIR_WHEEL_RIGHT, pin_pwm=PIN_PWM_WHEEL_RIGHT)
        '''

    def run(self):
        global dead
        print("Start Output...")
        while(not killpill):
            if not wheelQueue.empty():
                #self.servo_left.ang_set, self.servo_right.ang_set, self.wheel_left.V_set, self.wheel_right.V_set = outputQueue.get()
                wheel_left_V_set, wheel_right_V_set = wheelQueue.get()
                print("wheel_left_V_set, wheel_right_V_set", wheel_left_V_set, wheel_right_V_set)

                #self.wheel_left.process(self.wheel_left.V_set)
                #self.wheel_right.process(self.wheel_right.V_set)






def start_threads():
    threadlist = []
    inputThread = Input()
    threadlist.append(inputThread)
    inputThread.start()

    processingThread = Processing()
    threadlist.append(processingThread)
    processingThread.start()

    servoThread = Servos()
    threadlist.append(servoThread)
    servoThread.start()

    wheelThread = Wheels()
    threadlist.append(wheelThread)
    wheelThread.start()

    inputThread.join()
    processingThread.join()
    servoThread.join()
    wheelThread.join()

def start_thread_2():
    #run_event = threading.Event()
    #run_event.set()



    inputThread.join()
    processingThread.join()
    servoThread.join()
    wheelThread.join()


if __name__ == '__main__':
    try:
        killpill = False
        #start_thread_2()
        inputThread = Input()
        inputThread.daemon = True
        inputThread.start()

        processingThread = Processing()
        processingThread.daemon = True
        processingThread.start()

        servoThread = Servos()
        servoThread.daemon = True
        servoThread.start()

        wheelThread = Wheels()
        wheelThread.daemon = True
        wheelThread.start()
        input("killpill activ with enter: ")
        killpill = True
        inputThread.join()
        processingThread.join()
        servoThread.join()
        wheelThread.join()
        #stop_threads()
    except KeyboardInterrupt:
        exit(0)


