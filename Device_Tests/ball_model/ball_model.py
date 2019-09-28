import matplotlib.pyplot as plt
import numpy as np
from classes_ballmodell import *
# parameter for visualization
ON = 1
OFF = 0
SHOW_WITH_VEL = ON
VISUSUALIZATION = ON

# TO DO: evtl eigene Klassen fuer Ball und Roboter???
# ball status
ball_status = 0
FAR_BALL = 1
NEAR_BALL = 2
HAVE_BALL = 3
CONQUER_BALL = 4

r_ball = 114.8  # mm
origin = (0, 0)
ideal_line = 200  # distance of ball midpoint from robot for dribbling
ball_distance_max = 210  # border for making decision if have oder not
impact_point = (200, 0)  # (X|Y)
gear_bw = 0



'''
INIT
'''

# test data set DIE ECHTEN WERTE werden entweder als Varible am Anfang initialisiert oder per Funktion im Verlauf
ball_measure = State(p_x=210, p_y=-0, phi_z=0, v_x=-100, v_y=0, w_z=0)
ball_set = State(p_x=0, p_y=0, phi_z=0, v_x=0, v_y=0, w_z=0)

robot = State(p_x=0, p_y=0, phi_z=0, v_x=1000, v_y=0, w_z=0)

wheel_left = Wheel(freq=0, dutycycle=0, port="P4", radius=40, v_min=0, v_max=5000, v_now=0, voltage=3.3, n_now=None,
                   p_x=10, p_y=10)
wheel_right = Wheel(freq=0, dutycycle=0, port="P5", radius=40, v_min=0, v_max=5000, v_now=0, voltage=3.3, n_now=None,
                    p_x=0, p_y=0)

servo_left = Servo(freq=0, dutycycle=0, port="P6", radius=50, ang_min=np.deg2rad(-85), ang_max=0, ang_norm=54,
                   ang_now=0, ang_tang=0, voltage=5,
                   p_x=100, p_y=-100, ang_set=0)
servo_right = Servo(freq=0, dutycycle=0, port="P7", radius=50, ang_min=0, ang_max=np.deg2rad(85), ang_norm=90 - 54,
                    ang_tang=0, ang_now=0,
                    voltage=5, p_x=100, p_y=100, ang_set=0)

# imu = Imu()
# = PixyCam()
'''
FUNCTIONS
'''


def update_sensors():
    ball_measure.P_X, ball_measure.P_Y, ball_measure.V_X, ball_measure.V_Y = cam.run()
    # cam.debug(cam.run())
    robot.P_X, robot.P_Y, robot.V_X, robot.V_Y, robot.w_Z = imu.run()


def update_actuators():
    pass
# das ist eigentlich eine Funktion für das Objekt Ball
'''
The measured ball position and velocity is relativ to the robot. That means the robot velocity is 
included in this measured value.  
'''


def impact_Y():  # cutting point between ball motion and y-axle through ideal dribbel point
    return (((ball_measure.P_X - impact_point[0]) * -(ball_measure.V_Y / ball_measure.V_X)) + ball_measure.P_Y)



# das ist eigentlich eine Funktion für das Objekt Servo
def tangent_point(a, M_X, M_Y, P_X, P_Y):  # calculate the wheel position on servos motion circle
    # a = servo-radius
    # b = halber Abstand d zwischen Ballmittelpunkt und Servomittelpunkt
    #global c
    c, tangent_ang = cart2pol((M_X, M_Y), (P_X, P_Y))
    b = (c * 0.5)
    print("a,b,c, ang: ", a, b, c, np.rad2deg(tangent_ang))
    a_old = 0
    '''
    if a > b:
        a = a_old
        a = b
        b = a_old
        c = 2*b
    '''
    x = np.sqrt(((c * c) + (a * a) - (b * b)) / (2 * c))
    #x = ((c * c) + (a * a) - (b * b)) / (2 * c)
    y = (np.sqrt(np.absolute((a * a) - (x * x))))

    dx, dy = P_X - M_X, P_Y - M_Y

    S_X1 = M_X + x * (dx / c) - y * (dy / c)
    S_Y1 = M_Y + x * (dy / c) + y * (dx / c)
    S_X2 = M_X + x * (dx / c) + y * (dy / c)
    S_Y2 = M_Y + x * (dy / c) - y * (dx / c)

    print("Schnittpunkte x1, y1, x2, y2", S_X1, S_Y1, S_X2, S_Y2)

    # der ball darf nicht weiter als der servodrehpunkt kommen, also ist M_Y das seitliche Maximum
    # der ball kann niemals hinter dem servo sein, also ist P_X der äußerste Punkt auf dieser achse

    return tangent_ang, S_X1, S_Y1, S_X2, S_Y2


def setPoint_ball():  # setpoint for ball movement (include model from robot motion and relativ ball motion)
    '''
    Berechnung des Geschwindigkeitsbetrags des Balls in kartesischen Koordinaten V(X,Y)
    Zur Zeit befindet sich der ideale Dribbelpunkt vor dem Roboter in X-Richrung ohne Abweichung in Y-Richtung
    Überlegung: stattdessen aktuellen Dribbelpunkt verwenden!

    (akutelle Umsetzung) Um den Ball im idealen Dribbelpunkt zu dribbeln, werden
    die folgenden Ballgeschwindigkeiten benötigt:
    '''
    V_X = robot.V_X + np.cos(np.deg2rad(robot.w_Z)) * np.sqrt(
        ball_measure.P_X * ball_measure.P_X + ball_measure.P_Y * ball_measure.P_Y) - ball_measure.V_X
    V_Y = robot.V_Y + np.sin(np.deg2rad(robot.w_Z)) * np.sqrt(
        ball_measure.P_X * ball_measure.P_X + ball_measure.P_Y * ball_measure.P_Y) - ball_measure.V_Y
    return V_X, V_Y


def cart2pol(beg, end):  # transform cartesian coordiantes to polar
    rel = (end[0] - beg[0], end[1] - beg[1])
    mag = np.hypot(rel[0], rel[1])
    #print("mag= ", mag, beg, end)
    #print("rel= ", rel)
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

    # m = rel[0] * (1 / rel[1])
    # print("m ", m)
    # np.arctan((m - 1) / 1 + m)


def wheel_velocity(ball_mag, ball_ang):
    v_left = -(ball_mag * (np.cos(-servo_left.ang_norm + ball_ang) + np.sin(servo_left.ang_norm + ball_ang)))
    v_right = -(ball_mag * (np.cos(-servo_left.ang_norm + ball_ang) + np.sin(-servo_left.ang_norm + ball_ang)))
    return v_left, v_right

'''
Main Functions
'''


def catch_ball():
    global impact_point
    # print("deg= ", i, "rad= ", np.deg2rad(robot.w_Z + i))
    # calculate ideal dribbel point
    ball_set.q_dot = setPoint_ball()
    impact_point = (impact_Y(), ball_distance_max)
    print("imapact point: ", impact_point)
    # abs_val, ang = cart2pol(ball_set.q_dot, origin)
    # calculate wheel rotation
    abs_val, ang = cart2pol(ball_set.q_dot, impact_point)
    wheel_left.V_now, wheel_right.V_now = wheel_velocity(abs_val, ang)
    wheel_left.n_now = wheel_left.V_now * gear_bw
    wheel_right.n_now = wheel_right.V_now * gear_bw
    #print("q_dot= ", ball_set.q_dot)
    #print("abs, angle: ", abs_val, np.rad2deg(ang))
    #print("V_left | V_right: ", wheel_left.V_now, wheel_right.V_now)
    #print("n_left | n_right: ", wheel_left.n_now, wheel_right.n_now)
    # calculate servo angle
    servo_left.ang_tang, x2, y2, wheel_left.P_X, wheel_left.P_Y = tangent_point(servo_left.r, servo_left.P_X,
                                                                                servo_left.P_Y,
                                                                                impact_point[1], impact_point[0])
    servo_right.ang_tang, wheel_right.P_X, wheel_right.P_Y, x3, y3 = tangent_point(servo_right.r, servo_right.P_X,
                                                                                   servo_right.P_Y,
                                                                                   impact_point[1], impact_point[0])
    # wheel_right.P_X, wheel_right.P_Y
    #servo_left.ang_set = cart2pol((servo_left.P_X, servo_left.P_Y), (wheel_left.P_X, wheel_left.P_Y))[1]
    servo_left.ang_set = -servo_left.ang_tang
    print("ang_left: ", servo_left.ang_set, np.rad2deg(servo_left.ang_set))
    '''
    if servo_left.ang_set > servo_left.ang_max:
        servo_left.ang_set = servo_left.ang_max
    elif servo_left.ang_set < servo_left.ang_min:
        servo_left.ang_set = servo_left.ang_min
    '''
    # wheels are orthogonal to tangents position line
    #servo_right.ang_set = cart2pol((servo_right.P_X, servo_right.P_Y), (wheel_right.P_X, wheel_right.P_Y))[1]
    servo_right.ang_set = -servo_right.ang_tang
    '''
    if servo_right.ang_set > servo_right.ang_max:
        servo_right.ang_set = servo_right.ang_max
    elif servo_right.ang_set < servo_right.ang_min:
        servo_right.ang_set = servo_right.ang_min
    '''
    #print("wheel_L x|y: ", wheel_left.P_X, wheel_left.P_Y)
    print("tangete angle: %5.4f (%5d)|%5.4f (%5d) ", servo_left.ang_set, np.rad2deg(servo_left.ang_set),
          servo_right.ang_set, np.rad2deg(servo_right.ang_set))


def dribbel_ball():
    # calculate wheel rotation
    abs_val, ang = cart2pol(ball_set.q_dot, impact_point)
    wheel_left.V_now, wheel_right.V_now = wheel_velocity(abs_val, ang)
    wheel_left.n_now = wheel_left.V_now * gear_bw
    wheel_right.n_now = wheel_right.V_now * gear_bw


def conquer_ball():
    pass  #TO DO: based on bahviour from ALICA by communication over ROS-node (witch is not implemented yet)


'''
MAIN LOOP
'''
gear_bw = r_ball * (1 / wheel_left.r)

if ball_measure.P_X <= ball_distance_max:
    ball_status = HAVE_BALL
elif ball_measure.P_X > ball_distance_max:
    ball_status = NEAR_BALL
else:
    ball_status = FAR_BALL

execute_task = {
    # this status is send to robots main system
    NEAR_BALL: catch_ball(),
    HAVE_BALL: dribbel_ball(),
    # FAR_BALL: is in this case not relevant because its only send to main system
    # THIS is used then the conquer process is initiated by Robots main system, decision is not made here
    CONQUER_BALL: conquer_ball()
}

execute_task.get(ball_status, lambda: 'ball status unknown')
print("ball status: ", ball_status)


'''
VISUALIZATION
'''

def showDrawing():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # Servo links
    plt.plot((servo_left.P_Y), (servo_left.P_X), 'o', color="r")
    # plt.plot((servo_left.P_Y + wheel_left.P_Y), (servo_left.P_X + wheel_left.P_X), 'o', color="r")
    circle = plt.Circle((servo_left.P_Y, servo_left.P_X), servo_left.r, fill=False)
    ax.add_patch(circle)

    # Servo rechts
    plt.plot((servo_right.P_Y), (servo_right.P_X), 'o', color="r")
    circle = plt.Circle((servo_right.P_Y, servo_right.P_X), servo_right.r, fill=False)
    ax.add_patch(circle)

    # aktuelle ball position
    plt.plot((ball_measure.P_Y), (ball_measure.P_X), 'o', color="orange")
    circle = plt.Circle((ball_measure.P_Y, ball_measure.P_X), r_ball, fill=False)
    ax.add_patch(circle)

    # velocities
    if SHOW_WITH_VEL == 1:
        plt.plot([ball_measure.P_Y, ball_measure.P_Y + ball_measure.V_Y],
                 [ball_measure.P_X, ball_measure.P_X + ball_measure.V_X], color='orange')
        print("V ball ", ball_measure.V_Y, ball_measure.V_X)
        plt.plot([0, robot.V_Y], [0, robot.V_Y], color='purple')

    # wheel left
    plt.plot((wheel_left.P_Y), (wheel_left.P_X), 'o', color="g")


    # wheel_right
    plt.plot((wheel_right.P_Y), (wheel_right.P_X), 'o', color="g")

    # ideal line
    plt.plot([-250, 250], [ideal_line, ideal_line], color='black')
    plt.plot((impact_point[0]), (impact_point[1]), 'o', color="r")

    # Ball
    # plt.plot((ball_measure.P_Y), (ball_measure.P_X), 'o', color="r")
    # circle = plt.Circle((ball_measure.P_Y, ball_measure.P_X), r_ball, fill=False)
    # ax.add_patch(circle)
    # Hilfsgeometrie
    plt.plot([wheel_left.P_Y, impact_point[0]], [wheel_left.P_X, impact_point[1]],
             color='b')  # remember: [x1,x2],[y1,y2]
    plt.plot([wheel_right.P_Y, impact_point[0]], [wheel_right.P_X, impact_point[1]], color='b')
    # P2_X = servo_left.P_X + (50 * np.sin(servo_left.ang_set))
    # P2_Y = servo_left.P_Y + (50 * np.cos(servo_left.ang_set))

    # Abstandskreis
    # circle = plt.Circle(((impact_point[0]), (impact_point[1])), c, fill=False)
    #ax.add_patch(circle)

    # check angle
    # WB = ball_measure.P_X - wheel_left.P_X, ball_measure.P_Y - wheel_left.P_Y
    # SB = ball_measure.P_X - servo_left.P_X, ball_measure.P_Y - servo_left.P_Y
    # print("Hypo, GK: ", np.sqrt(WB[0] * WB[0] + WB[1] * WB[1]), SB[0] * SB[0] + SB[1] * SB[1])
    # Hypo = np.sqrt(SB[0] * SB[0] + SB[1] * SB[1])
    #GK = 200  # np.sqrt(WB[0]*WB[0]+WB[1]*WB[1])
    dummy_ang = servo_left.ang_set  #np.deg2rad(-32) servo_left.ang_set   np.deg2rad(-30)
    #print("dummy ang = ", dummy_ang, np.rad2deg(dummy_ang))
    plt.plot([servo_left.P_Y, servo_left.P_Y + servo_left.r * np.sin(dummy_ang)],
             [servo_left.P_X, servo_left.P_X + servo_left.r * np.cos(dummy_ang)], color='r')
    plt.plot([servo_right.P_Y, servo_right.P_Y + servo_right.r * np.sin(servo_right.ang_set)],
             [servo_right.P_X, servo_right.P_X + servo_right.r * np.cos(servo_right.ang_set)], color='r')
    #print("cos|sin ", np.cos(servo_left.ang_set), np.sin(servo_left.ang_set), np.cos(1 - servo_left.ang_set),
          #np.sin(1 - servo_left.ang_set))
    # plt.plot((P2_X), (P2_Y), 'o', color="b")
    # plt.plot((servo_left.P_X + 50 * np.cos(servo_left.ang_set)), servo_left.P_Y + 70 * np.sin(servo_left.ang_set), 'o', color="b")
    # circle = plt.Circle((M_X + b * np.cos(tangent_ang), M_Y + b * np.sin(tangent_ang)), b, fill=False)
    # ax.add_patch(circle)

    plt.xlim(-500, 500)
    plt.ylim(0, 600)
    ax.axis('equal')
    plt.show()


if VISUSUALIZATION == ON:
    showDrawing()
