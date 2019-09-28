
'''
Constants
'''
# parameter for visualization
ON = 1
OFF = 0
SHOW_WITH_VEL = ON
VISUSUALIZATION = ON

'''
Ball specific
'''
# ball status
# parameter for interception
r_ball = 114.8  # mm
origin = (0, 0)
ideal_line = 375
ball_distance_max = 400
impact_point = (375, 0)

'''
Wheel specific
'''
# wheel parameter
R_WHEEL = 40
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

'''
Robot specific
'''
# robot
V_ROB_MAX = 2000
V_ROB_MIN = -2000

'''
Servo specifig
'''
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
WAIT_FOR_SERVO = 0.2 #1sek = 1000 ms