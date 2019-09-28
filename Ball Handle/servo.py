# coding: utf8

import Adafruit_BBIO.PWM as PWM

'''
Coordinate System
                X
     0          |
-90___|___90   |_ _ Y
'''

class Servo(object):
    def __init__(self, name, port, ang_min, ang_max, ang_crit, ang_start, ang_dribbel,
                 ang_offset, p_x, p_y, sim_mode=False, voltage=SERVO_VOLTAGE, radius=SERVO_RADIUS, freq=SERVO_FREQ,
                 start_duty=7.5,
                 pwm_min=5, pwm_max=10, pwm_crit_min, pwm_crit_max):
        # self.pwm = PWM.start(port,start_duty,freq)
        self.sim_mode = sim_mode
        if not self.sim_mode:
            PWM.start(channel=port, duty_cycle=start_duty, frequency=freq)
        self.name = name  # string
        self.port = port
        self.duty = 0
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.freq = freq  # Hz
        self.start_duty = start_duty
        self.r = radius
        self.pwm_factor = (pwm_max - pwm_min) / 180  #4/85
        self.ang_dribbel = ang_dribbel
        self.ang_start = (start_duty - pwm_min) * (
                    1 / self.pwm_factor)  #(start_duty-pwm_min) * 85/4  # startposotion of servos turning direction (starts in middle postion)
        self.ang_min = ang_min  #values for safty
        self.ang_max = ang_max
        self.ang_crit = ang_crit
        self.ang_norm = ang_start
        self.ang_set = 0  # incoming angle command as absolut angle in relation to robots CS
        self.ang_abs = 0
        self.ang_rel = 0
        self.ang_tang = 0
        self.U = voltage
        self.P_X = p_x  #position of the servo-axle at robots coordinate system
        self.P_Y = p_y
        self.ang_offset = ang_offset  # rotational shifting of the servo chassis mounting in relation to the robor
        #self.pwm_factor = abs(1 / (self.ang_max - self.ang_min)) * (self.pwm_max - self.pwm_min)  # self.U
        self.pwm_crit_min = pwm_crit_min #minimal possible position
        self.pwm_crit_max = pwm_crit_max #maximal possible position



    def calculate_servo_ang(self):
        '''
        Berechnung des Winkels, der als pwm-signal an den jeweiligen Servo gesendet wird.
        '''
        # self.ang_abs = self.ang_offset + self.ang_rel
        #self.ang_rel = self.ang_set + self.ang_offset - self.ang_start -self.ang_start
        self.ang_rel = self.ang_set + self.ang_offset
        '''
        if self.name is 'left':
            self.ang_rel = self.ang_set + self.ang_offset
        elif self.name is 'right':
            self.ang_rel = self.ang_set - self.ang_offset
        '''
        print("name", self.name, "ang_rel calculated: ", self.ang_rel)

    def process(self):
        self.calculate_servo_ang()
        '''
        if self.ang_abs < self.ang_offset - self.ang_crit:
            self.ang_abs = self.ang_offset - self.ang_crit

        elif self.ang_abs > self.ang_offset + self.ang_crit:
            self.ang_abs = self.ang_offset + self.ang_crit
        
        if self.ang_rel < - self.ang_crit:
            self.ang_rel = - self.ang_crit

        elif self.ang_rel > self.ang_crit:
            self.ang_rel = self.ang_crit
        '''
        '''
        if self.name is 'left':
            self.ang_abs = (self.ang_start + self.ang_rel)
        elif self.name is 'right':
            self.ang_abs = (self.ang_start + self.ang_rel)
         
        if self.ang_abs > self.ang_offset:
            self.ang_abs = (self.ang_start + self.ang_rel)
        elif self.ang_abs < self.ang_offset:
            self.ang_abs = (self.ang_start - self.ang_rel)
        '''
        #print("ang_abs: ", self.ang_abs)
        #print("pwm_factor: ", self.pwm_factor)
        #self.duty = np.round((self.ang_abs * self.pwm_factor) + self.pwm_min,2)
        self.ang_abs = (self.ang_start + self.ang_rel)
        self.duty = (self.ang_abs * self.pwm_factor) + self.pwm_min
        '''
        # redundance for more saftey
        if self.duty > self.start_duty + self.pwm_crit:
            self.duty = self.start_duty + self.pwm_crit

        if self.duty < self.start_duty - self.pwm_crit:
            self.duty = self.start_duty - self.pwm_crit

        if self.duty > self.pwm_max:
            self.duty = self.pwm_max

        if self.duty < self.pwm_min:
            self.duty = self.pwm_min
        '''
        # this ensures the protection of the wheels not to hit the construction
        if self.duty > self.pwm_crit_max:
            self.duty = self.pwm_crit_max

        if self.duty < self.pwm_crit_min:
            self.duty = self.pwm_crit_min
        print("duty of ", self.port, self.duty)
        #if self.sim_mode == False:
        PWM.set_duty_cycle(self.port, self.duty)
        #time.sleep(0.2)



    def hand_control(self):
        ang = input("set angle for ",self.name,": ")
        self.ang_set = float(ang)
        self.run()
        '''
        self.duty = self.ang_abs * self.pwm_factor + self.pwm_min

        if self.duty > self.pwm_crit:
            self.duty = self.pwm_crit
        '''

        #PWM.set_duty_cycle(self.port, self.duty)
        #print("duty of ", self.port, self.duty)
def testing():
    # Servo constants
    SERVO_ANG_OFFSET_LEFT = 60  # 50 #50  # np.deg2rad(-40)
    SERVO_ANG_OFFSET_RIGHT = 35  # -50 #-50  # np.deg2rad(40)
    SERVO_ANG_DRIBBEL_LEFT = 0
    SERVO_ANG_DRIBBEL_RIGHT = 0
    SERVO_PORT_LEFT = "P9_14"
    SERVO_PORT_RIGHT = "P9_16"
    SERVO_FREQ = 50
    SERVO_RADIUS = 55
    SERVO_VOLTAGE = 5  # Volt
    SERVO_ANG_CRIT_LEFT = 85  # np.deg2rad(-85)
    SERVO_ANG_CRIT_RIGHT = -85  # np.deg2rad(85)
    SERVO_ANG_START = 60
    #pwm_min and pwm_max extract from script pwmTestBBB_1pin.py
    def servo_test(sim_mode=True):

        servo_left = Servo(sim_mode=False, name='left', port="P9_14", ang_min=40, ang_max=125,
                           ang_crit=SERVO_ANG_CRIT_LEFT,
                           ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_LEFT, pwm_min=5, pwm_max=10,
                           start_duty=7.75,
                           ang_offset=SERVO_ANG_OFFSET_LEFT, p_x=30, p_y=-100)

        servo_right = Servo(sim_mode=False, name='right', port="P9_16", ang_min=-125, ang_max=-40,
                            ang_crit=SERVO_ANG_OFFSET_RIGHT,
                            ang_start=SERVO_ANG_START, ang_dribbel=SERVO_ANG_DRIBBEL_RIGHT, pwm_min=5, pwm_max=10,
                            ang_offset=SERVO_ANG_OFFSET_RIGHT, p_x=30, p_y=100)

        if sim_mode:
            for count in range(0, 10):
                servo_left.hand_control()
                servo_right.hand_control()
        else:
            servo_left.run()
            servo_right.run()

    try:
        servo_test(sim_mode=True)
    except KeyboardInterrupt:
        PWM.stop(SERVO_PORT_LEFT)
        PWM.stop(SERVO_PORT_RIGHT)
        PWM.cleanup()

testing()