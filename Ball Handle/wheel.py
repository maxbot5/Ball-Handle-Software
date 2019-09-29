import time
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
from constants import *

class Wheel():
    def __init__(self, pin_en, pin_dir, pin_pwm, freq=F_MOTOR, start_duty=0, voltage=VOLTAGE_WHEEL,
                 slip_ideal=SLIP_IDEAL, slip_crit=SLIP_CRITICAL, v_min=V_ROB_MIN, v_max=V_ROB_MAX):
        self.freq = freq

        self.start_duty = start_duty
        self.V_set = 0
        self.V_min = v_min
        self.V_max = v_max
        self.V_now = 0
        self.gear_rpm = (1 / 7) * 19.500  # normalized rotational speed of motor in rpm
        self.gear_hz = (1 / (7 * 8)) * (26 * 100)  # same but in Hz
        self.dir = 1
        self.n_set = 0
        self.n_new = 0
        self.n_old = 0
        self.slip_factor_ideal = 1 / (1 - slip_ideal)
        self.slip_factor_crit_max = 1 / (1 - slip_crit)
        self.slip_factor_crit_min = 1 - self.slip_factor_crit_max
        self.n_min = 0  #v_min * self.gear_rpm  # calculate from Vmin and gear wheel2ball and gear motor2wheel
        self.n_max = v_max * self.gear_rpm
        self.U = voltage
        self.pwm_factor = (1 / (self.n_max - self.n_min)) * 100  # seems to be not relevant: self.U
        self.P_X = 0
        self.P_Y = 0
        self.pwmpin = pin_pwm
        self.enablepin = pin_en
        self.directionpin = pin_dir
        self.motor_init()

    def motor_init(self):
        # GPIO initialisieren
        GPIO.setup(self.pwmpin, GPIO.OUT)
        GPIO.setup(self.enablepin, GPIO.OUT)
        GPIO.setup(self.directionpin, GPIO.OUT)

        GPIO.output(self.enablepin, GPIO.HIGH)
        # PWM-Frequenz auf 53.6 kHz setzen
        PWM.start(self.pwmpin,0, self.freq)

    # transform incoming velocity in mm TO motor rotational speed
    def mmps2rpm(self, Vel):
        return self.gear_rpm * Vel

    # Drehrichtung eingeben
    def set_dir(self):
        if self.V_set > 0:
            GPIO.output(self.directionpin, CCW)
            #print("drive forward")
        elif self.V_set < 0:
            GPIO.output(self.directionpin, CW)
            #print("drive backward")

    # Umrechnung Grad in Tastverhaeltnis
    def set_motor(self, n):
        if n < self.n_min:
            n = self.n_min
        elif n > self.n_max:
            n = self.n_max
        pwm = n * self.pwm_factor  # 0.025 * n
        PWM.set_duty_cycle(self.pwmpin, pwm)
        #print("rpm: ", n)
        #print("duty: ", pwm)

    def run(self):
        self.n_set = self.gear_rpm * abs(self.V_set)
        #print("n_set", self.n_set)
        self.set_dir()
        self.set_motor(self.n_new)

    def pause(self,state):
        PWM.set_duty_cycle(self.pwmpin, 0)
        if state is OFF:
            GPIO.output(self.enablepin, GPIO.HIGH)
        else:
            GPIO.output(self.enablepin, GPIO.LOW)

    def end_routine(self):
        PWM.stop(PIN_PWM_WHEEL_LEFT)
        PWM.stop(PIN_PWM_WHEEL_RIGHT)
        PWM.stop()
        PWM.cleanup()

    def hand_control(self):
        try:
            #GPIO.output(self.enablepin, 1)
            time.sleep(1)  # pause for 1 second to recognize abort
            # Endlosschleife Servoansteuerung
            # dir = input("Richtung eingeben (0,1): ")
            # dir = int(dir)
            #print("dir: ", dir)
            #self.set_dir(dir)
            vel = input("Geschwindigkeit eingeben (0 - 4000): ")
            self.V_set = float(vel)
            #("V_set", self.V_set)
            self.run()

        except KeyboardInterrupt:
            # Abbruch mit [Strg][C],
            # motor auf 0 rpm, PWM beenden
            self.stop()
            self.cleanup()


def test_wheels(sim_mode=False):
    # ball
    R_BALL = 114.2
    # wheel
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

    print("init wheels")
    wheel_left = Wheel(pin_en=PIN_EN_WHEEL_LEFT, pin_dir=PIN_DIR_WHEEL_LEFT, pin_pwm=PIN_PWM_WHEEL_LEFT)
    wheel_right = Wheel(pin_en=PIN_EN_WHEEL_RIGHT, pin_dir=PIN_DIR_WHEEL_RIGHT, pin_pwm=PIN_PWM_WHEEL_RIGHT)

    if sim_mode:
        print("start simulation by hand")
        wheel_left.hand_control()
        wheel_right.hand_control()
    else:
        wheel_left.run()
        wheel_right.run()

    while(True):
        try:
            test_wheels(sim_mode=True)
        except KeyboardInterrupt:
            PWM.stop(PIN_PWM_WHEEL_LEFT)
            PWM.stop(PIN_PWM_WHEEL_RIGHT)
            PWM.stop()
            PWM.cleanup()

if __name__ == '__main__':
    test_wheels()