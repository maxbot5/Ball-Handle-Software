# from: http://www.toptechboy.com/tutorial/beaglebone-black-lesson-6-control-pwm-signals-on-gpio-pins-from-python/

# import Adafruit_BBIO.PWM as PWM
# import Adafruit_BBIO.GPIO as GPIO
# from Adafruit_BBIO.PWM import PWM
# GPIO.setup("P8_13", GPIO.OUT)

SERVO_L = "P9_14"
SERVO_R = "P9_16"
pwm_min = 5
pwm_max = 10
freq = 50  # Hz
start_duty = 0

# PWM.start(SERVO_L,0,freq)
# PWM.start(SERVO_R,0,freq)

# Ausrichtung auf 0° relativ zum Roboter
Servo_L_zero = 40
Servo_R_zero = 140
# Servowinkel soll fuer ideale Dribbelposition
Servo_L_ideal = 54
Servo_R_ideal = 54
offset_servo_L = 0
offset_servo_R = 0

'''		
						0°
	L__/_\__R          	|
	| Robot	|		--------- 90°
	|_______|			|
'''


def servo_init():
    # Servo_1 = PWM.start(SERVO_L, start_duty, freq)
    # Servo_2 = PWM.start(SERVO_R, start_duty, freq)
	global offset_servo_L, offset_servo_R
	servo_L_ok = False
	servo_R_ok = False
	ready = input("Ready to init servo? (y|n)")
	if ready is 'y':
		while (servo_L_ok is not True or servo_R_ok is not True):

			servo_run(Servo_L_zero + Servo_L_ideal + offset_servo_L, Servo_R_zero + Servo_R_ideal + offset_servo_R)
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


'''
Das mit einer Funktion zu machen hat sich als kacke herausgestellt!
def servo_calc(ang_soll):
    return Servo_L_zero + ang_soll, Servo_R_zero + ang_soll
'''


def servo_run(ang_left, ang_right):  # der blanke winkel, um den sich der servo dreht

    def setServo_L(angle):
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        duty = angle / 36.0 + pwm_min
        if duty > pwm_max:
            duty = pwm_max
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


# try:

servo_init()

for i in range(10):
    angle = int(input("Set current angle: "))
    print("angle ", angle)
    servo_run(angle, angle)
# except:
PWM.stop(SERVO_1)
PWM.stop(SERVO_2)
PWM.cleanup()
