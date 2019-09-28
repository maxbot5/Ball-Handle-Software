#from: http://www.toptechboy.com/tutorial/beaglebone-black-lesson-6-control-pwm-signals-on-gpio-pins-from-python/

import Adafruit_BBIO.PWM as PWM
from time import sleep
#import Adafruit_BBIO.GPIO as GPIO
#from Adafruit_BBIO.PWM import PWM
#GPIO.setup("P8_13", GPIO.OUT)

steps  = 60

SERVO_1="P9_14"
SERVO_2="P9_16"
pwm_min = 5
pwm_max = 10
freq = 50 #Hz

start_duty = 0

PWM.start(SERVO_1,0,freq)
PWM.start(SERVO_2,0,freq)


def setServo_1(angle):
	if angle < 0:
		angle = 0
	elif angle > 180:
		angle = 180
	duty = angle/36.0+ pwm_min
	if duty > pwm_max:
		duty = pwm_max
	PWM.set_duty_cycle(SERVO_1, duty)
#	print "duty 1", duty

def setServo_2(angle):
	if angle < 0:
		angle = 0
	elif angle > 180:
		angle = 180
	duty = angle/36.0+ pwm_min
	if duty > pwm_max:
		duty = pwm_max
	PWM.set_duty_cycle(SERVO_2, duty)
#	print "duty 2", duty


#try:
Servo_1 = PWM.start(SERVO_1,start_duty,freq)
Servo_2 = PWM.start(SERVO_2,start_duty,freq)
#for i in range(10):
i = 0
while i < 181:
	#angle=input("What Duty Cycle Would You Like (0-100)? ")
	print "angle ",i
	setServo_1(i)
	setServo_2(i)
	sleep(1)
	i += steps
while i > 0:
	#angle=input("What Duty Cycle Would You Like (0-100)? ")
	print "angle ",i
	setServo_1(i)
	setServo_2(i)
	sleep(1)
	i -= steps
setServo_1(0)
setServo_2(0)
sleep(1)
#except:
PWM.stop(SERVO_1)
PWM.stop(SERVO_2)
PWM.cleanup()
