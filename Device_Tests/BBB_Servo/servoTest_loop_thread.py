#from: http://www.toptechboy.com/tutorial/beaglebone-black-lesson-6-control-pwm-signals-on-gpio-pins-from-python/

import Adafruit_BBIO.PWM as PWM
from time import sleep
import threading
#import Adafruit_BBIO.GPIO as GPIO
#from Adafruit_BBIO.PWM import PWM
#GPIO.setup("P8_13", GPIO.OUT)

steps  = 7
SERVO_1="P9_14"
SERVO_2="P9_16"
pwm_min = 5
pwm_max = 10
freq = 50 #Hz

start_duty = 0

PWM.start(SERVO_1,0,freq)
PWM.start(SERVO_2,0,freq)
angle = 0

def setServo_1(angle):
	if angle < 0:
		angle = 0
	elif angle > 180:
		angle = 180
	duty = angle/36.0+ pwm_min
	if duty > pwm_max:
		duty = pwm_max
	PWM.set_duty_cycle(SERVO_1, duty)
	print "duty 1", duty

def setServo_2(angle):
	if angle < 0:
		angle = 0
	elif angle > 180:
		angle = 180
	duty = angle/36.0+ pwm_min
	if duty > pwm_max:
		duty = pwm_max
	PWM.set_duty_cycle(SERVO_2, duty)
	print "duty 2", duty


def servoLoop_1():
	#Servo_1 = PWM.start(SERVO_1,start_duty,freq)

	i = 0
	while i < 51:
		print "angle ",i
		setServo_1(i)
		sleep(1)
		i += steps
	while i > 0:
		print "angle ",i
		setServo_2(i)
		sleep(1)
		i -= steps

def servoLoop_2():
	#Servo_2 = PWM.start(SERVO_2,start_duty,freq)
	i = 0
	while i < 50:
		print "angle ",i
		setServo_1(i)
		sleep(1)
		i += steps

	while i > 0:
		print "angle ",i
		setServo_2(i)
		sleep(1)
		i -= steps

if __name__=="__main__":
	
#	PWM.start(SERVO_1,start_duty,freq)
#	PWM.start(SERVO_2,start_duty,freq)
	#Servo_1 = PWM.start(SERVO_1,start_duty,freq)
	#Servo_2 = PWM.start(SERVO_2,start_duty,freq)
	#servoLoop_1()
	#servoLoop_2()
#	setServo_1(20)
#	setServo_2(20)

'''
Does not work at the moment because loop is missing
'''
	angle = 0
	s1 = threading.Thread(target=setServo_1)
	s1.setDaemon(True)
	s2 = threading.Thread(target=setServo_2)
	s2.setDaemon(True)
	s1.start()
	s2.start()	
	for j in range(20):
		angle = j
		print "angle loop: ",angle
		sleep(1)

	s1.join()
	s2.join()

	PWM.stop(SERVO_1)
	PWM.stop(SERVO_2)
	PWM.cleanup()
