#from: http://www.toptechboy.com/tutorial/beaglebone-black-lesson-6-control-pwm-signals-on-gpio-pins-from-python/

import Adafruit_BBIO.PWM as PWM
#import Adafruit_BBIO.GPIO as GPIO

#GPIO.setup("P8_13", GPIO.OUT)
myPWM="P9_16"
pwm_min = 5
pwm_max = 10
freq = 50 #Hz
PWM.start(myPWM, 7.5, freq)

def setservo (angle):
	if angle < 0:
		angle = 0
    elif angle > 120:
        angle = 120
    duty = angle / 24.0 + pwm_min
	if duty > pwm_max:
		duty = pwm_max
	PWM.set_duty_cycle(myPWM, duty)
    print("duty", duty)
#for i in range(0,5):

try:
	
	while True:
        # angle=input("What Duty Cycle Would You Like (0-100)? ")
        # print ("angle ",angle)
        # setservo(float(angle))
        duty = input("What Duty Cycle Would You Like (0-100)? ")
        # setservo(float(duty))
        PWM.set_duty_cycle(myPWM, float(duty))
except:
    PWM.stop(myPWM)
    PWM.cleanup()
