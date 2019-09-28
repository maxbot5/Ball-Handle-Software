#from: http://www.toptechboy.com/tutorial/beaglebone-black-lesson-6-control-pwm-signals-on-gpio-pins-from-python/

import Adafruit_BBIO.PWM as PWM


PWMport_1="P9_14"
PWMport_2="P8_13"

PWM.start(PWMport_1,0,50)
PWM.start(PWMport_2,0,53000)

for i in range(0,5):
    DC_1=input("What Duty Cycle Would You Like on Port 1(0-100)? ")
    PWM.set_duty_cycle(PWMport_1, DC_1)
    DC_2=input("What Duty Cycle Would You Like on Port 2 (0-100)? ")
    PWM.set_duty_cycle(PWMport_2, DC_2)

PWM.stop(PWMport_1)
PWM.stop(PWMport_2)
PWM.cleanup()
