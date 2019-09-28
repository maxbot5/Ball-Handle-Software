#coding=utf-8

import Adafruit_BBIO.PWM as PWM
import time
import Adafruit_BBIO.GPIO as GPIO


# Servo-GPIO (PWM-GPIO 18, Pin 12)
MOTOR_1 = "P8_13"  #pwm pin
MOTOR_2 = "P8_19"

enablepin_1 = "P8_9" 
directionpin_1 = "P8_11" 

# GPIO initialisieren
GPIO.setup(enablepin_1, GPIO.OUT)
GPIO.setup(directionpin_1, GPIO.OUT)

# PWM-Frequenz auf 53.6 kHz setzen
PWM.start(MOTOR_1,0, 53600)
PWM.start(MOTOR_2,0, 53600)


#Drehrichtung eingeben
def setdir(pin,dir):
  if dir is True:
 	 GPIO.output(pin, GPIO.HIGH)
  else:
 	 GPIO.output(pin, GPIO.LOW)

# Umrechnung Velocity Robot in Tastverhaeltnis
def setmotor_1(V_robot):
	'''
	V_robot = 4000 mm/s, r_ball = 114.1 mm, r_wheel = 40 mm
	n_wheel = V_robot/(r_wheel *2*pi) = 15.915 mm/s = 954.93 mm/min

	resolution = 4096 (12 Bit Auflösung)

	1/(r_wheel *2*pi)*60 = 0.23873 1/min
	pwm = 1/n_wheel_max *100 = 1/954.92 *100 =0.10472 
	'''
	if V_robot < 0:
   		V_robot = 0
	elif V_robot > 4000:
 		V_robot = 4000
	n_wheel = V_robot*0.23873
	pwm = n_wheel*0.10472
	PWM.set_duty_cycle(MOTOR_1, pwm)
	print "duty: ",pwm

try:
  GPIO.output(enablepin_1, GPIO.HIGH)
  # Endlosschleife Servoansteuerung
  while True:
    dir = raw_input("Richtung eingeben (0,1): ")
    dir = bool(dir)   
    setdir(directionpin_1,dir)

    V_robot = raw_input("Geschwindigkeit eingeben (0 - 4000): ")
    V_robot = float(V_robot)
    setmotor_1(V_robot)

except KeyboardInterrupt:
  # Abbruch mit [Strg][C],
  # Servo auf 0 Grad, PWM beenden
	PWM.stop(MOTOR_1)
	PWM.stop(MOTOR_2)
	GPIO.cleanup()
