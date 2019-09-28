# -*- coding: utf-8 -*-
"""
Created on Sun May 26 11:47:21 2019

@author: User
"""

import RPi.GPIO as gpio
import time
import Adafruit_PCA9685
import sys 

pwm_min = 5
pwm_min_rel = 150
pwm_max_rel = 600

servo = Adafruit_PCA9685.PCA9685() 

# PWM-Frequenz auf 50 Hz setzen
servo.set_pwm_freq(50)

# PWM starten, Servo auf 0 Grad
servo.set_pwm(pwm_min) #2.5

# Umrechnung Grad in Tastverhaeltnis
def setservo(winkel):
  if winkel < 0:
    winkel = 0
  elif winkel > 180:
    winkel = 180
  #pwm = winkel/36 + pwm_min
 # pwm_length = winkel/36 + pwm_min
  pwm =50*(winkel/36 + pwm_min)*4096
  if pwm > pwm_max_rel:
			 pwm = pwm_max_rel
  servo.set_pwm(0,0,pwm)

try:
  # Endlosschleife Servoansteuerung
  while True:
    winkel = raw_input("Winkel eingeben (0 - 180): ")
    winkel = float(winkel)
    setservo(winkel)

except KeyboardInterrupt:
  # Abbruch mit [Strg][C],
  # Servo auf 0 Grad, PWM beenden
  servo.set_pwm(0,0,pwm_min_rel)
  servo.stop()
  gpio.cleanup()
