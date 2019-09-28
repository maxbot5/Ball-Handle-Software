import RPi.GPIO as gpio
import time

# Servo-GPIO (PWM-GPIO 18, Pin 12)
pwmpin = 17
enablepin = 23
directionpin = 24

# GPIO initialisieren
gpio.setmode(gpio.BCM)
gpio.setup(pwmpin, gpio.OUT)
gpio.setup(enablepin, gpio.OUT)
gpio.setup(directionpin, gpio.OUT)


# PWM-Frequenz auf 53.6 kHz setzen
motor = gpio.PWM(pwmpin, 53600)

# PWM starten, Servo auf 0 Grad
motor.start(0)

#Drehrichtung eingeben
def setdir(dir):
  gpio.output(directionpin, dir)

# Umrechnung Grad in Tastverhaeltnis
def setmotor(vel):
  if vel < 0:
    vel = 0
  elif vel > 4000:
    vel = 4000
  pwm = 0.025*vel
  motor.ChangeDutyCycle(pwm)

try:
  gpio.output(enablepin, 1)
  # Endlosschleife Servoansteuerung
  while True:
    dir = raw_input("Richtung eingeben (0,1): ")
    dir = int(dir)   
    setdir(dir)
    vel = raw_input("Geschwindigkeit eingeben (0 - 4000): ")
    vel = float(vel)
    setmotor(vel)

except KeyboardInterrupt:
  # Abbruch mit [Strg][C],
  # Servo auf 0 Grad, PWM beenden
  motor.stop()
  gpio.cleanup()
