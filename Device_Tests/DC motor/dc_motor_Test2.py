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
	'''
	V_robot = 4000 mm/s, r_ball = 114.1 mm, r_wheel = 40 mm
	n_wheel = V_robot/(r_wheel *2*pi) = 15.915 mm/s = 954.93 mm/min

	resolution = 4096 (12 Bit Auflösung)

	res_max = 954.93/4096 = 0.2331 mm*bit/min
	'''
  if vel < 0:
    vel = 0
  elif vel > 955:
    vel = 955
  pwm = 0.2331*vel
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
