import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

pwmpin = "P8_66"
enablepin = "P8_69"
directionpin = "P_8_45"

# GPIO initialisieren
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwmpin, GPIO.OUT)
GPIO.setup(enablepin, GPIO.OUT)
GPIO.setup(directionpin, GPIO.OUT)


# PWM-Frequenz auf 53.6 kHz setzen
motor = GPIO.PWM(pwmpin, 53600)

# PWM starten, Servo auf 0 Grad
motor.start(0)

#Drehrichtung eingeben
def setdir(dir):
  GPIO.output(directionpin, dir)

# Umrechnung Grad in Tastverhaeltnis
def setmotor(vel):
  if vel < 0:
    vel = 0
  elif vel > 4000:
    vel = 4000
  pwm = 0.025*vel
  motor.ChangeDutyCycle(pwm)

try:
  GPIO.output(enablepin, 1)
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
  GPIO.cleanup()
