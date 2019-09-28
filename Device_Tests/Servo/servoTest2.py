import RPi.GPIO as gpio
import time

# Servo-GPIO (PWM-GPIO 18, Pin 12)
servopin = 17

# GPIO initialisieren
gpio.setmode(gpio.BCM)
gpio.setup(servopin, gpio.OUT)

# PWM-Frequenz auf 50 Hz setzen
servo = gpio.PWM(servopin, 50)

# PWM starten, Servo auf 0 Grad
servo.start(2.5) #2.5

# Umrechnung Grad in Tastverhaeltnis
def setservo(winkel):
  if winkel < 0:
    winkel = 0
  elif winkel > 180:
    winkel = 180
  pwm = winkel/18 + 2.5
  servo.ChangeDutyCycle(pwm)

try:
  # Endlosschleife Servoansteuerung
  while True:
    winkel = raw_input("Winkel eingeben (0 - 180): ")
    winkel = float(winkel)
    setservo(winkel)

except KeyboardInterrupt:
  # Abbruch mit [Strg][C],
  # Servo auf 0 Grad, PWM beenden
  servo.ChangeDutyCycle(2.5)
  servo.stop()
  gpio.cleanup()
