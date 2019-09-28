import RPi.GPIO as GPIO
import time

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 als PWM mit 50Hz
p.start(7.5) # Initialisierung
try:
  while True:
    
    p.ChangeDutyCycle(5)
    time.sleep(1)
   
    p.ChangeDutyCycle(10)
    time.sleep(1)
    """
    p.ChangeDutyCycle(9)
    time.sleep(1)
    p.ChangeDutyCycle(10.5)
    time.sleep(1)
    p.ChangeDutyCycle(9)
    time.sleep(1)
    p.ChangeDutyCycle(7.5)
    time.sleep(1)
    p.ChangeDutyCycle(5)
    time.sleep(1)
    p.ChangeDutyCycle(4.5)
    time.sleep(1)
    """
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()
