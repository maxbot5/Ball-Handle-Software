import RPi.GPIO as GPIO
import time
 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
coil_A_1_pin = 4 # pink
coil_A_2_pin = 17 # orange
coil_B_1_pin = 23 # blau
coil_B_2_pin = 24 # gelb
#enable_pin   = 7 # Nur bei bestimmten Motoren benoetigt (+Zeile 24 und 30)
 
# anpassen, falls andere Sequenz


StepCount_cw = 4

#GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(coil_A_1_pin, GPIO.OUT)
GPIO.setup(coil_A_2_pin, GPIO.OUT)
GPIO.setup(coil_B_1_pin, GPIO.OUT)
GPIO.setup(coil_B_2_pin, GPIO.OUT)
 
#GPIO.output(enable_pin, 1)
 
def setStep(w1, w2, w3, w4):
    GPIO.output(coil_A_1_pin, w1)
    GPIO.output(coil_A_2_pin, w2)
    GPIO.output(coil_B_1_pin, w3)
    GPIO.output(coil_B_2_pin, w4)
 
def forward(delay, steps):
    i = 0
    setStep(0,0,0,0)
    Seq_cw = list(range(0, StepCount_cw))
    Seq_cw[0] = [1,0,0,0]
    Seq_cw[1] = [0,1,0,0]
    Seq_cw[2] = [0,0,1,0]
    Seq_cw[3] = [0,0,0,1]

    for i in range(steps):
        for j in range(StepCount_cw):
            setStep(Seq_cw[j][0], Seq_cw[j][1], Seq_cw[j][2], Seq_cw[j][3])
            time.sleep(delay)
    

    
if __name__ == '__main__':
    while True:
        setStep(0,0,0,0)
        delay = raw_input("Zeitverzoegerung (ms)?")
        steps = raw_input("Wie viele Schritte vorwaerts? ")
        forward(int(delay) / 1000.0, int(steps))
       
      
        
