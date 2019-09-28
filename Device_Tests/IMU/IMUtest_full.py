
import smbus
import math
import os
import csv
import time 
import numpy as np
from time import sleep
from datetime import datetime


iteration = 1200 #number of logged measurse
InitData = [] 

#--------------------
#--- LOGGER-Part ----
#--------------------
def init_folder():
 # csv-files saved in own folder
 #from: https://www.tutorialspoint.com/python/python_files_io.htm
 try:
  os.chdir("csvData")
 except:
  os.mkdir("csvData")
  os.chdir("csvData")


def init_file(sensorName):

        praefix=raw_input("Richtung angeben: ")
#name of file is current date and time 
        #print "entrance file_setting"
	fileName = str(sensorName)+'_'+str(time.strftime("%y%m%d"))+'_'+str(time.strftime("%H%M%S"))+'_'+str(praefix)+'.csv'
	print ("fileName: ",fileName)
	return fileName

def init_table(col2,col3):
#from: https://www.ilsb.tuwien.ac.at/~pahr/317.530/12.html
	f = csv.writer(open(str(fileName), "w"))
	#print "file written"
	f.writerow(['Time',str(col2), str(col3)
    ])
	


#------------------
#--- IMU-Part -----
#------------------

# Register
power_mgmt_1  = 0x6b 
power_mgmt_2  = 0x6c
ACCEL_CONFIG  = 0x1C #Reg 28 
ACCEL_CONFIG2 = 0x1D #Reg 29



def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(2) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect
 


# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0) #full power mode
#bus.write_byte_data(address, power_mgmt_2, 0b00001111)  #disabele=1, disabled accel_z, gyro_x bis _z
#setzt Accelerometer Full Scale Select (hier auf +-2g)
bus.write_byte_data(address, ACCEL_CONFIG , 0b00100000)
#setzt den Tiefpass-Filter
bus.write_byte_data(address, ACCEL_CONFIG2 , 0b00000100) #entspricht dem Wert 4, also 19,8 ms ~50Hz

#init logger
print ("init folder")
init_folder()
#init csv file
fileName = init_file("IMU")
file = csv.writer(open(str(fileName), "w"))
#init_table('a_x','a_y')

print
print ("Beschleunigungssensor")
print ("---------------------")

for j in range(iteration):
    
 
 beschleunigung_xout = read_word_2c(0x3b)
 beschleunigung_yout = read_word_2c(0x3d)
 #beschleunigung_zout = read_word_2c(0x3f)
 
 
 beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0 #wo kommt die Zahl her???
 beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
 #beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
 
 print ("beschleunigung_xout: ", ("%6d" % beschleunigung_xout), " skaliert: ", beschleunigung_xout_skaliert)
 print ("beschleunigung_yout: ", ("%6d" % beschleunigung_yout), " skaliert: ", beschleunigung_yout_skaliert)
 #print ("beschleunigung_zout: ", ("%6d" % beschleunigung_zout), " skaliert: ", beschleunigung_zout_skaliert)
 
 #print ("X, Y skaliert: " , beschleunigung_xout_skaliert, beschleunigung_yout_skaliert)
 #print ("Y Rotation: " , beschleunigung_xout_skaliert, beschleunigung_yout_skaliert)

 now = int(round(time.time() * 1000)) #datetime.strptime('%S.%f') #datetime.now(%s)
 InitData.append((now, beschleunigung_xout, beschleunigung_yout))
 # = str(data)

print("init data= ",InitData)

npdata = np.array(InitData)

np.savetxt(fileName,npdata,delimiter=",")

file = csv.writer(open(str(fileName), "w"))
file.writerows(InitData)
InitData[j]=(str(now),str(beschleunigung_xout_skaliert),str(beschleunigung_yout_skaliert))

median = np.median(npdata, axis=0)
print("median= ", median)

offset_X = median[1]
offset_Y = median[2]

print("offset_X = ", offset_X)
print("offset_Y = ", offset_Y)
