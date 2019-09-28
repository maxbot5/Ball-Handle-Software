# from: https://www.instructables.com/id/Raspberry-Pi-Data-Logging/

import os
import time 
from time import sleep
from datetime import datetime


def file_setting(sensorName, iteration):

#name of file is current date and time 
	id = time.strftime("%y%m%d%H%M%S")
	fileName = str(sensorName)+'_'+str(id)+'.csv'

	file = open(str(fileName), "r+")
	i=0
	if os.stat(+str(fileName)).st_size == 0:
	        file.write("Time,"+str( SensorName)+"\n")

#main 
while True:	
 i=0
 file_setting("IMU",int(100.0))
 i=i+1
 now = datetime.now()
 file.write(str(now)+","+str(i)+"\n")
 file.flush()
 time.sleep(5)<br>file.close()



