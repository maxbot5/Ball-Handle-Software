# from: https://www.instructables.com/id/Raspberry-Pi-Data-Logging/

import os
import csv
import time 
from time import sleep
from datetime import datetime

iteration = 100


def init_folder():
 # csv-files saved in own folder
 #from: https://www.tutorialspoint.com/python/python_files_io.htm
 try:
  os.chdir("csvData")
 except:
  os.mkdir("csvData")
  os.chdir("csvData")


def file_setting(sensorName):

#name of file is current date and time 
        print "entrance file_setting"
	fileName = str(sensorName)+'_'+str(time.strftime("%y%m%d%H%M%S"))+'.csv'
	print "fileName: ",fileName
	#from: https://www.ilsb.tuwien.ac.at/~pahr/317.530/12.html
	f = csv.writer(open(str(fileName), "w"))
	print "file written"
	f.writerow(['Time','Data'])
	return fileName

#main 
print "init folder"
init_folder()
#init csv file
fileName = file_setting("IMU")
file = csv.writer(open(str(fileName), "w"))
i=0
#main loop
print "ready for loop"
for j in range(iteration):
  i=i+1
  now = datetime.now()
  file.writerow([str(now),str(i*i/2)])
  print "current Value: ",i 




