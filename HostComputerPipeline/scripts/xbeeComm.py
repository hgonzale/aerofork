import serial
import time
from datetime import datetime

ser = serial.Serial(port='COM3',baudrate=115200, timeout=3)
#s.write("hello")
i=0
deltas =[]
while i<100:
	now = datetime.now()
	ser.write(now.strftime("%H:%M:%S.%f")+"\n")
	ser.readline()
	now2 = datetime.now()
	delta=now2-now
	#print delta
	deltas.append(delta.microseconds)
	i=i+1
print sum(deltas)/float(len(deltas))
	# try:
	# 	print ser.read()
	# 	time.sleep(1)
	# except:
	# 	print('Data could not be read')
	# 	time.sleep(1)