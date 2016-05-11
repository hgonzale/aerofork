import socket
import sys
from datetime import datetime

#HOST, PORT = "192.168.95.219", 27015
HOST, PORT = "localhost", 27015

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((HOST, PORT))
# for x in range(0, 1000):
#     print("Step 1")
#     s.send(b'Hello'+str(x)+"\0")
#     print("Step 2")
#     print(str(s.recv(4096)))
#     print(x)
now = datetime.now()
i=0
while i<1000:
	temp = str(s.recv(512))
	temp = temp.split("\n")[0]
	print temp
	#print datetime.now()-now
	now = datetime.now()
