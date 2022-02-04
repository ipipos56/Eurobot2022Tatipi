import serial
import time
import json
ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
ser.flush()

jsonTestSend = {"status" : "1", "stop": "False", "message" : "Good", "points" : [{"x" : 97, "y" : 15},{"x" : 30, "y" : 80},{"x" : -32, "y" : 60}]}

line = ""

while line != "Good":
	mess = json.dumps(jsonTestSend) + "\n"
	ser.write(mess.encode())
	line = ser.readline().decode('utf-8').rstrip()
	print(line)
	time.sleep(0.1)
