import serial
import time
import json
ser = serial.Serial('/dev/ttyACM0',57600,timeout=1)
ser.flush()

jsonTestSend = [{},{}]
jsonTestSend[0] = {"status" : "1", "stop": "0", "message" : "Good", "angle" : "-90"}
jsonTestSend[1] = {"status" : "1", "stop" : "0", "message" : "Good", "angle" : "0"}

line = ""

for i in range(0,2):
	line = ""
	time.sleep(1)
	while line != "Good":
		mess = json.dumps(jsonTestSend[i]) + "\n"
		ser.write(mess.encode('UTF-8'))
		line = ser.readline().decode('utf-8').rstrip()
		print(line)
		time.sleep(0.1)
