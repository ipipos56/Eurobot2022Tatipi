import serial
import time
import json
ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
ser.flush()

jsonTestSend = {"status" : "1", "stop": "0", "message" : "Good", "angle" : "0"}

line = ""

while line != "Good":
	mess = json.dumps(jsonTestSend) + "\n"
	ser.write(mess.encode('UTF-8'))
	line = ser.readline().decode('utf-8').rstrip()
	print(line)
	time.sleep(0.1)
