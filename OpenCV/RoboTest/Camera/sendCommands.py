import serial
import time
import json
ser = serial.Serial('/dev/ttyUSB0',38400,timeout=1)
ser.flush()
message = {"s" : "1", "m" : "0", "a" : "45", "d" : "12", "t" : "-150"}


def sendMessage(messageTemp):
    line = ""
    while line != "Good":
        mess = json.dumps(messageTemp) + "\n"
        ser.write(mess.encode('UTF-8'))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(0.1)
    return 1;

def testSend(status, angle, direction,stop,tick):
    message["s"] = status
    message["a"] = angle
    message["d"] = direction
    message["m"] = stop
    message["t"] = tick
    sendMessage(message)
    
def check():
    testSend("4","180","12","1","-100")
    time.sleep(3)
    testSend("1","-180","12","0","0")
    time.sleep(0.45)
    testSend("1","-180","12","1","0")
    time.sleep(0.5)
    testSend("1","-90","12","0","0")
    time.sleep(1.7)
    testSend("1","-90","12","1","0")
    time.sleep(0.5)
    testSend("2","-90","12","0","0")
    time.sleep(3)
    testSend("1","0","12","0","0")
    time.sleep(0.7)
    testSend("1","0","12","1","0")
    time.sleep(0.5)
    testSend("1","-90","12","1","0")
    time.sleep(0.5)
    testSend("4","180","12","0","0")
    time.sleep(4)
    testSend("4","90","12","0","-300")
    time.sleep(4)
    testSend("1","-180","12","0","0")
    time.sleep(0.5)
    testSend("1","-180","12","1","0")
    time.sleep(0.5)
    testSend("2","0","12","0","0")
    time.sleep(4)
    testSend("1","0","12","0","0")
    time.sleep(6)
    testSend("1","0","12","1","0")
    time.sleep(0.5)
    testSend("4","90","12","1","-300")
    time.sleep(5)
    testSend("1","-180","12","0","0")
    time.sleep(4)
    testSend("1","-180","12","1","0")
check()
