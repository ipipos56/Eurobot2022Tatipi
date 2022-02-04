from imutils.video import VideoStream
import time
import cv2
import numpy as np
import math

vs = VideoStream(src=0).start()
time.sleep(2.0)

def detectShape(c):          #Function to determine type of polygon on basis of number of sides
       shape = 'unknown' 
       peri=cv2.arcLength(cnt,True) 
       vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
       sides = len(vertices)
       if sides==6:
            sideCount = 0
            i = 0
            for sideCheck in vertices:
                nextSide = (i + 1) % 6
                side = math.sqrt(((vertices[i,0,0] - vertices[nextSide,0,0]) ** 2) + ((vertices[i,0,1] - vertices[nextSide,0,1]) ** 2))
                if side > 30.0:
                    sideCount += 1
            if sideCount >= 6:
                shape='hexagon'
       return shape

while(True):
    
    img = vs.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, 170, 255)            #Determine edges of objects in an image
    ret,thresh = cv2.threshold(gray,240,255,cv2.THRESH_BINARY)  
    (imgTest,contours,h) = cv2.findContours(edged,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #Find contours in an image
    for cnt in contours:
        moment=cv2.moments(cnt)
        if moment['m00'] != 0:
            cx = int(moment['m10'] / moment['m00']) 
            cy = int(moment['m01'] / moment['m00']) 
            shape=detectShape(cnt)
            if shape == 'hexagon':
                cv2.drawContours(img,[cnt],-1,(0,255,0),2)
                cv2.putText(img,shape,(cx,cy),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)  #Putting name of polygon along with the shape 
    cv2.imshow('polygons_detected',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

cv2.destroyAllWindows()
