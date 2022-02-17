# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
import math
import numpy as np

radius_of_sample = 140


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
    default="DICT_4X4_100",
    help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
ARUCO_DICT.get(args["type"], None)
# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = cv2.VideoCapture(0)
vs.set(3, 1920)
vs.set(4, 1080)

def undistort(src):
  width  = src.shape[1]
  height = src.shape[0]

  distCoeff = np.zeros((4,1),np.float64)

  # TODO: add your coefficients here!
  k1 = -2e-5; # negative to remove barrel distortion
  k2 = 0.0;
  p1 = 0.0;
  p2 = 0.0;

  distCoeff[0,0] = k1;
  distCoeff[1,0] = k2;
  distCoeff[2,0] = p1;
  distCoeff[3,0] = p2;

  # assume unit matrix for camera
  cam = np.eye(3,dtype=np.float32)

  cam[0,2] = width/2.0  # define center x
  cam[1,2] = height/2.0 # define center y
  cam[0,0] = 10.        # define focal length x
  cam[1,1] = 10.        # define focal length y

  # here the undistortion will be computed
  dst = cv2.undistort(src,cam,distCoeff)
  return dst

def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=2.0, threshold=0):
    """Return a sharpened version of the image, using an unsharp mask."""
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

checkList = 0
info = ""
checkIds = [36,13,47,42,1,17]

# loop over the frames from the video stream
while True:
    checkList = 0
    info = ""
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 1000 pixels
    _, frame = vs.read()
    initialFrame = frame
    
    # frame = imutils.resize(frame, width=1000)
    # frame = unsharp_mask(frame)
    frame=undistort(frame)
    
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
        arucoDict, parameters=arucoParams)
              # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                      
            
            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            
            
            cLeftLineY = (topLeft[1] + bottomLeft[1]) / 2.0
            cRightLineY = (topRight[1] + bottomRight[1]) / 2.0
                        
            cLeftLineX = (topLeft[0] + bottomLeft[0]) / 2.0
            cRightLineX = (topRight[0] + bottomRight[0]) / 2.0
            
            radius_of_sample = int((cRightLineX - cLeftLineX) / 1.25)
            
            centerRight = (int(cRightLineX), int(cRightLineY))
            centerLeft = (int(cLeftLineX), int(cLeftLineY))
            #cv2.line(frame, centerLeft, centerRight, (0, 255, 0), 2)
            
            differenceLeft = [0,0]
            differenceLeft[0] = (cX - centerLeft[0]) * 1.5
            differenceLeft[1] = (cY - centerLeft[1]) * 1.5
            
            differenceRight = [0,0]
            differenceRight[0] = (centerRight[0] - cX) * 1.5
            differenceRight[1] = (centerRight[1] - cY) * 1.5
            
            #cv2.line(frame, (int(cRightLineX + differenceRight[0]), int(cRightLineY + differenceRight[1])), centerRight, (0, 255, 0), 2)
            #cv2.line(frame, (int(cLeftLineX - differenceLeft[0]), int(cLeftLineY - differenceLeft[1])), centerLeft, (0, 255, 0), 2)
            
            #cv2.line(frame, (topRight[0] + radius_of_sample, topRight[1]), (bottomRight[0] + radius_of_sample, bottomRight[1]), (0, 255, 0), 2)
            #cv2.line(frame, (bottomLeft[0] - radius_of_sample, bottomLeft[1]), (topLeft[0] - radius_of_sample, topLeft[1]), (0, 255, 0), 2)
                    
            # draw the ArUco marker ID on the frame
            cv2.putText(frame, str(markerID),
                (topLeft[0], topLeft[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            if(markerID in checkIds):
                checkList+=1
                if checkList != 1:
                    info += ", "
                info += str(markerID) + " " + str(cX) + " " + str(cY)
            
            
            
    # show the output frame
    number = 10
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    if checkList >= 6:
        cv2.imwrite("sampleInitial" + str(number) + ".jpg", initialFrame)
        cv2.imwrite("sampleWithIDs" + str(number) + ".jpg", frame)
        with open("sample" + str(number) + ".txt",'w') as f:
            f.write(info)
        break
# do a bit of cleanup
cv2.destroyAllWindows()
print("Done")
#vs.stop()