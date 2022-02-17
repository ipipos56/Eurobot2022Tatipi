# import the necessary packages
import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
import math

SCALE_COEFFICIENT = 1
CENTER = (0,0)

dst_path = "map.jpg"

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

#Default
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
                default="DICT_4X4_100",
                help="type of ArUCo tag to detect")
args = vars(ap.parse_args())
# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
ARUCO_DICT.get(args["type"], None)
# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()


def perspectiveTransform(src, saved):
    drawing = False  # true if mouse is pressed
    src_x, src_y = -1, -1
    dst_x, dst_y = -1, -1

    src_list = []
    dst_list = []
    H = 0
    mask = 0
    image = 0

    # mouse callback function
    def select_points_src(event, x, y, flags, param):
        nonlocal src_x, src_y, drawing
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            src_x, src_y = x, y
            cv2.circle(src_copy, (x, y), 5, (0, 0, 255), -1)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False

    # mouse callback function
    def select_points_dst(event, x, y, flags, param):
        nonlocal dst_x, dst_y, drawing
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            dst_x, dst_y = x, y
            cv2.circle(dst_copy, (x, y), 5, (0, 0, 255), -1)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False

    def get_plan_view(src, dst, H=None):
        src_pts = np.array(src_list).reshape(-1, 1, 2)
        dst_pts = np.array(dst_list).reshape(-1, 1, 2)
        mask=0
        if H == None:
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        print("H:")
        print(H)
        plan_view = cv2.warpPerspective(src, H, (dst.shape[1], dst.shape[0]))

        return plan_view, H, mask

    def merge_views(src, dst, H=None):
        plan_view, H, mask = get_plan_view(src, dst, H)

        for i in range(0, dst.shape[0]):
            for j in range(0, dst.shape[1]):
                if (plan_view.item(i, j, 0) == 0 and
                        plan_view.item(i, j, 1) == 0 and
                        plan_view.item(i, j, 2) == 0):
                    plan_view.itemset((i, j, 0), dst.item(i, j, 0))
                    plan_view.itemset((i, j, 1), dst.item(i, j, 1))
                    plan_view.itemset((i, j, 2), dst.item(i, j, 2))
        return plan_view, H, mask

    if not saved:
        src_copy = src.copy()
        cv2.namedWindow('src')
        cv2.moveWindow("src", 80, 80)
        cv2.setMouseCallback('src', select_points_src)

        dst = cv2.imread(dst_path, -1)
        dst_copy = dst.copy()
        cv2.namedWindow('dst')
        cv2.moveWindow("dst", 780, 80)
        cv2.setMouseCallback('dst', select_points_dst)

        while (1):
            cv2.imshow('src', src_copy)
            cv2.imshow('dst', dst_copy)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('s'):
                print('save points')
                cv2.circle(src_copy, (src_x, src_y), 5, (0, 255, 0), -1)
                cv2.circle(dst_copy, (dst_x, dst_y), 5, (0, 255, 0), -1)
                src_list.append([src_x, src_y])
                dst_list.append([dst_x, dst_y])
                print("src points:")
                print(src_list)
                print("dst points:")
                print(dst_list)
            elif k == ord('h'):
                print('create plan view')
                plan_view, H, mask = get_plan_view(src, dst)
                cv2.imshow("plan view", plan_view)
            elif k == ord('m'):
                print('merge views')
                merge, H, mask = merge_views(src, dst)
                cv2.imshow("merge", merge)
                image = merge
            elif k == ord('q'):
                break
        cv2.destroyWindow("src")
        cv2.destroyWindow("dst")
        cv2.destroyWindow("plan view")
        cv2.destroyWindow("merge")

        save_file(H, mask)
        cv2.imwrite('merge.jpg', image)

    if saved:
        H, mask = load_file()
        dst = cv2.imread(dst_path, -1)
        merge, H, mask = merge_views(src, dst, H)
        image = merge


    return image, H, mask

def save_file(np_array, np_array2, filename="data.txt"):
    with open(filename, 'wb') as f:
        np.savetxt(f, np_array)
        np.savetxt(f, np_array2)

def load_file(filename="data.txt"):
    with open(filename, 'wb') as f:
        a = np.loadtxt(f)
        b = np.loadtxt(f)
    return a, b

def undistort(src):
    width  = src.shape[1]
    height = src.shape[0]

    distCoeff = np.zeros((4,1),np.float64)

    # TODO: add your coefficients here!
    k1 = -2e-5 + 3e-6 # negative to remove barrel distortion
    k2 = 0.0
    p1 = 0.0
    p2 = 0.0

    distCoeff[0,0] = k1
    distCoeff[1,0] = k2
    distCoeff[2,0] = p1
    distCoeff[3,0] = p2

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

def ReadAndTransformCamera(videoStream):
    frame = videoStream.read()
    # frame = imutils.resize(frame, width=1000)
    frame = undistort(frame)
    frame = unsharp_mask(frame)
    # cv2.imshow("frame",frame)
    # cv2.waitKey(0)
    return frame

def Callibration():
    print("[INFO] starting callibration...")
    vs = VideoStream(src=0).start()
    time.sleep(2.0)

    perspectiveTransform(ReadAndTransformCamera(vs), False)

def MapCoordinates():
    radius_of_sample = 140

    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()
    time.sleep(2.0)

    while True:
        frame, H, mask = perspectiveTransform(ReadAndTransformCamera(vs), True)

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
            arucoDict, parameters=arucoParams)
                  # verify *at least* one ArUco marker was detected

        center = (0,0)
        coordinates = {}

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

                print(markerID)

                coordinates[markerID] = ((cX - CENTER[0])*SCALE_COEFFICIENT, (cY - CENTER[1])*SCALE_COEFFICIENT)

                cLeftLineY = (topLeft[1] + bottomLeft[1]) / 2.0
                cRightLineY = (topRight[1] + bottomRight[1]) / 2.0

                cLeftLineX = (topLeft[0] + bottomLeft[0]) / 2.0
                cRightLineX = (topRight[0] + bottomRight[0]) / 2.0

                radius_of_sample = int((cRightLineX - cLeftLineX) / 1.25)

                centerRight = (int(cRightLineX), int(cRightLineY))
                centerLeft = (int(cLeftLineX), int(cLeftLineY))
                cv2.line(frame, centerLeft, centerRight, (0, 255, 0), 2)

                differenceLeft = [0,0]
                differenceLeft[0] = (cX - centerLeft[0]) * 1.5
                differenceLeft[1] = (cY - centerLeft[1]) * 1.5

                differenceRight = [0,0]
                differenceRight[0] = (centerRight[0] - cX) * 1.5
                differenceRight[1] = (centerRight[1] - cY) * 1.5

                cv2.line(frame, (int(cRightLineX + differenceRight[0]), int(cRightLineY + differenceRight[1])), centerRight, (0, 255, 0), 2)
                cv2.line(frame, (int(cLeftLineX - differenceLeft[0]), int(cLeftLineY - differenceLeft[1])), centerLeft, (0, 255, 0), 2)

                #cv2.line(frame, (topRight[0] + radius_of_sample, topRight[1]), (bottomRight[0] + radius_of_sample, bottomRight[1]), (0, 255, 0), 2)
                #cv2.line(frame, (bottomLeft[0] - radius_of_sample, bottomLeft[1]), (topLeft[0] - radius_of_sample, topLeft[1]), (0, 255, 0), 2)

                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        # show the output frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    return coordinates


Callibration()
MapCoordinates()