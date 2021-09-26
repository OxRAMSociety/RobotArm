# Get extrinsic position from aruco marker
import cv2
import pickle
import numpy as np

UNDISTORT = True

# Load previously saved data
with open('camera_data.pkl', 'rb') as file:
    mtx, dist = pickle.load(file)


# Load video stream
vs = cv2.VideoCapture(0)

active = True
while active:
    # Get frame
    active, img = vs.read()
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Unidistort the image
    if UNDISTORT:
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, gray.shape[:2][::-1], cv2.CV_16SC2)
        img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow('img',img)
    k = cv2.waitKey(1)
    if k & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
    elif k & 0xFF == ord('d'):
        UNDISTORT = not UNDISTORT
