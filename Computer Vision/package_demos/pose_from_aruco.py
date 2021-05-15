# Get extrinsic position from aruco marker
import cv2
import pickle
import numpy as np

def plot_markers(cv2_aruco_output, screen, VERBOSE=False):

    # Detector data
    (corners, ids, _) = cv2_aruco_output
    
    centres = []
    # If there are valid points
    if len(corners) > 0:
        for (markerCorner, markerID) in zip(corners, ids.flatten()):
            # extract the marker corners 
            # (top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(screen, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(screen, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(screen, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(screen, bottomLeft, topLeft, (0, 255, 0), 2)
            # Put a little circle on the top left
            cv2.circle(screen, topLeft, 8, (0, 255, 0), -1)
            # compute and draw the center coordinates of the ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(screen, (cX, cY), 4, (0, 0, 255), -1)
            centres.append((cX, cY))
            # draw the ArUco marker ID on the screen
            cv2.putText(screen, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                2, (0, 255, 0), 5)
            if VERBOSE: print("[INFO] ArUco marker ID: {}".format(markerID))
    
    return centres

def clean_angles(angle):
    # angle = (angle / (2 * np.pi)) * 360
    angle = round(angle, 2)
    return angle

# Load previously saved data
with open('./camera_data.pkl', 'rb') as file:
    mtx, dist = pickle.load(file)

# Define Aruco markers being used
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters_create()

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
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, gray.shape[:2][::-1], cv2.CV_16SC2)
    img = cv2.remap(gray, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    #### Do aruco stuff

    # Get the aruco marker position
    cv2_aruco_output = cv2.aruco.detectMarkers(img, arucoDict,
        parameters=arucoParams)
    (corners, ids, rejected) = cv2_aruco_output

    plot_markers(cv2_aruco_output, img)

    # If there were any detected
    if np.all(ids != None):
        # POSE ESTIMATION
        rvec, trans,_ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, mtx, dist) 
        # # Convert to rotation matrix
        # rot = cv2.Rodrigues(rvec)
        # x, y, z = trans[0][0]
        # rho, theta, phi = list(map(clean_angles, rvec[0][0]))
        # print(f"x: {round(x,2)}, y: {round(y,2)}, z: {round(z,2)}")
        # print(f"rho: {rho}, theta: {theta}, phi: {phi}")
        # print("")
        for r, t in zip(rvec, trans):
            img = cv2.aruco.drawAxis(img, mtx, dist, r, t, length=0.2)
    ###

    cv2.imshow('img',img)
    cv2.waitKey(1)
