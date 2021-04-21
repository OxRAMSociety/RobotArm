# Detect the position of aruco markers in a video stream
# Cv2 gets picky here and needs ONLY python-contrib-opencv, not the usual, so I 
# installed that in a separate environemnt. Contact me if help needed.
import numpy as np
import cv2

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

# Load  video stream
vs = cv2.VideoCapture(0)

# Define used aruco markers
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
arucoParams = cv2.aruco.DetectorParameters_create()

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:

    # Run image from stream through model and display
    active, frame = vs.read()

    # Use cv2 to detect aruco marker
    cv2_aruco_output = cv2.aruco.detectMarkers(frame, arucoDict,
        parameters=arucoParams)
    (corners, ids, rejected) = cv2_aruco_output

    # Plot markers
    centres = plot_markers(cv2_aruco_output, frame)

    # Join the centres
    if len(centres) > 3:
        hull = cv2.convexHull(np.array(centres))
        cv2.drawContours(frame, [hull], 0, color=0, thickness=10)

    # Show image
    cv2.imshow("Webcam Feed", frame)
    cv2.waitKey(20)