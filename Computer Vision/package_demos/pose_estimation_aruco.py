# Implement the pose-detection demo from the open-cv website
import cv2
import numpy as np
import pickle

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



def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)

    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)

    return img

# Load previously saved data
with open('./camera_data.pkl', 'rb') as file:
    mtx, dist = pickle.load(file)

# Define Aruco markers being used
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters_create()

# Define dictionary to take aruco key to point in relative space
aruco_pos = {
    4: [0, 0, 0],
    5: [1, 0, 0],
    6: [2, 0, 0],
    9: [3, 0, 0],
    1: [0, 1, 0],
    11: [0, 2, 0],
    12: [0, 3, 0],
    7: [1, 3, 0],
    0: [2, 3, 0],
    10: [3, 3, 0],
    8: [3, 2, 0],
    2: [3, 1, 0]
}

# Define points for drawing object (points on cube vertexes)
axis = np.float32(
    [[0,0,0], [0,3,0], [3,3,0], [3,0,0],
    [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3]]
)

# Load video stream
vs = cv2.VideoCapture(0)

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:
    # Run image from stream through model and display
    active, img = vs.read()
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))
    img = cv2.GaussianBlur(img, (3, 3), 0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the aruco markers
    cv2_aruco_output = cv2.aruco.detectMarkers(img, arucoDict,
        parameters=arucoParams)
    (corners, ids, rejected) = cv2_aruco_output
    centers = plot_markers(cv2_aruco_output, img)

    if ids is not None and len(ids) > 10:
        print("found them all")
        # Get the centres
        centers = np.array([[list(c)] for c in centers], dtype=np.float32)
        # Get the corresponding aruco-code space points
        objp = np.array([aruco_pos[int(id_num)] for id_num in ids], dtype=np.float32)

        # Find the rotation and translation vectors.
        print("Find rotation and translation")
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, centers, mtx, dist)

        # project 3D points to image plane
        print("Project Points")
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        # Draw image
        print("Draw image") 
        print(imgpts)
        img = draw(img, centers, imgpts)
        # k = cv2.waitKey(0) & 0xff

    cv2.imshow('img',img)
    cv2.waitKey(1)

cv2.destroyAllWindows()

#TODO; Use all the corners of the aruco markers as points for the solver (as we can assume each marker is flat, and the corners are easy to calculate!)