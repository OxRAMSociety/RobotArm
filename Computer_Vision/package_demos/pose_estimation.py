# Implement the pose-detection demo from the open-cv website
import cv2
import numpy as np
import pickle

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

# Define search critera for chessboard, and the points the 
# chessboard corners correspond to in it's relative space
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Define lines for drawing object (commented out axes lines)
# axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
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

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        print(len(corners2))
        # Find the rotation and translation vectors.
        print("Find rotation and translation")
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        # project 3D points to image plane
        print("Project Points")
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        # Draw image
        print("Draw image") 
        img = draw(img,corners2,imgpts)
        # k = cv2.waitKey(0) & 0xff

    cv2.imshow('img',img)
    cv2.waitKey(1)

cv2.destroyAllWindows()