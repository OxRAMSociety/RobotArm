# Get the camera parameters using calibration on the CV chessboard (and save!)
import numpy as np
import cv2
import pickle

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points (3d point in real world space)
# like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)


# Load video stream
vs = cv2.VideoCapture(0)

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:
    # Run image from stream and prepare
    active, img = vs.read()
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))
    img = cv2.GaussianBlur(img, (3, 3), 0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        
        # Refined image points (2d points in image plane) from the corners
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints = corners2.copy()

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)

        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objp], [imgpoints], gray.shape[::-1],None,None)
        # Save camera data (matrix and distortion coeffs)
        with open('./camera_data.pkl', 'wb') as file:
            pickle.dump([mtx, dist], file)

        # Get optimal camera matrix for chessboard region
        h, w, c = img.shape
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        # Undistort and crop
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imshow('Undistort', dst)

    cv2.imshow('img',img)
    cv2.waitKey(1)

cv2.destroyAllWindows()