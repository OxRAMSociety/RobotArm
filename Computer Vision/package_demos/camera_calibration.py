# Get the camera parameters using calibration on the CV chessboard (and save!)
import numpy as np
import cv2
import pickle

# termination criteria
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW#+cv2.fisheye.CALIB_CHECK_COND

# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points (3d point in real world space)
# like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
CHECKERBOARD = (6, 9)
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

image_points = []
board_points = []

# Load video stream
vs = cv2.VideoCapture(0)

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:
    # Run image from stream and prepare
    active, img = vs.read()
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If found, add object points, image points (after refining them)
    if ret == True:
        
        corners2 = cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
        board_points.append(objp)
        image_points.append(corners2.copy())

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

    cv2.imshow('Camera Image',img)
    k = cv2.waitKey(10)
    if k & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break

# Calibrate camera
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(board_points, image_points, gray.shape[::-1],None,None)

# Calibrate camera
N_OK = len(board_points)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        board_points,
        image_points,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(img.shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")

# Save camera data (matrix and distortion coeffs)
with open('./camera_data.pkl', 'wb') as file:
    pickle.dump([K, D], file)

# # Get optimal camera matrix for chessboard region
# h, w, c = img.shape
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# # Undistort and crop
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
# x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# cv2.imshow('Undistort', dst)
