# Get extrinsic position from aruco marker
import cv2
import torch
import pickle
import numpy as np
from PIL import Image
from threading import Thread


def get_boxes(frame):
    global box_data, ACTIVE_THREADS
    ACTIVE_THREADS += 1
    # Get prediction bounding boxes
    results = model(frame, size=320)
    box_data = results.xyxy
    ACTIVE_THREADS -= 1


def draw_boxes(frame, box, line_thickness=4, color=(255, 0, 0), x_offset=0):
    if box[4] > CONFIDENCE_THRESHOLD:
        # Box    
        c1 = (int(box[0]), int(box[1]))
        c2 = (int(box[2]), int(box[3]))
        cv2.rectangle(
            frame, c1, c2, color, 
            thickness = line_thickness,
            lineType = cv2.LINE_AA
        )
        # Dot in centre
        cv2.circle(
            frame, (int((c2[0]+c1[0])/2), int((c2[1]+c1[1])/2)), 
            radius = 2, 
            color = color, 
            thickness = line_thickness,
            lineType = cv2.LINE_AA
        )
        # Label
        label = f"{class_names[int(box[5])]}: {box[4]:.2f}"
        font_thickness = max(line_thickness - 1, 1)
        t_size = cv2.getTextSize(
            label, 0, 
            fontScale=line_thickness / 3, 
            thickness=font_thickness)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(
            frame, c1, c2, color, 
            thickness = -1, 
            lineType = cv2.LINE_AA)
        cv2.putText(
            frame, label, (c1[0], c1[1] - 2), 0, 
            line_thickness / 3, [225, 255, 255],
            thickness = font_thickness, 
            lineType=cv2.LINE_AA
        )


THREAD_LIMIT = 5
ACTIVE_THREADS = 0
CONFIDENCE_THRESHOLD = 0.4
box_data = None

# Load previously saved data
with open('./camera_data.pkl', 'rb') as file:
    mtx, dist = pickle.load(file)


# Load best trained model
best_trained = "/Users/jamesashford/Documents/Projects/Programming/Python/YOLO_chesspieces/yolov5/runs/train/wobblepieces-withboard-small-100/weights/best.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model=best_trained)
class_names = model.names

# Define Aruco markers being used
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters_create()

# Load video stream
vs = cv2.VideoCapture(0)

active = True
while active:
    print(f"Active Threads: {ACTIVE_THREADS}", end="\r")
    # Run image from stream through model and display
    active, img = vs.read()
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))
    img = cv2.GaussianBlur(img, (5, 5), 0)
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Unidistort the image
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, img.shape[:2][::-1], cv2.CV_16SC2)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Resize incoming frame to square by cropping edges
    offset = int((img.shape[1] - img.shape[0])/2)
    # frame = img[:, offset:-offset]
    frame = img

    # Get the image results asyncronously
    if ACTIVE_THREADS < THREAD_LIMIT:
        Thread(target=get_boxes, args=(frame, )).start()

    # Draw the boxes onto the image
    if box_data and box_data[0].numel() > 0:
        for box_tensor in box_data[0]:
            draw_boxes(img, box_tensor.numpy(), x_offset=offset)

    #### Do aruco stuff
    (corners, ids, rejected)  = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
    if np.all(ids != None):
        # Pose estimation from marker
        rvec, trans, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, mtx, dist) 
        # Plot the axes
        for r, t in zip(rvec, trans):
            img = cv2.aruco.drawAxis(img, mtx, dist, r, t, length=0.2)
    ###

    cv2.imshow('img',img)
    cv2.waitKey(1)


# TODO; Add key to capture frames, and triangulate positions from them