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


def draw_boxes(frame, box_data, line_thickness=4, color=(255, 0, 0)):
    centers = []
    for box_tensor in box_data[0]:
        box = box_tensor.numpy()
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
            center = (int((c2[0]+c1[0])/2), int((c2[1]+c1[1])/2))
            centers.append(center)
            cv2.circle(
                frame, center, 
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
    return centers

THREAD_LIMIT = 5
ACTIVE_THREADS = 0
CONFIDENCE_THRESHOLD = 0.5
box_data = None
frame1, frame2 = [], []

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
    active, frame = vs.read()
    h, w, c = frame.shape
    frame = cv2.resize(frame, (int(w/2), int(h/2)))
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Unidistort the image
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, frame.shape[:2][::-1], cv2.CV_16SC2)
    frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    show_frame = frame.copy()

    #### Do aruco stuff
    (corners, ids, rejected)  = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    if np.all(ids != None):
        # Pose estimation from marker
        rvec, trans, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, mtx, dist) 
        # Plot the axes
        for r, t in zip(rvec, trans):
            show_frame = cv2.aruco.drawAxis(show_frame, mtx, dist, r, t, length=0.2)
    ###

    # Resize incoming frame to square by cropping edges
    # offset = int((img.shape[1] - img.shape[0])/2)
    # frame = img[:, offset:-offset]
    # frame = img

    # Get the image results asyncronously
    if ACTIVE_THREADS < THREAD_LIMIT:
        Thread(target=get_boxes, args=(show_frame, )).start()

    # Draw the boxes onto the image
    if box_data and box_data[0].numel() > 0:
        current_centers = draw_boxes(show_frame, box_data)


    cv2.imshow('Annotated Camera View', show_frame)

    # Controls
    k = cv2.waitKey(10)
    if k & 0xFF == ord("q"):
        # If q: close
        cv2.destroyAllWindows()
        break
    # TODO; Add key to capture frames, 
    elif k & 0xFF == ord("c"):
        # If c: save new frame info
        output = [frame, rvec, trans, current_centers[0]]
        if not frame1:
            frame1 = output
            print("Frame1 assigned")
        elif not frame2:
            frame2 = output
            print("Frame2 assigned")

# Get projection matrixes
rot_difference = cv2.Rodrigues(frame2[1] - frame1[1])[0]
trans_difference = np.reshape(np.array(frame2[2] - frame1[2]), (3, 1))
proj1 = np.dot(mtx, np.hstack([np.eye(3), np.zeros(shape=(3,1))]))
proj2 = np.dot(mtx, np.hstack([rot_difference, trans_difference]))
print(proj1)
print(proj2)

# TODO; triangulate positions from these frames
points1 = np.array([frame1[3]], dtype=np.float32).T
points2 = np.array([frame2[3]], dtype=np.float32).T
points_4d = cv2.triangulatePoints(proj1, proj2, points1, points2)
homog_points = np.array(points_4d).reshape((1, 4))
points_3d = cv2.convertPointsFromHomogeneous(homog_points)
print(points_3d)