import cv2
import torch
from PIL import Image
from threading import Thread


def get_boxes(frame):
    global box_data, ACTIVE_THREADS
    ACTIVE_THREADS += 1
    # Get prediction bounding boxes
    results = model(frame, size=320)
    box_data = results.xyxy
    ACTIVE_THREADS -= 1


def draw_boxes(frame, box, line_thickness=4, color=(255, 0, 0)):
    if box[4] > CONFIDENCE_THRESHOLD:
        # Box    
        c1 = (int(box[0]), int(box[1]))
        c2 = (int(box[2]), int(box[3]))
        cv2.rectangle(
            frame, c1, c2, color, 
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

# Load best trained model
best_trained = "Computer Vision/rsc/best_trained_yolo.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model=best_trained)
class_names = model.names

# Start video capture
vs = cv2.VideoCapture(0)

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:
    print(f"Active Threads: {ACTIVE_THREADS}", end="\r")
    # Run image from stream through model and display
    active, frame = vs.read()
    # Resize incoming frame to square by cropping edges
    offset = int((frame.shape[1] - frame.shape[0])/2)
    frame = frame[:, offset:-offset]
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    
    # Get the image results asyncronously
    if ACTIVE_THREADS < THREAD_LIMIT:
        Thread(target=get_boxes, args=(frame, )).start()

    # Draw the boxes onto the image
    if box_data and box_data[0].numel() > 0:
        for box_tensor in box_data[0]:
            draw_boxes(frame, box_tensor.numpy())

    # Show image
    # frame = cv2.resize(frame, (160, 160))
    cv2.imshow("Webcam Feed", frame)
    cv2.waitKey(20)
