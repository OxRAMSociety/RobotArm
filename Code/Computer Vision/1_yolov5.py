import cv2
import torch
from threading import Thread


def get_boxes(frame):
    global box_data, ACTIVE_THREADS
    ACTIVE_THREADS += 1
    results = model(frame, size=320)
    box_data = results.xyxy
    ACTIVE_THREADS -= 1


def draw_boxes(frame, box, line_thickness=4, color=(255, 0, 0)):
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
box_data = None

# Load YOLOv5 Model and video stream
model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
vs = cv2.VideoCapture(0)
# Load COCO class names
with open("Code/Computer Vision/rsc/coco.names") as coco_file:
    class_names = coco_file.read().split("\n")
# model.classes = range(1, len(class_names))

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:
    print(f"Active Threads: {ACTIVE_THREADS}", end="\r")
    # Run image from stream through model and display
    active, frame = vs.read()
    

    # Get the image results asyncronously
    if ACTIVE_THREADS < THREAD_LIMIT:
        Thread(target=get_boxes, args=(frame, )).start()

    # Draw the boxes onto the image
    if box_data and box_data[0].numel() > 0:
        for box_tensor in box_data[0]:
            draw_boxes(frame, box_tensor.numpy())

    # Show image
    cv2.imshow("Webcam Feed", frame)
    cv2.waitKey(20)

