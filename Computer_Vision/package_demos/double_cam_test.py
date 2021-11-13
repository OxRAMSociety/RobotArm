# Try running two camera feeds
import cv2

def prep_image(frame):
    h, w, _ = frame.shape
    frame = cv2.resize(frame, (int(w/2), int(h/2)))
    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    return frame

# Load video stream
vs1 = cv2.VideoCapture(0)
vs2 = cv2.VideoCapture(1)

# While there is an active stream (can also test vs.isOpened() until ready)
active = True 
while active:

    # Grab the two feeds
    active, img1 = vs1.read()
    active, img2 = vs2.read()

    # Prepare for showing
    frame1 = prep_image(img1)
    frame2 = prep_image(img2)
    
    # Show
    cv2.imshow('Cam1', frame1)
    cv2.imshow('Cam2', frame2)
    cv2.waitKey(1)
