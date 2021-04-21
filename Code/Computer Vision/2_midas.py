import cv2
import torch
import numpy as np
from threading import Thread


def get_model_prediction(sample_image):
    global overlay, ACTIVE_THREADS
    ACTIVE_THREADS += 1

    img = transform(sample_image).to(device)
    with torch.no_grad():
        pred = model(img)
        pred = torch.nn.functional.interpolate(
            pred.unsqueeze(1),
            size=sample_image.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    pred = pred.cpu().numpy()
    pred = ((pred / np.max(pred)) * 255).astype("uint8")
    pred = cv2.cvtColor(pred, cv2.COLOR_GRAY2RGB)
    
    # Make a red image
    # pred[:, :, :2] = 0

    # Apply colormap
    overlay = cv2.applyColorMap(overlay, cv2.COLORMAP_MAGMA)

    overlay = pred

    ACTIVE_THREADS -= 1


THREAD_LIMIT = 5
ACTIVE_THREADS = 0
SMALL = True

# Load MiDaS Model 
model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small" if SMALL else "MiDaS")
model.eval()
device = torch.device("cpu")
im_trans = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = im_trans.small_transform if SMALL else im_trans.default_transform

# Set up video stream
vs = cv2.VideoCapture(0)
active, frame = vs.read()
overlay = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

# While there is an active stream (can also test vs.isOpened() until ready)
while active:
    print(f"Active Threads: {ACTIVE_THREADS}", end="\r")
    # Run image from stream through model and display
    active, temp_frame = vs.read()
    frame = cv2.resize(temp_frame, (0, 0), fx=0.5, fy=0.5)

    # Get the image results asyncronously
    if ACTIVE_THREADS < THREAD_LIMIT:
        Thread(target=get_model_prediction, args=(frame, )).start()
    
    # Red Hellscape
    # show_img = frame ** overlay
    
    # Merge over the original frame
    show_img = cv2.addWeighted(frame, 1, overlay, 0.8, 0)
    
    # Mask the image
    # show_img = cv2.cvtColor(frame, cv2.COLOR_RGB2RGBA)
    # show_img[:, :, 3] = overlay[:, :, 2]
    # background = np.zeros(shape=show_img.shape).astype("uint8")
    # show_img = cv2.addWeighted(background, 0.4, show_img, 1, 0)
    # print(show_img.shape)

    # Show image
    cv2.imshow("Webcam Feed", show_img)
    cv2.waitKey(50)

