# detect my real chessboard (not B&W)
from einops import rearrange
from skimage.metrics import structural_similarity as ssim
from skimage.measure import label, regionprops
import numpy as np
import pickle
import time 
import cv2


def process_frame(frame, camera_param):
    h, w, c = frame.shape
    frame = cv2.resize(frame, (int(w/2), int(h/2)))
    frame = cv2.GaussianBlur(frame, (3,3), cv2.BORDER_DEFAULT)
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Flip image
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Unidistort the image
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_param[0], camera_param[1], np.eye(3), camera_param[0], frame.shape[:2][::-1], cv2.CV_16SC2)
    frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Get centre-image sectionc
    im_width = frame.shape[1]
    border = im_width//5
    frame = frame[:, border:im_width-border]

    return frame


def set_points(event, x, y, flags, param):
    global board_corners
    if event == cv2.EVENT_LBUTTONDOWN:
        # draw circle here (etc...)
        print(f'x = {x}, y = {y}')
        if not board_corners[0]:
            board_corners[0] = [x, y]
            print("Set Top Left")
        elif not board_corners[1]:
            board_corners[1] = [x, y]
            print("Set Top Right")
        elif not board_corners[2]:
            board_corners[2] = [x, y]
            print("Set Bottom Right")
        elif not board_corners[3]:
            board_corners[3] = [x, y]
            print("Set Bottom Left")
            print("All corners assigned!")
        else: 
            print("All corners have been assigned!")
        return True

def board_position(piece, board_spec):
    """Translate coordinate to boad location (i.e. A5) assuming A1 is bottom
    right of image being shown. board_spec must include corner locations and 
    steps along those lines (Ds)"""
    return True

MERGE_RANGE = 100
MOVE_FRAME_THRESHOLD = 5

background_frame = None
two_region_frame_count = 0

# Load previously saved data
with open('./camera_data.pkl', 'rb') as file:
    camera_parameters = pickle.load(file)

# Run video stream
vs = cv2.VideoCapture(2)
time.sleep(2)

# Take image and setup board outline with mouse
cv2.namedWindow('Board Setup')
cv2.setMouseCallback('Board Setup', set_points)
board_corners = np.array([None, None, None, None]) # tl, tr, br, bl
_, img = vs.read()
while not all(board_corners):

    # Draw board so far
    frame = process_frame(img, camera_parameters)
    assigned_points = np.array([p for p in board_corners if p is not None])
    if len(assigned_points) > 0:
        frame = cv2.polylines(frame, [assigned_points], True, (255, 0, 0), 2)

    # Show frame
    cv2.imshow('Board Setup', frame)    

    # Handle exit
    k = cv2.waitKey(10)
    if k & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
cv2.destroyAllWindows()

# Finalise board corners
board_corners = np.array([p for p in board_corners if p is not None])

# Calculate rest of board spec
board_spec = {
    "TL" : board_corners[0],
    "TR" : board_corners[1],
    "BR" : board_corners[2],
    "BL" : board_corners[3]
}
board_spec["D_T"] = (board_spec["TL"] - board_spec["TR"])/8
board_spec["D_L"] = (board_spec["TL"] - board_spec["BL"])/8
board_spec["D_R"] = (board_spec["TR"] - board_spec["BR"])/8
board_spec["D_B"] = (board_spec["BL"] - board_spec["BR"])/8

print(board_spec)

# Run Live loop
while vs.isOpened():

    # Get frame from stream
    active, img = vs.read()
    if not active: 
        print("Skipping blank frame...")
        continue

    # Process frame
    frame = process_frame(img, camera_parameters)
    draw_frame = frame.copy()

    # Save background from is not yet done
    if background_frame is None:
        background_frame = frame
    
    # Calculate difference between the background and current frame
    (_, diff) = ssim(frame, background_frame, multichannel=True, full=True)
    diff = cv2.cvtColor(1-diff.astype('float32'), cv2.COLOR_BGR2GRAY)
    
    # Let mess around with diff to make the regions full-er
    # Cool: make everything rectangles, erode/dilate, smooth
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    diff = cv2.morphologyEx(diff, cv2.MORPH_CLOSE, kernel, iterations=3)
    diff = cv2.GaussianBlur(diff, (3, 3), cv2.BORDER_DEFAULT)

    # Convert to colour for plotting (and stuff will bedrawn on here)
    diff_col = cv2.cvtColor(diff, cv2.COLOR_GRAY2BGR)

    # Get image regions in the difference image
    label_image = label(diff, connectivity=2)
    image_regions = regionprops(label_image)
    for region in image_regions:        
        y1, x1, y2, x2 = region.bbox
        diff_col = cv2.rectangle(diff_col, (x1, y1), (x2, y2), (255,0,0), 2)

    # Update image_regions_count, if there are 2 objects, else reset
    if len(image_regions) == 2: 
        two_region_frame_count += 1
    else:
        two_region_frame_count = 0

    # If over the threshold, find move, take new background, and reset counter
    if two_region_frame_count > MOVE_FRAME_THRESHOLD:
        # Find move
        for region in image_regions:
            # Work out board position
            # piece_pos = board_position(region.centroid, board_spec)
            coord = region.centroid

            board_row, board_col = None, None
            for val in range(8):
                # Test this row (board perspective)
                if board_row is None:
                    top = board_spec["TR"] + (board_spec["D_T"] * val)
                    bottom = board_spec["BR"] + (board_spec["D_B"] * val)

                    test = (coord[0] - top[0])*(bottom[1]-top[1]) - (coord[1] - top[1])*(bottom[0] - top[0])
                    if test > 0: 
                        board_row = val
                        print(f"Row: {board_row}")
                    
                if board_col is None:
                    left = board_spec["BL"] + (board_spec["D_L"] * val)
                    right = board_spec["BR"] + (board_spec["D_R"] * val)
                    test = (coord[0] - left[0])*(right[1]-left[1]) - (coord[1] - left[1])*(right[0] - left[0])
                    if test < 0:
                        board_col = val
                        print(f"Col: {board_col}")
        # Reset background and test
        background_frame = frame
        two_region_frame_count = 0

    # Draw the board outline
    draw_frame = cv2.polylines(draw_frame, [board_corners], True, (255, 0, 0), 2)

    # Show current and background side-by-side
    out_img = rearrange([draw_frame/255, diff_col], 't w h c -> w (t h) c')
    cv2.imshow('Camera Image', out_img)

    # Handle exit
    k = cv2.waitKey(10)
    if k & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
