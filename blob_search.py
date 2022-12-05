#!/usr/bin/env python

import cv2
import numpy as np
import sys
import math as m

# ========================= Student's code starts here =========================

# Params for camera calibration
# theta = 0 # DEGREES
theta = 0 # RADIANS
beta = 750 # 75 pix/10cm = 750 pixels/m
tx = 210
ty = 80

# Function that converts image coord to world coord
def IMG2W(col, row):

    Oc = 320
    Or = 240

    xcr = (row-Or) #in camera, left/right = x-axis (size 640)
    ycc = (col-Oc) #in camera, up/down = y-axis (size 480)

    xw = xcr + tx
    yw = ycc + ty

    xr = xw/beta
    yr = yw/beta

    x = xr
    y = yr

    #x = xr*m.cos(theta)+yr*m.sin(theta)
    #y = yr*m.cos(theta)-xr*m.sin(theta)

    return x,y

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    # params.maxArea = 1500

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.2

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    yellow_lower = (20,240,240)     # yellow lower
    yellow_upper = (40,255,255)   # yellow upper

    green_lower = (50,240,240)     # green lower
    green_upper= (70,255,255)      # green upper

    purple_lower = (100,20,100)      # purple lower
    purple_upper = (130,100,150)   # purple upper

    lower = None
    upper = None

    if (color == 'yellow'):
        #print("o COLOR CHOSEN: " + color)
        lower = yellow_lower
        upper = yellow_upper
    elif (color == 'green'):
        #print("g COLOR CHOSEN: " + color)
        lower = green_lower
        upper = green_upper
    elif (color == 'purple'):
        #print("p COLOR CHOSEN: " + color)
        lower = purple_lower
        upper = purple_upper
    else:
        print('Invalid Color: ' + color)
        sys.exit()
    

    # Define masks using lower and upper bounds
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, image_raw, (255,0,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found of color " + color)
    else:
        #print(num_blobs)
        #print(blob_image_center[-1][0], blob_image_center[-1][1])
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
            #print(xw_yw[i])


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
