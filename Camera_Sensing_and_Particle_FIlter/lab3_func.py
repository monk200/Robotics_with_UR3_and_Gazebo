#!/usr/bin/env python

import cv2
import numpy as np

"""
To init blob search params, will be init (called) in the ImageConverter class
"""
def blob_search_init():

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    ################# Your Code Starts Here #################

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = False

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.maxCircularity = 1.2

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # Any other params to set???

    ################## Your Code Ends Here ##################

    # Create a detector with the parameters
    blob_detector = cv2.SimpleBlobDetector_create(params)

    return blob_detector


"""
To find blobs in an image, will be called in the callback function of image_sub subscriber
"""
def blob_search(image, detector):

    # Convert the color image into the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    ############################ Your Code Starts Here ############################

    # Find lower & upper for orange
    lower = (1,190,138)      # orange lower
    upper = (29,270,230)   # orange upper

    ############################# Your Code Ends Here #############################

    # Define a mask using the lower and upper bounds of the orange color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    crop_top_row = 130
    crop_bottom_row = 350
    crop_top_col = 130
    crop_bottom_col = 450

    crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

    blob_image_center = []

    ############################ Your Code Start Here ############################

    # Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in
    # crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

    # Make sure this blob center is in the full image pixels not the cropped image pixels
    #print("is it here?")

    # Set up the detector with default parameters.
    detector = blob_search_init()

    # Detect blobs.
    keypoints = detector.detect(mask_image)

    keypoints_list = list(keypoints[0].pt)
    keypoints_list[0] = keypoints[0].pt[0] - crop_top_row
    keypoints_list[1] = keypoints[0].pt[1] - crop_top_col

    #print "Number of Keypoints = ", len(keypoints)
    #if len(keypoints) >= 1:
    #    print "Location = ", keypoints_list[0], keypoints_list[1]

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)

    # Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"
    for keypoints in keypoints:
        blob_image_center.append(str(int(keypoints.pt[0]))+" "+str(int(keypoints.pt[1])))

    im_with_keypoints = image # Change this to create a proper im_with_keypoints

    ############################# Your Code End Here #############################

    # Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
    # under that pixel location and see what the HSV values are for that color.
    #pixel = [int(crop_bottom_col), int(crop_top_row)] # Change this to select a new dot location [column, row]
    pixel = [int(crop_bottom_col), int(260)]
    im_with_keypoints = cv2.circle(im_with_keypoints, (pixel[0], pixel[1]), 3, (0, 0, 255), -1) #Note: circle uses (c,r) for its center
    print('H,S,V at pixel ' + str(pixel[1]) + ' ' + str(pixel[0]) + ' ' + str(hsv_image[pixel[1],pixel[0]]))    # hsv_image uses [r,c]

    cv2.namedWindow("Maze Window")
    cv2.imshow("Maze Window", im_with_keypoints)

    cv2.namedWindow("MaskImage Window")
    cv2.imshow("MaskImage Window", mask_image)

    cv2.namedWindow("Crop Window")
    cv2.imshow("Crop Window", crop_image)

    cv2.waitKey(2)

    return blob_image_center
