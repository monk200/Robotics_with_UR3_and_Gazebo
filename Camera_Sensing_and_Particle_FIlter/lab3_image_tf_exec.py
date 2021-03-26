#!/usr/bin/env python

import sys
import cv2
import copy
import time
import numpy as np 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lab3_func import blob_search_init, blob_search


# Params for camera calibration
theta = 0 
beta = 0
tx = 0
ty = 0

class ImageConverter:

    def __init__(self, SPIN_RATE):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)
        self.detector = blob_search_init()

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")

    def image_callback(self, data):

        global theta
        global beta
        global tx
        global ty

        try:
            # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Flip the image 180 degrees
        cv_image = cv2.flip(raw_image, -1)

        # Draw a black line on the image
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # cv_image is normal color image
        blob_image_center = blob_search(cv_image, self.detector)

        # Given world coordinate (xw, yw)
        xw = 0.2875
        yw = 0.1125

        print(blob_image_center, " BLOB_TEST")

        # Only two blob center are found on the image
        if(len(blob_image_center) < 2):
            print("Not enough blobs found. " + str(len(blob_image_center)) + " found.")

        elif(len(blob_image_center) == 2):

            c1 = int(blob_image_center[0].split()[0])
            r1 = int(blob_image_center[0].split()[1])
            c2 = int(blob_image_center[1].split()[0])
            r2 = int(blob_image_center[1].split()[1])

            print("Blob Center 1: ({0}, {1}) and Blob Center 2: ({2}, {3})".format(c1, r1, c2, r2))

            ################################# Your Code Start Here ################################# 

            # Calculate beta, tx and ty, given c1, r1, c2, r2
            crop_top_row = 130
            crop_bottom_row = 350
            crop_top_col = 130
            crop_bottom_col = 450

            O_r = -(.5 * (crop_top_col - crop_bottom_col))      # O_r = .5*height
            O_c = -(.5 * (crop_top_row - crop_bottom_row))      # O_c = .5*width

            beta = 770
            theta = .019682 #radians

            tx = 7/beta #meters
            ty = 15/beta #meters
            X_c_1 = (r1 -O_r)/beta
            Y_c_1 = (c1 -O_c)/beta
            X_c_2 = (r2 -O_r)/beta
            Y_c_2 = (c2 -O_c)/beta

            rotation = np.array([[cos(theta * np.pi/180), -sin(theta * np.pi/180)],[sin(theta * np.pi/180),cos(theta * np.pi/180)]])
            translation = np.array([[tx],[ty]])
            camera_1 = np.array([[X_c_1],[Y_c_1]])
            camera_2 = np.array([[X_c_2],[Y_c_2]])

            world_1 = np.dot(np.linalg.inv(translation),np.subtract(camera_1 - translation))
            world_2 = np.dot(np.linalg.inv(translation),np.subtract(camera_2 - translation))

            ################################## Your Code End Here ################################## 

            print("theta = {0}\nbeta = {1}\ntx = {2}\nty = {3}\n".format(theta, beta, tx, ty))
        else:
            print("To many points returned " + str(len(blob_image_center)) + " found.")


def main():

    SPIN_RATE = 20 # 20Hz
    rospy.init_node('lab3ImageCalibrationNode', anonymous=True)
    ic = ImageConverter(SPIN_RATE)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()