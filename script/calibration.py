#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import signal

import sys
import os

class Calibration :
    def __init__(self, topic) :
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic , Image , self.imageCallback)
        self.checker_row = 8
        self.checker_col = 6

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER , 30 , 0.001)

        self.objp = np.zeros((self.checker_row * self.checker_col , 3) , np.float32)
        self.objp[: , :2] = np.mgrid[0:self.checker_row , 0:self.checker_col].T.reshape(-1 , 2)*25

        self.objpoints = []
        self.imgpoints = []

    def shutdown_print() :
        print(" ")

    def imageCallback(self, data) :
        try :
            cv_image = self.bridge.imgmsg_to_cv2(data , desired_encoding='bgr8')

        except CvBridgeError as e:
            print(e)
            return



        self.gray_img = cv2.cvtColor(cv_image , cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(self.gray_img , (self.checker_row , self.checker_col) , None)
        if ret == True:
            self.objpoints.append(self.objp)
            corners2 = cv2.cornerSubPix(self.gray_img,corners,(11,11),(-1,-1),self.criteria)
            self.imgpoints.append(corners2)
            # Draw and display the corners
            img = cv2.drawChessboardCorners(cv_image, (self.checker_row, self.checker_col), corners2,ret)
            cv2.imshow('image',img)
            cv2.waitKey(2)

        else :
            cv2.imshow('image' , cv_image)
            cv2.waitKey(2)

        if len(self.objpoints) == 300 :
            print('Calibration !')
            print("Captured Frames : " + str(len(self.objpoints)))
            cv2.destroyAllWindows()

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray_img.shape[::-1],None,None)

            print("=========Camera Mtx=========")
            for i in mtx :
                print(i)

            rospy.on_shutdown(self.shutdown_print)
            sys.exit(0)


def main(args) :

    rospy.init_node("Image_test")
    topic = "/camera/color/image_raw"
    calib = Calibration(topic)
    rospy.spin()

if __name__ == "__main__" :
    main(sys.argv)
