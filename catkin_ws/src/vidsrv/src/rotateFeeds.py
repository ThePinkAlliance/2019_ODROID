#!/usr/bin/python

import cv2 as cv
import numpy as np
import rospy
import message_filters
import math
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisionSystem(object):
    def __init__(self):
        
        self.frontImage = None
        self.rearImage = None

        self.frontUpdate = False
        self.rearUpdate = False

        self.frontSubscriber = rospy.Subscriber("front_cam/image_raw", Image, self.frontCallback)
        self.rearSubscriber = rospy.Subscriber("rear_cam/image_raw", Image, self.rearCallback)


        self.publisher = rospy.Publisher("/driverFeed/image_raw", Image, queue_size = 1)

        self.bridge = CvBridge()

        while not rospy.is_shutdown():
            if self.frontUpdate and self.rearUpdate:
                self.publishImages()
                self.frontUpdate = False
                self.rearUpdate = False
                


    def frontCallback(self, front_cam):
        self.frontImage = front_cam
        self.frontUpdate = True

    def rearCallback(self, rear_cam):
        self.rearImage = rear_cam
        self.rearUpdate = True
        


    def publishImages(self):
        
        #convert to opencv image
        cv_front = self.bridge.imgmsg_to_cv2(self.frontImage, "bgr8")
        cv_rear = self.bridge.imgmsg_to_cv2(self.rearImage, "bgr8")


        #rotated_front = np.zeros((240,320,3), np.uint8)
        #rotated_rear = np.zeros((320,240,3), np.uint8)
        

        combined_image = np.zeros((480,320,3), np.uint8)

        clockwise_M = cv.getRotationMatrix2D((320/2,240/2),-90,1)
        counterClockwise_M = cv.getRotationMatrix2D((320/2,240/2),90,1)

        cosc = np.abs(clockwise_M[0,0])
        sinc = np.abs(clockwise_M[0,1])

        coscc = np.abs(counterClockwise_M[0,0])
        sincc = np.abs(counterClockwise_M[0,1])

        nWc = int((240*sinc) + (320*cosc))
        nHc = int((240*cosc) + (320*sinc))

        nWcc = int((240*sincc) + (320*coscc))
        nHcc = int((240*coscc) + (320*sincc))

        clockwise_M[0,2] += (nWc/2) - (320/2) - 1
        clockwise_M[1,2] += (nHc/2) - (240/2)

        counterClockwise_M[0,2] += (nWcc/2) - (320/2)
        counterClockwise_M[1,2] += (nHcc/2) - (240/2)


        rotated_front = cv.warpAffine(cv_front, clockwise_M, (240,320))
        rotated_rear = cv.warpAffine(cv_rear, counterClockwise_M, (240,320))


        combined_image = np.concatenate((rotated_front, rotated_rear), axis=1)


        self.publisher.publish(self.bridge.cv2_to_imgmsg(combined_image, "bgr8"))



def main(args):
    rospy.init_node('VisionSystem', anonymous=True)
    vision = VisionSystem()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS vision system"
    
if __name__ == '__main__':
    main(sys.argv)
