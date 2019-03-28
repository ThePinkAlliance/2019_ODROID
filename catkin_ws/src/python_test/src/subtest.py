#!/usr/bin/python

import cv2 as cv
import numpy as np
import rospy
import math
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisionSystem(object):
    def __init__(self, min_thresh=227, max_tresh=255):
        self.subscriber = rospy.Subscriber("front_cam/image_raw", Image, self.callback, queue_size = 1)

        self.publisher = rospy.Publisher("/detectedcontours/image_raw", Image)

        self.bridge = CvBridge()


    def callback(self, ros_data):
        
        #convert to opencv image
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")

        (height, width, channels) = cv_image.shape

        #convert to grayscale, threshold, find contours
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        n, threshImg = cv.threshold(gray, 250, 255, cv.THRESH_BINARY)
        n, contours, h = cv.findContours(threshImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(cv_image, contours, -1, (0,255,0), 3)

        #for each contour
        for cntr in contours:

            #get bounding rotated rectangle, make into numpy box
            rect = cv.minAreaRect(cntr)
            box = cv.boxPoints(rect)
            box = np.int0(box)

            #get center of rectangle
            center = ( int(rect[0][0]), int(rect[0][1]) )

            #make sure it isn't just a... point? 
            if set(box[0] != box[1]):

                #sort so we have top left and top right
                sortedBox = sorted(box, key=lambda point:point[1])
                topLeft = sortedBox[0]
                topRight = sortedBox[1]

                #get width and height from rotated rectangle
                wid = rect[1][0]
                hei = rect[1][1]

                #as long as neither is zero
                if( wid != 0 and hei != 0 ):

                    #get aspect ratios
                    ratio1 = wid/hei
                    ratio2 = hei/wid

                    #aspect ratio is 2.75 or so, use both ratios because
                    #something weird happened with the rotation of the
                    #rectangles and I don't feel like figuring it out now.
                    if ( (ratio1 > 2.2 and ratio1 < 3.3) or (ratio2 > 2.2 and ratio2 < 3.3) ):

                        #draw bounding box and center
                        cv.drawContours(cv_image, [ box ], 0, (255, 0, 0), 2)
                        cv.circle(cv_image, center, 1, (255, 0, 0), thickness=2, lineType=8, shift=0)


        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))



def main(args):
    vision = VisionSystem()
    rospy.init_node('VisionSystem', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS vision system"
    
if __name__ == '__main__':
    main(sys.argv)
