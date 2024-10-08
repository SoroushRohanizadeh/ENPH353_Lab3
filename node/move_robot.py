#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np


blackThreshold = 127

class ControlNode:

    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)

        self.bridge = CvBridge()
        self.move = Twist()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.callback)
        self.rate = rospy.Rate(2)

    
    def processFrame(self, frame):
        """
        Process the given frame and return the middle point.

        @param frame The frame to process
        """
        ret, binarized = cv.threshold(frame,blackThreshold,255,cv.THRESH_BINARY)

        dilationKernel = np.ones((5,5))
        dilated = cv.dilate(binarized, dilationKernel, iterations = 1)

        yLimit = dilated.shape[0] - 1

        lineSeen = False
        firstLineXPixel = 0
        numLineXPixels = 0

        for i in range(dilated.shape[1]):
            if (not lineSeen):
                if (self.isLinePixel(dilated[yLimit, i])):
                    lineSeen = True
                    firstLineXPixel = i
                    numLineXPixels += 1
            else:
                numLineXPixels += 1
                if (not self.isLinePixel(dilated[yLimit, i])):
                    break
        return int(firstLineXPixel + numLineXPixels / 2)

    def isLinePixel(self, pixel):
        """
        Return true if the given pixel is on the line.

        @param pixel The pixel to check
        """
        return pixel[0] < blackThreshold and pixel[1] < blackThreshold and pixel[2] < blackThreshold

    def callback(self, img):
        cvImg = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        middle = self.processFrame(cvImg)
        error = cvImg.shape[1] / 2 - middle

        kP = 1 / 400

        self.move.linear.x = 0.1
        self.move.angular.z = error * kP

        self.pub.publish(self.move)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ControlNode()
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
