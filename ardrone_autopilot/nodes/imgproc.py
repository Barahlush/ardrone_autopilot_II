#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import rospy
from sensor_msgs.msg import Image

import cv2
import cookielib
import urllib
import math
import numpy as np
import os
import time
import urllib2

from threading import Thread

from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor():
    def __init__(self):
        self.encoding = rospy.get_param('~encoding', '8UC3')
        self.pub = rospy.Publisher('/out/image', Image, queue_size=5)
        rospy.init_node('imgproc_node')
        rospy.Subscriber('/in/image', Image, self.on_image, queue_size=5)
        self.bridge = CvBridge()

    def on_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data, self.encoding)
        res = cv2.resize(img, None, fx=7, fy=0.5, interpolation=cv2.INTER_CUBIC)
        img_msg = self.bridge.cv2_to_imgmsg(res, self.encoding)
        self.pub.publish(img_msg)



if __name__ == "__main__":
    ip = ImageProcessor()
    rospy.loginfo('Starting webcam')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
