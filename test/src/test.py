#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import roslib
import sys
import rospy
import cv2
import argparse
import ros_numpy

def callPoint(event,x,y,flags,param):
    global dX,dY,uX,uY
    if event == cv2.EVENT_LBUTTONDOWN:
        dX,dY = x,y
        print('1. x: ' + str(dX) + ' y: ' + str(dY))
    if event == cv2.EVENT_LBUTTONUP:
        uX,uY = x,y
        print('1. x: ' + str(uX) + ' y: ' + str(uY))
        rospy.Subscriber("/camera/depth/image_raw",Image,depth_calculator)


def depth_calculator(data):
    test_bridge = CvBridge()
    test_depth = test_bridge.imgmsg_to_cv2(data,'passthrough')
    depth = np.asarray(test_depth)
    print(np.average(depth[int(dY):int(uY),int(dX):int(uX)]))



class image_converter:
    
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)


    def callback(self,data):
        try:   
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.namedWindow('Image windows')
        cv2.setMouseCallback('Image windows',callPoint)
        cv2.imshow("Image windows", cv_image)
        #print(type(cv_image))
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    
    rospy.init_node('image_converter',anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    