#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
import roslib
import sys
import rospy
import cv2

def img_rgb(data):
    print(type(data))

def main(args):
    rospy.init_node('depth_finder',anonymous=True)
    rospy.Subscriber('/camera/depth/points',PointCloud2)
    rospy.Subscriber('/image_topic_2',Image,img_rgb)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')

if __name__ == '__main__':
    main(sys.argv)
    