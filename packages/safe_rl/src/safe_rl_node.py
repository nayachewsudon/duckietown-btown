#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32, Bool

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

import cv2
from cv_bridge import CvBridge
import random
from collections import deque

import os
import time
import pickle

class SafeRLNode:
    def __init__(self):
        #Initialize node
        rospy.init_node('rlnode', anonymous=True)

        self.pub_cmd = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size = 1)
        
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        #CONTINUOUS ACTION SPACE
        

def run(self): 
    while not rospy.is_shutdown(): 
        self.rate.sleep()

if __name__ == '__main__': 
    try: 
        node = SafeRLNode()
        node.run()
    except rospy.ROSInterruptException: 
        pass