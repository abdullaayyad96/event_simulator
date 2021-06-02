#!/usr/bin/env python

import numpy as np
import sys
import math
import time
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dvs_msgs.msg import Event, EventArray
from dynamic_reconfigure.server import Server
from event_simulator.cfg import EventSimCfgConfig
import copy
                                 

class event_simulator:
    def __init__(self):
        self.ros_node = rospy.init_node('event_simulator', anonymous=True)
        self.event_pubs = rospy.Publisher('/sim/events', EventArray, queue_size=1)
        self.image_subs = rospy.Subscriber("/dvs/image_raw", Image, self.image_cb)
        self.param_serv = srv = Server(EventSimCfgConfig, self.param_callback)

        self.cv_bridge = CvBridge()

        self.canny_low_thresh = 100
        self.canny_high_thresh = 150
        self.canny_kernel_size = 3

        rospy.spin()

    def image_cb(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        event_array = self.create_event_array_from_image(gray, msg.header.stamp)
        if len(event_array.events) > 0:
            self.event_pubs.publish(event_array)
        
    def detect_edges(self, gray_img):
        edge_img = cv2.Canny(gray_img, self.canny_low_thresh, self.canny_high_thresh, self.canny_kernel_size)

        edges = np.nonzero(edge_img)

        return edges, edge_img
        
    def create_event_array_from_image(self, image, timestamp):
        edges, _ = self.detect_edges(image)

        edges_x = edges[1]
        edges_y = edges[0]

        event_array = EventArray()
        event_array.header.stamp = timestamp
        event_array.header.frame_id = "dvs"

        num_rows, num_cols = image.shape
        event_array.width = num_cols
        event_array.height = num_rows

        for i in range(len(edges_x)):

            event = Event()
            event.x = edges_x[i]
            event.y = edges_y[i]
            event.ts = timestamp
            event.polarity = True
            
            event_array.events.append(event)

        return event_array

    def param_callback(self, config, level):
        self.canny_low_thresh = config.canny_low_thresh
        self.canny_high_thresh = config.canny_high_thresh
        self.canny_kernel_size = config.canny_kernel_size

        return config

if __name__ == '__main__':
    sim = event_simulator()
    exit()

