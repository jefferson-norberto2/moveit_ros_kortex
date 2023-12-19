#!/usr/bin/env python3

from rospy import init_node, loginfo, Publisher, Timer, Duration, spin
import png
import json
import logging
logging.basicConfig(level=logging.INFO)
import numpy as np
import cv2
import time
import os
from geometry_msgs.msg import Vector3

class Camera:
    def __init__(self):
        try:
            init_node("camera")
            self.robot_name = "my_gen3_lite"
            loginfo("Using robot_name " + self.robot_name)

            self.publisher_position = Publisher('move_robot', Vector3, queue_size=10)

            self.timer_pub = Timer(Duration(secs=5), self.timer_callback)

            self.x = 0.4
            self.y = 0.3
            self.z = 0.4

            self.is_initialized = True
        except Exception as e:
            loginfo("Error " + str(e))
            self.is_initialized = False
    
    def timer_callback(self, event):
        msg = Vector3()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        loginfo("Publiquei o x" + str(msg.x))
        self.publisher_position.publish(msg)
            
    def main(self):
        spin()

if __name__ == "__main__":
    camera = Camera()
    camera.main()
