#!/usr/bin/env python3

from rospy import init_node, loginfo, Publisher, Subscriber, Timer, Duration, spin
import logging
logging.basicConfig(level=logging.INFO)
from geometry_msgs.msg._PoseStamped import PoseStamped
from webcam import Webcam
from camera import Camera
from aruco import Aruco
from kinova_transform import KinovaTransformation
from std_msgs.msg import String
from math import degrees


class CameraNode():
    def __init__(self, cam: Camera, aruco: Aruco, transform: KinovaTransformation):
        try:
            init_node("camera_node")
            self.robot_name = "my_gen3_lite"
            loginfo("Using robot_name " + self.robot_name)

            self.publisher_position = Publisher("move_robot", PoseStamped, queue_size=10)
            self.joints_sub = Subscriber("my_joints", String, self.update_joints_value)
            self.joints = [0, 0, 0, 0, 0, 0]

            self.cam = cam
            self.aruco = aruco
            self.trans = transform

            self.is_initialized = True
        except Exception as e:
            loginfo("Error " + str(e))
            self.is_initialized = False
    
    def update_joints_value(self, msg: String):
        ljoint = []
        for j in msg.data.split(','):
            j = j.replace('[', '')
            j = j.replace(']', '')
            ljoint.append(float(j))
        
        for i in range(len(self.joints)):
            self.joints[i] = degrees(ljoint[i])
    
    def publisher_pose(self):
        msg = PoseStamped()

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        print(f"Publiquei x: {msg.pose.position.x}, y: {msg.pose.position.y} z: {msg.pose.position.z}")
        self.publisher_position.publish(msg)

            
    def main(self):
        answer = ''
        while answer != 'e':
            try:
                tv, rv, id = self.aruco.marker_pose()
                print(f"TV: {tv}, RV: {rv}")
                pose = self.trans.marker_to_base(rv, tv, self.joints)
                self.x = pose[0] / 10
                self.y = pose[1] / 10
                self.z = pose[2] / 10
                self.publisher_pose()
            except:
                pass
            
            answer = input("press enter to repit or e to exit")

        self.cam.release_camera()

if __name__ == "__main__":
    cam = Webcam()
    aruco = Aruco(0, cam)
    trans = KinovaTransformation()
    camera = CameraNode(cam=cam, aruco=aruco, transform=trans)
    camera.main()
