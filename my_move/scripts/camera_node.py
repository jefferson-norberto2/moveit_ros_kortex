#!/usr/bin/env python3

from rospy import init_node, loginfo, Publisher, Subscriber, Timer, Duration, spin
import logging
logging.basicConfig(level=logging.INFO)
from geometry_msgs.msg._PoseStamped import PoseStamped
from webcam import Webcam
from camera import Camera
from aruco import Aruco
from kinova_transform import KinovaTransformation


class CameraNode():
    def __init__(self, cam: Camera, aruco: Aruco, transform: KinovaTransformation):
        try:
            init_node("camera_node")
            self.robot_name = "my_gen3_lite"
            loginfo("Using robot_name " + self.robot_name)

            self.publisher_position = Publisher("move_robot", PoseStamped, queue_size=10)

            self.cam = cam
            self.aruco = aruco
            self.trans = transform

            self.is_initialized = True
        except Exception as e:
            loginfo("Error " + str(e))
            self.is_initialized = False
    
    def publisher_pose(self, event):
        msg = PoseStamped()

        f = self.cam.get_frame()

        print('O frame', f)
        
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        loginfo("Publiquei x: " + str(msg.pose.position.x))
        self.publisher_position.publish(msg)
    
    def new_pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        loginfo("Atualizou a pose x:" + str(msg.pose.position.x))
        self.publisher_pose(None)

            
    def main(self):
        answer = ''
        while answer != 'e':
            frame = self.cam.get_frame()
            tv, rv, id = self.aruco.marker_pose()
            print("O que apareceu", tv, rv, id)
            posicoes = "Tenho que pegar ainda"
            self.trans.marker_to_base(rv, tv, posicoes)
            
            answer = input("press enter to repit or e to exit")

        self.cam.release_camera()

if __name__ == "__main__":
    cam = Webcam()
    aruco = Aruco(0, cam)
    trans = KinovaTransformation()
    camera = CameraNode(cam=cam, aruco=aruco, transform=trans)
    camera.main()
