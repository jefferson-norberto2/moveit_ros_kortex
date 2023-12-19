#!/usr/bin/env python3

from rospy import init_node, loginfo, Publisher, Subscriber, Timer, Duration, spin
import logging
logging.basicConfig(level=logging.INFO)
from geometry_msgs.msg import Vector3
from geometry_msgs.msg._PoseStamped import PoseStamped

class Camera:
    def __init__(self):
        try:
            init_node("camera")
            self.robot_name = "my_gen3_lite"
            loginfo("Using robot_name " + self.robot_name)

            self.publisher_position = Publisher("move_robot", PoseStamped, queue_size=10)

            self.subscriber_pose = Subscriber("new_move", PoseStamped, self.new_pose_callback, queue_size=10)

            self.timer_pub = Timer(Duration(secs=5), self.timer_callback)

            self.x = 0.3
            self.y = 0.2
            self.z = 0.5

            self.stop_timer = True

            self.is_initialized = True
        except Exception as e:
            loginfo("Error " + str(e))
            self.is_initialized = False
    
    def timer_callback(self, event):
        msg = PoseStamped()
        
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
        if self.stop_timer:
            self.timer_pub.shutdown()
            self.stop_timer = False
        self.timer_callback(None)

            
    def main(self):
        spin()

if __name__ == "__main__":
    camera = Camera()
    camera.main()
