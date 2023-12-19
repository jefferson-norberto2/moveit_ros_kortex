#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from geometry_msgs.msg import Vector3

class PlanningMoveit(object):
  """PlanningMoveit"""
  def __init__(self):

    # Initialize the node
    super(PlanningMoveit, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('planning_moveit')

    try:
      self.name_space = '/my_gen3_lite/'
      # gripper_joint_names = rospy.get_param(self.name_space + "gripper_joint_names", [])
      gripper_joint_names = ['right_finger_bottom_joint']
      self.gripper_joint_name = gripper_joint_names[0]

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=self.name_space)
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=self.name_space)
      self.display_trajectory_publisher = rospy.Publisher(self.name_space + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      self.arm_group.set_planner_id("LazyPRMstar")
      self.arm_group.set_pose_reference_frame('base_link')
      self.arm_group.allow_replanning(False)
      self.arm_group.set_num_planning_attempts(20)
      self.arm_group.set_planning_time(4)

      print("planner query id -- ", self.arm_group.get_planner_id())

      self.move_subscriver = rospy.Subscriber('move_robot', Vector3, self.move_callback, queue_size=10)

      gripper_group_name = "gripper"
      self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=self.name_space)

      rospy.loginfo("Initializing node in namespace " + self.name_space)
      self.is_init_success = True
    except Exception as e:
      print (e)
      self.is_init_success = False
      
  def move_callback(self, msg: Vector3):
    rospy.loginfo("Enviaram " + str(msg.x))

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

  def main(self):
    rospy.spin()
  

if __name__ == '__main__':
  example = PlanningMoveit()
  example.main()
