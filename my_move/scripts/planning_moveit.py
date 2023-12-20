#! /usr/bin/env python3
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize
from rospy import loginfo, spin, init_node, Subscriber, Publisher, DEBUG, Timer, Duration
from geometry_msgs.msg._PoseStamped import PoseStamped

class PlanningMoveit():
  def __init__(self) -> None:  
    init_node('planning_moveit', anonymous=True, log_level=DEBUG)
    loginfo('Starting the Initialization')

    self.subcriber_move = Subscriber("move_robot", PoseStamped, self.my_move_callback, queue_size=10)
    self.publisher_pose = Publisher("my_joints", PoseStamped, queue_size=10)
    self.timer_pub = Timer(Duration(2), self.publish_joints)

    joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
    roscpp_initialize(joint_state_topic)

    self.robot = RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    self.scene = PlanningSceneInterface('my_gen3_lite', synchronous=True)

    group_name = "arm"
    self.move_group = MoveGroupCommander(group_name, robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    self.move_group.set_planner_id("LazyPRMstar")
    self.move_group.set_pose_reference_frame('base_link')
    self.move_group.allow_replanning(False)
    self.move_group.set_num_planning_attempts(20)
    self.move_group.set_planning_time(4)
    print("planner query id -- ", self.move_group.get_planner_id())

  def publish_joints(self):
    joinsts = self.move_group.get_joints()
    print("joints", joinsts)
    print("type", type(joinsts))
  
  def my_move_callback(self, msg: PoseStamped):
    print("Recebi o x", msg.pose.position.x)
    current_pose = self.get_cartesian_pose()
    
    new_pose = current_pose
    new_pose.pose.position.x = msg.pose.position.x
    new_pose.pose.position.y = msg.pose.position.y
    new_pose.pose.position.z = msg.pose.position.z

    self.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)

  def get_cartesian_pose(self) -> PoseStamped:
    # Get the current pose and display it
    pose = self.move_group.get_current_pose()
    loginfo("Actual cartesian pose is : ")
    loginfo(pose.pose)
    loginfo("Pose type: " + str(type(pose)))

    return pose
  
  def reach_cartesian_pose(self, pose: PoseStamped, tolerance, constraints):    
    # Set the tolerance
    self.move_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      self.move_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    self.move_group.set_pose_target(pose)

    # Plan and execute
    loginfo("Planning and going to the Cartesian Pose")
    return self.move_group.go(wait=True)
  
  def reach_named_position(self, target):    
    # Going to one of those targets
    loginfo("Going to named target " + target)
    # Set the target
    self.move_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = self.move_group.plan()
    # Execute the trajectory and block while it's not finished
    return self.move_group.execute(trajectory_message, wait=True)
  
  def main(self):
    self.reach_named_position("home")
    spin()
    

if __name__ == '__main__':
  planning = PlanningMoveit()
  planning.main()
