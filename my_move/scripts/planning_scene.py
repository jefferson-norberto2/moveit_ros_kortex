#! /usr/bin/env python3
import sys
import copy
import time
import math
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import moveit_commander
import geometry_msgs.msg 
import sensor_msgs.msg
import moveit_msgs.msg
import rospy  
moveit_msgs.msg.MoveItErrorCodes
planning_times = []
times_from_start = []
plans = []
execution_sucess_list = []
success = True

def teste1(move_group, pose_goal):
    global success
    
    move_group.set_start_state_to_current_state()
    joint_start = copy.deepcopy(move_group.get_current_joint_values())
    print("joints inicial or last position --> ", joint_start)
    move_group.set_pose_reference_frame('base_link')
    move_group.set_num_planning_attempts(100)
    move_group.set_planning_time(5)

    print("planner to move to start postion id -- ", move_group.get_planner_id())
    joint_start = [0.64353, -0.76711, 1.34479, -0.89388, 0.72538, 0.92763]
    move_group.set_planner_id("RRTConnect")
    move_group.set_joint_value_target(joint_start)
    plan_start = move_group.plan()
    while not plan_start[0]:
        plan_start = move_group.plan()
    print("global success", success)
    if success:
        start_position_confirmed = move_group.execute(plan_start[1], wait=True)

        print("start execution", start_position_confirmed)
        
        while not start_position_confirmed:
            start_position_confirmed = move_group.execute(plan_start[1], wait=True)

    rospy.sleep(0.5)        
    move_group.stop()
    move_group.set_start_state_to_current_state()    


    move_group.set_pose_target(pose_goal)
    # move_group.set_goal_tolerance(0.025)
    move_group.set_planner_id("LazyPRMstar")
    move_group.set_pose_reference_frame('base_link')
    move_group.allow_replanning(False)
    move_group.set_num_planning_attempts(20)
    move_group.set_planning_time(4)

    print("planner query id -- ", move_group.get_planner_id())
    
    plan = move_group.plan()

    print("plan success = ", plan[0])
    if plan[0]:
        print("going to position")
        success = move_group.execute(plan[1], wait=True)
        print("success of plan =", success)
        if success:
            execution_sucess_list.append(1)   
            rospy.sleep(0.5)
            move_group.stop()
            time_from_start = plan[1].joint_trajectory.points[-1].time_from_start.to_sec()
            times_from_start.append(time_from_start)
        else:
            execution_sucess_list.append(0)    
        pose_goal = move_group.get_current_pose()
        plans.append(plan)
    else:
        print("plan failed")
        plans.append(plan)

    print("planning time: ", plan[2])
    print("error code: ", plan[3])
    return plan_start

# Não funciona, mas capturado usando outro método
def check_computation_time(msg):
    # get computation time for successful plan to be found
    print("message ", msg)
    if msg.status.status == 3:
        planning_times.append(msg.result.planning_time)
    

if __name__ == '__main__':

    rospy.init_node('planning_scene_kinova', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting the Initialization')
    print("argv -- ", sys.argv)
    joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)

    robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    scene = moveit_commander.PlanningSceneInterface('my_gen3_lite', synchronous=True)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")

    display_trajectory_publisher = rospy.Publisher( "/move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20,
                                                   )
    pub_work_scene = rospy.Publisher('/my_gen3_lite/planning_scene',moveit_msgs.msg.PlanningScene, queue_size=20)
    # joint_state_publisher = rospy.Publisher('/my_gen3_lite/joint_states', sensor_msgs.msg.JointState, queue_size=20)
    
    # Esse subscriber não está printando o callback - TODO - descobrir o motivo depois
    planning_time_sub = rospy.Subscriber('/move_group/result', moveit_msgs.msg.MoveGroupActionResult, check_computation_time)
    

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # # Sometimes for debugging it is useful to print the entire state of the
    # # robot:
    # print("============ Printing robot state")
    # robot_state = robot.get_current_state()
    # print(type(robot_state))
    # print("")

    ## Planner params
    #print("============ Printing planner param")
    #print(robot.get_planner_param())
    #print("")

    rospy.loginfo('Cleaning of the objects in the scene')
    try:
        scene.clear()

    except Exception as e:
        print(e)

    rospy.sleep(1.5)
    #euler_from_quaternion --> https://python.hotexamples.com/pt/site/file?hash=0xddea61d221dba056f657cb35f51cee33fd02af40dd9863afc4b2d7e6ebf1abad&fullName=msc-kinova-experiments-master/src/grasp_server/moveit.py&project=m-rios/msc-kinova-experiments

    quaternion = quaternion_from_euler(1.60, 0.50, 0.00, axes='rxyz')
    # print("quaternion -- ", quaternion)
    stl_id = 'grasping_object'
    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = 'base_link'
    obj_pose.pose.position.x =  0.35 
    obj_pose.pose.position.y = 1.265
    obj_pose.pose.position.z = -0.01
    obj_pose.pose.orientation.x = quaternion[0]
    obj_pose.pose.orientation.y = quaternion[1]
    obj_pose.pose.orientation.z = quaternion[2]
    obj_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo('Adding the structure')
    scene.add_mesh(
    stl_id, obj_pose,
    '/home/robo1/Documents/catkin_ws/src/my_move/scripts/montagemApenasEstrutura_.stl'
    )

    rospy.loginfo('Adding the structure')

    time.sleep(1)

    rospy.loginfo('Adding the table object')
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = 0.00
    table_pose.pose.position.y = 0.00
    table_pose.pose.position.z = -0.01
    scene.add_box("table", table_pose, size=(1.90, 1.8, 0.01))
    
    time.sleep(1)

    paths_lenght = []
    planning_time_list = []

    # montagemApenasEstrutura_ MESH POSE CONFIGURAÇÃO LAB ATUAL
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.pose.position.x = 0.28956
    pose_goal.pose.position.y = 0.55213
    pose_goal.pose.position.z = 0.08001
    pose_goal.pose.orientation.x = 0.24478
    pose_goal.pose.orientation.y = 0.93136
    pose_goal.pose.orientation.z = 0.25643
    pose_goal.pose.orientation.w = 0.08302
    pose_goal.header.frame_id = "world"
    print("pose goal frame_id ", pose_goal.header.frame_id)


    for i in range(1, 5):
        print("running:", i)
        plan = teste1(move_group, pose_goal)
        planning_time = plan[2]
        waypoints = len(plan[1].joint_trajectory.points)
        if plan[0]:
            motion_time = plan[1].joint_trajectory.points[-1].time_from_start.to_sec()
            motion_time = round(motion_time, 4)
            planning_time_list.append(motion_time)
        paths_lenght.append(waypoints)

    print("waypoints25 =", paths_lenght)
    print("planning_times25 =", planning_time_list)
    print("succesful_execution_times =", [round(i, 4) for i in times_from_start])
    print("25 list success", execution_sucess_list)

    print("planner interface ", move_group.get_interface_description().planner_ids)
    moveit_commander.roscpp_shutdown()
