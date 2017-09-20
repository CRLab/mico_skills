import rospy
import copy
from math import sin, cos
import sys
import moveit_commander
import roscpp

import graspit_msgs.msg
import graspit_commander
import actionlib
import geometry_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import (MoveGroupInterface, PickPlaceInterface)

from grid_sample_client import GridSampleClient
from graspit_commander import GraspitCommander
import graspit_interface.msg
import tf.transformations
from geometry_msgs.msg import Twist
import time

#Move the base
def goto(client, x, y, frame="map"):
    move_goal = MoveBaseGoal()
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    # move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
    # move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
    move_goal.target_pose.pose.orientation.z = 0.998
    move_goal.target_pose.pose.orientation.w = -0.055
    move_goal.target_pose.header.frame_id = frame
    move_goal.target_pose.header.stamp = rospy.Time.now()
    # TODO wait for things to work
    client.send_goal(move_goal)
    client.wait_for_result()

#Move the base in Agumented Reality App
def ar_goto(client, x, y, ori_x, ori_y, ori_z, ori_w, frame="map"):

    move_goal = MoveBaseGoal()
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y

    move_goal.target_pose.pose.orientation.x = ori_x
    move_goal.target_pose.pose.orientation.y = ori_y
    move_goal.target_pose.pose.orientation.z = ori_z
    move_goal.target_pose.pose.orientation.w = ori_w
    move_goal.target_pose.header.frame_id = frame
    move_goal.target_pose.header.stamp = rospy.Time.now()
    # TODO wait for things to work
    client.send_goal(move_goal)
    client.wait_for_result()




def set_gripper_width(val=0.05):
    mgc = moveit_commander.MoveGroupCommander("gripper")
    jv = mgc.get_current_joint_values()
    jv = [val,val]
    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)

def open_gripper(val=0.05):
    set_gripper_width(val)

def close_gripper(val=0.0):
    set_gripper_width(val)


def home_torso(height=0.211):
    mgc = moveit_commander.MoveGroupCommander("arm_with_torso")
    jv = mgc.get_current_joint_values()
    jv[0] = height

    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)


def home_arm():
    mgc = moveit_commander.MoveGroupCommander('arm')
    mgc.set_named_target('home')
    p =mgc.plan()

    return mgc.execute(p)

def customized_arm():
    mgc = moveit_commander.MoveGroupCommander('arm')
    mgc.set_named_target('customized_place')
    p =mgc.plan()
    return mgc.execute(p)


def remove_object_from_hand_and_scene():
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    scene.remove_attached_object('wrist_roll_link')
    scene.remove_world_object()

def attach_object_to_hand(object_name):
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)
    scene. attach_mesh(link='wrist_roll_link', name=object_name)


def customized_place():
    customized_arm()
    rospy.sleep(3)
    open_gripper()


def customized_pick():
    customized_arm()
    rospy.sleep(4)
    close_gripper()


#Follow the trajectory
def move_to(client, joint_names, positions, duration=5.0):
    if len(joint_names) != len(positions):
        print("Invalid trajectory position")
        return False
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = positions
    trajectory.points[0].velocities = [0.0 for _ in positions]
    trajectory.points[0].accelerations = [0.0 for _ in positions]
    trajectory.points[0].time_from_start = rospy.Duration(duration)
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory

    client.send_goal(follow_goal)
    client.wait_for_result()

#Point head on detected object
def look_at(client, x, y, z, frame, duration=1.0):
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = frame
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z
    goal.min_duration = rospy.Duration(duration)
    client.send_goal(goal)
    client.wait_for_result()


def move_head(head_goal=[0.00583338737487793, 1.0428600030838013]):
        #start head_controller/follow_joint_trajectory action
        head_client = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        head_client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        
        head_joints = ['head_pan_joint', 'head_tilt_joint']

        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = head_joints
        head_trajectory.points.append(JointTrajectoryPoint())
        head_trajectory.points[0].positions = head_goal
        head_trajectory.points[0].velocities = [0.0 for i in head_joints]
        head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
        head_trajectory.points[0].time_from_start = rospy.Duration(3.0)
        #send the trajectory to the head action server
        rospy.loginfo('Homing the head...')

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = head_trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        #send the goal
        head_client.send_goal(head_goal)

        #wait for up to 5 seconds for the motion to complete
        head_client.wait_for_result(rospy.Duration(5.0))

def wheel_control(duration, speed):
    tw = Twist()
    tw.linear.x = speed
    start_time = time.time()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    execute_time = 0
    while execute_time <= duration:
        end_time = time.time()
        execute_time = end_time - start_time
        pub.publish(tw)

#Tuck the arm    
def tuck():
    move_group = MoveGroupInterface("arm", "base_link")
    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    while not rospy.is_shutdown():
        result = move_group.moveToJointPosition(joints, pose, 0.02)

def place(save_for_place_grasps, place_pose_stamp):
    rospy.loginfo("About to place the object ...")
    place_client = actionlib.SimpleActionClient('place_execution_action', graspit_interface.msg.PlaceExecutionAction)
    place_client.wait_for_server()
    goal = graspit_interface.msg.PlaceExecutionGoal()
    goal.grasp = save_for_place_grasps[0]
    goal.place_pose_stamp = place_pose_stamp
    place_client.send_goal(goal)    
    place_client.wait_for_result()
    

def get_grasp_from_graspit(
    model_name,
    mesh_path,
    robot="fetch_gripper",
    obstacle="table"):

    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()

    # gc.importObstacle(obstacle)
    # table_pose = geometry_msgs.msg.Pose()
    # table_pose.orientation.w = 1
    # table_pose.position.x = 0.53
    # table_pose.position.y = -0.687
    # table_pose.position.z = 0.505
    # gc.setBodyPose(0,table_pose)

    gc.importRobot(robot)
    gc.importGraspableBody(mesh_path)

    # response = gc.planGrasps()
    gl = GridSampleClient()
    result = gl.computePreGrasps(20, 1)

    pre_grasps = result.grasps
    unchecked_for_reachability_grasps = gl.evaluatePreGrasps(pre_grasps, pre_grasp_dofs=(4,))

    return unchecked_for_reachability_grasps


def check_grasps_for_reachability(
        unchecked_for_reachability_grasps,
        model_name):

    reachability_client = actionlib.SimpleActionClient('analyze_grasp_action', graspit_interface.msg.CheckGraspReachabilityAction)
    reachability_client.wait_for_server()

    reachable_grasps = []

    for i, unchecked_grasp in enumerate(unchecked_for_reachability_grasps):
        rospy.loginfo("checking grasp "+ str(i) +" for reachability")

        # #this is the message we are sending to reachability analyzer to check for reachability
        goal = graspit_interface.msg.CheckGraspReachabilityGoal()
        goal.grasp = unchecked_grasp

        reachability_client.send_goal(goal)    
        reachability_client.wait_for_result()


        reachability_check_result = reachability_client.get_result()
        if reachability_check_result.isPossible:
            reachable_grasps.append(goal.grasp)
            break

    return reachable_grasps


def grasp_old(
    model_name,
    mesh_path,
    robot="fetch_gripper",
    obstacle="table"):  

    rospy.loginfo("About to plan grasps in Graspit")

    unchecked_for_reachability_grasps = get_grasp_from_graspit(
        model_name,
        mesh_path,
        robot="fetch_gripper",
        obstacle="table")

    rospy.loginfo("We have received grasps from Graspit")

    for i in range(len(unchecked_for_reachability_grasps)):
        unchecked_for_reachability_grasps[i].object_name = model_name
        unchecked_for_reachability_grasps[i].approach_direction.vector.z = 1.0

    print "skills.py:256"
    import IPython
    IPython.embed()
    assert False

    rospy.loginfo("checking grasps for reachability")

    reachable_grasps = check_grasps_for_reachability(
        unchecked_for_reachability_grasps,
        model_name)

    rospy.loginfo("Finished checking grasps for reachability")


    if len(reachable_grasps):
        rospy.loginfo("going to execute first reachable grasp")
        # import IPython
        # IPython.embed()
        execution_client = actionlib.SimpleActionClient('grasp_execution_action', graspit_interface.msg.GraspExecutionAction)
        execution_client.wait_for_server()

        goal = graspit_interface.msg.GraspExecutionGoal()
        goal.grasp = reachable_grasps[0]


        execution_client.send_goal(goal)    
        execution_client.wait_for_result()
    else:
        rospy.loginfo("No reachable grasps found")

    # print "skills.py:285"
    # import IPython
    # IPython.embed()
    # assert False
    return reachable_grasps



def grasp(
    model_name,
    mesh_path,
    robot="fetch_gripper",
    obstacle="table"):  

    rospy.loginfo("About to plan grasps in Graspit")

    unchecked_for_reachability_grasps = get_grasp_from_graspit(
        model_name,
        mesh_path,
        robot="fetch_gripper",
        obstacle="table")

    rospy.loginfo("We have received grasps from Graspit")

    for i in range(len(unchecked_for_reachability_grasps)):
        unchecked_for_reachability_grasps[i].object_name = model_name
        unchecked_for_reachability_grasps[i].approach_direction.vector.x = 1.0

    pick_success, pick_plan = execute_graspit_grasps(unchecked_for_reachability_grasps)



    return pick_success, pick_plan



def execute_graspit_grasps(list_of_graspit_interface_grasp_msgs):

    open_gripper()
    pick_success =  False

    for i in range(len(list_of_graspit_interface_grasp_msgs)):
        success, pick_plan = get_pick_plan(list_of_graspit_interface_grasp_msgs[i])
        if success:
            object_name = list_of_graspit_interface_grasp_msgs[i].object_name
            pick_success = execute_pickup_plan(pick_plan, object_name)
            break

    # TODO: Attach object to end_effector link
    return pick_success, pick_plan


def execute_pickup_plan(pick_plan, object_name):       
    ''' TODO: Check if necessary '''
    mgc_arm = moveit_commander.MoveGroupCommander('arm')
    mgc_gripper = moveit_commander.MoveGroupCommander("gripper")

    # pick_plan.trajectory_descriptions
    # ['plan', 'pre_grasp', 'approach', 'grasp', 'retreat']

    success = mgc_arm.execute(pick_plan.trajectory_stages[0])
    success = mgc_gripper.execute(pick_plan.trajectory_stages[1])

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)
    scene.attach_mesh(link='wrist_roll_link', name=object_name, touch_links = ['gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'])   

    success = mgc_arm.execute(pick_plan.trajectory_stages[3])
    success = mgc_gripper.execute(pick_plan.trajectory_stages[4])

    return success



from reachability_analyzer.message_utils import build_pickup_goal, graspit_grasp_to_moveit_grasp
import moveit_msgs

def send_pick_request(pickup_goal):

    allowed_planning_time = rospy.get_param('reachability_analyzer/allowed_planning_time') + 1

    pick_plan_client = actionlib.SimpleActionClient('/pickup', moveit_msgs.msg.PickupAction)
    pick_plan_client.wait_for_server(rospy.Duration(3))

    success = False
    received_result = False
    pick_plan_client.send_goal(pickup_goal)

    received_result = pick_plan_client.wait_for_result(rospy.Duration(allowed_planning_time))

    if received_result:
        result = pick_plan_client.get_result()
        rospy.loginfo("result: " + str(result))
        success = result.error_code.val == result.error_code.SUCCESS
    else:
        result = None
        success = False

    rospy.loginfo("success of pick_plan_client:" + str(success))

    return success, result


def get_pick_plan(graspit_grasp_msg):

    moveit_grasp_msg = graspit_grasp_to_moveit_grasp(graspit_grasp_msg)

    move_group_name = rospy.get_param('move_group_name')
    planner_id = move_group_name + rospy.get_param('reachability_analyzer/planner_config_name')
    group = moveit_commander.MoveGroupCommander(move_group_name)    
    allowed_planning_time = rospy.get_param('reachability_analyzer/allowed_planning_time')

    pickup_goal = build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                      object_name=graspit_grasp_msg.object_name,
                                      allowed_planning_time = allowed_planning_time,
                                      planner_id=planner_id,
                                      planning_group=group)

    success, result = send_pick_request(pickup_goal)

    return success, result


def place_new():

    mgc = moveit_commander.MoveGroupCommander('arm')
    mgc.set_named_target('customized_place_0')
    p =mgc.plan()
    mgc.execute(p)
    open_gripper()
