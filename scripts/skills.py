#!/usr/bin/env python

import graspit_commander
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from message_utils import build_pickup_goal, graspit_interface_to_moveit_grasp
import moveit_msgs.msg
import actionlib
import curvox.mesh_conversions
import std_msgs
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
import sensor_msgs
import tf
import control_msgs.msg


def set_gripper_width(position, effort):

    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = effort

    client = actionlib.SimpleActionClient("/gripper_controller/gripper_action", control_msgs.msg.GripperCommandAction)
    client.wait_for_server(rospy.Duration(3))
    client.send_goal_and_wait(goal, rospy.Duration(5))
    result = client.get_result()

    return result


def open_gripper():
    set_gripper_width(0.05, 100)


def close_gripper():
    set_gripper_width(0.0, 100)


def home_arm(mgc):
    rospy.loginfo("home_arm")
    mgc.set_named_target('home')
    p = mgc.plan()

    return mgc.execute(p)


def get_grasps_from_graspit_sim_ann(
        mesh_filepath,
        target_object_pose,
        search_energy,
        max_steps,
        robot="fetch_gripper",
        obstacle_info=[]):

    rospy.loginfo("get_grasps_from_graspit_sim_ann")
    gc = graspit_commander.GraspitCommander()
    gc.clearWorld()
    gc.importRobot(robot)
    gc.importGraspableBody(mesh_filepath, target_object_pose)

    if obstacle_info:
        for each_object in obstacle_info:
            object_filepath = each_object['file_path']
            # obstacle_mesh_filepath = os.path.join(args.mesh_root, object_name)
            object_pose = each_object['pose']
            object_pose_in_graspit = set_object_pose(object_pose)
            gc.importObstacle(object_filepath, object_pose_in_graspit)

    grasp_results = gc.planGrasps(search_energy=search_energy, max_steps=max_steps)

    return grasp_results


def get_pick_plan(mgc_arm, graspit_interface_grasp_msg, grasp_frame_id):
    moveit_grasp_msg = graspit_interface_to_moveit_grasp(graspit_interface_grasp_msg, grasp_frame_id)

    # TODO Move to config
    planner_id = "arm" + "[RRTConnectkConfigDefault]"
    allowed_planning_time = 5

    pickup_goal = build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                    object_name=grasp_frame_id,
                                    allowed_planning_time=allowed_planning_time,
                                    planner_id=planner_id,
                                    planning_group=mgc_arm)

    import IPython
    IPython.embed()

    success, result = send_pick_request(pickup_goal)

    return success, result


def send_pick_request(pickup_goal):
    allowed_planning_time = rospy.get_param('reachability_analyzer/allowed_planning_time') + 3

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


def execute_pickup_plan(mgc_arm, mgc_gripper, pick_plan, object_name):

    # pick_plan.trajectory_descriptions
    # ['plan', 'pre_grasp', 'approach', 'grasp', 'retreat']

    for i in range(5):

        if i % 2 == 0:
            success = mgc_arm.execute(pick_plan.trajectory_stages[i])
        elif i == 3:
            success = mgc_gripper.execute(pick_plan.trajectory_stages[i])
            # close_gripper()
            # starting_jv = mgc_gripper.get_current_joint_values()
            # for x in range(3):
            #     success = mgc_gripper.execute(pick_plan.trajectory_stages[i])
            #     jv = mgc_gripper.get_current_joint_values()
            #     if not np.allclose(jv, starting_jv):
            #         break

        else:
            success = mgc_gripper.execute(pick_plan.trajectory_stages[i])

        if not success:
            # if a stage of the trajectory is not successful
            break
        rospy.sleep(1)
    return success