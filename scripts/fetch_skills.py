#!/usr/bin/env python

import graspit_commander
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from reachability_analyzer.message_utils import build_pickup_goal, graspit_interface_to_moveit_grasp
import moveit_msgs.msg
import actionlib
import curvox.mesh_conversions
import std_msgs
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
import sensor_msgs
import tf
import control_msgs.msg


def align_gt_object(gt_object_filepath, downsample_factor=10, rate=5):
    ply = curvox.mesh_conversions.read_mesh_msg_from_ply_filepath(gt_object_filepath)
    points = []
    for i in range(len(ply.vertices)):
        if i % downsample_factor == 0:
            points.append((ply.vertices[i].x, ply.vertices[i].y, ply.vertices[i].z))

    header = std_msgs.msg.Header()

    header.frame_id = '/gt_object'
    pcl_arr = pcl2.create_cloud_xyz32(header, np.array(points))

    pc_pub = rospy.Publisher('/gt_overlay', sensor_msgs.msg.PointCloud2, queue_size=10)

    r = rospy.Rate(rate)
    tfl = tf.TransformListener()

    for i in range(1000):
        tfl.waitForTransform("/base_link", "/gt_object", rospy.Time(0), rospy.Duration(4))
        tran, rot = tfl.lookupTransform("/base_link", "/gt_object", rospy.Time(0))

        print '<node pkg="lcsr_tf_tools" type="interactive_transform_publisher" name="static_tf_world_to_gt" args="{} {} {} {} {} {} {} /base_link /gt_object 100"/>'.format(
            tran[0], tran[1], tran[2], rot[0], rot[1], rot[2], rot[3])
        print 'object_ps.pose = Pose(Point({}, {}, {}), Quaternion({}, {}, {}, {}))'.format(tran[0], tran[1], tran[2],
                                                                                            rot[0], rot[1], rot[2],
                                                                                            rot[3])
        pc_pub.publish(pcl_arr)
        r.sleep()


def set_gripper_width_moveit(mgc, val=0.05):
    jv = [val, val]
    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)
    rospy.sleep(2)


def open_gripper_moveit(mgc, val=0.05):
    set_gripper_width_moveit(mgc, val)


def close_gripper_moveit(mgc, val=0.0):
    set_gripper_width_moveit(mgc, val)


def set_gripper_width(position, effort):

    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = effort

    client = actionlib.SimpleActionClient("/gripper_controller/gripper_action", control_msgs.msg.GripperCommandAction)
    client.wait_for_server()
    # client.cancel_all_goals()
    rospy.sleep(3)
    client.send_goal_and_wait(goal, rospy.Duration(10))
    result = client.get_result()
    print result
    rospy.sleep(3)
    return result


def open_gripper():
    set_gripper_width(0.05, 100)


def close_gripper():
    set_gripper_width(0.0, 100)


def home_arm(mgc):
    mgc.set_named_target('home')
    p = mgc.plan()

    return mgc.execute(p)


# input: [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]
def set_object_pose(object_pose):
    object_pose_in_graspit = Pose()
    pos = Point(object_pose[0], object_pose[1], object_pose[2])
    ori = Quaternion(object_pose[3], object_pose[4], object_pose[5], object_pose[6])
    object_pose_in_graspit.position = pos
    object_pose_in_graspit.orientation = ori
    return object_pose_in_graspit


def get_grasps_from_graspit_sim_ann(
        mesh_filepath,
        target_object_pose,
        search_energy,
        max_steps,
        robot="fetch_gripper",
        obstacle_info=[]):
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

    planner_id = "arm" + rospy.get_param('reachability_analyzer/planner_config_name')
    allowed_planning_time = rospy.get_param('reachability_analyzer/allowed_planning_time')

    pickup_goal = build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                    object_name=grasp_frame_id,
                                    allowed_planning_time=allowed_planning_time,
                                    planner_id=planner_id,
                                    planning_group=mgc_arm)

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