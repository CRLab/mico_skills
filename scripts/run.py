#!/usr/bin/env python
import numpy as np
import os
import copy
import time
import argparse
import yaml
import shutil

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
import tf_conversions

import world_manager.srv
import graspit_commander
import skills
import moveit_commander
import sys


def pose_to_transform_array(pose):
    # TODO: add to Curvox
    frame = tf_conversions.fromMsg(pose)
    transform = tf_conversions.toMatrix(frame)
    return transform


def add_object_to_planning_scene(mesh_name, mesh_filepath, pose_stamped, add_to_planning_scene=True):
    ''' add_to_planning scene determines whether to add object or just add tf '''
    # print >> sys.stderr, 'spam'
    wm_add_service_proxy = rospy.ServiceProxy("/world_manager/add_object",
                                              world_manager.srv.AddObject)
    wm_add_service_proxy.wait_for_service(timeout=1)
    rospy.loginfo("Waited for service")
    wm_add_service_proxy(mesh_name, mesh_filepath, pose_stamped, add_to_planning_scene)


def fix_grasps_for_moveit(grasp_results, world_frame_to_object_frame_transform):
    ''' world_to_object_transform is equivalent to object pose in world '''
    rospy.loginfo("fix_grasps_for_moveit")
    grasps_for_moveit = []
    for grasp_initial in grasp_results.grasps:
        grasp_final = copy.deepcopy(grasp_initial)
        grasp_final.approach_direction.vector = Vector3(1, 0, 0)
        grasp_final.dofs = [0.0, 0.0]

        # Transform grasp from root frame to object frame
        grasp_final.pose = transform_pose(grasp_initial.pose, world_frame_to_object_frame_transform)
        grasps_for_moveit.append(grasp_final)

    return grasps_for_moveit


def transform_pose(grasp_pose_in_world, world_frame_to_object_frame_transform):

    # Convert ROSmsg to 4x4 numpy arrays
    grasp_pose_in_world_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(grasp_pose_in_world))
    object_pose_in_world_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(world_frame_to_object_frame_transform))

    # Transform the grasp pose into the object reference frame
    grasp_pose_in_object_tran_matrix = np.dot(np.linalg.inv(object_pose_in_world_tran_matrix),
                                              grasp_pose_in_world_tran_matrix)

    # Convert pose back into ROS message
    grasp_pose_in_object = tf_conversions.toMsg(tf_conversions.fromMatrix(grasp_pose_in_object_tran_matrix))

    return grasp_pose_in_object


def get_args():
    parser = argparse.ArgumentParser(description='Grasp planning with reachability')
    parser.add_argument('--mesh_root', type=str,
                        default=os.path.join(rospkg.RosPack().get_path('mesh_models'), 'meshes'))
    # parser.add_argument('--reachability_config_dir', type=str,
    #                     default=os.path.join(rospkg.RosPack().get_path('reachability_energy_plugin'), 'data'))
    parser.add_argument('--search_energy', type=str,
                        default='GUIDED_POTENTIAL_QUALITY_ENERGY')
    parser.add_argument('--max_steps', type=int,
                        default=40000)
    parser.add_argument('--object_name', type=str,
                        default='object_0')
    parser.add_argument('--mesh_filename', type=str,
                        default='frenchs_classic_yellow_mustard_14oz.ply')

    args = parser.parse_args()

    args.mesh_filepath = os.path.join(args.mesh_root, args.mesh_filename)

    object_ps = PoseStamped()
    object_ps.header.frame_id = "root"
    object_ps.pose =  Pose(Point(0.241788947582, -0.0385499969125, 0.1), Quaternion(3.62054478669e-08, -3.92341219469e-08, 0.278758395564, 0.960361263745))
    args.object_ps = object_ps

    return args

def main():
    rospy.init_node("graspit_to_moveit_demo")
    args = get_args()

    gc = graspit_commander.GraspitCommander()
    mgc_arm = moveit_commander.MoveGroupCommander('arm')
    mgc_gripper = moveit_commander.MoveGroupCommander("gripper")

    # skills.open_gripper()
    # skills.home_arm(mgc_arm)

    add_object_to_planning_scene(args.object_name, args.mesh_filepath, args.object_ps, True)

    # get grasps from graspit
    grasp_results = skills.get_grasps_from_graspit_sim_ann(
        mesh_filepath=args.mesh_filepath,
        target_object_pose=args.object_ps.pose,
        search_energy=args.search_energy,
        max_steps=args.max_steps,
        robot="MicoGripper",
        obstacle_info=[])

    # check grasps for reachability
    grasps_for_moveit = fix_grasps_for_moveit(grasp_results, args.object_ps.pose)

    # import IPython
    # IPython.embed()

    # execute grasps
    for i, moveit_grasp in enumerate(grasps_for_moveit):
        success, pick_plan = skills.get_pick_plan(mgc_arm, moveit_grasp, args.object_name)

        if success:
            result_info["index_first_reachable"] = i
            success = skills.execute_pickup_plan(mgc_arm, mgc_gripper, pick_plan, args.object_name)
            break


if __name__ == '__main__':
    main()