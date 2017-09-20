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
import fetch_skills
import moveit_commander


class Profiler(object):
    def __init__(self):
        self._start_times = {}
        self._end_times = {}

    def start(self, timer_name):
        self._start_times[timer_name] = time.time()

    def end(self, timer_name):
        self._end_times[timer_name] = time.time()

    def collect(self):
        keys = set(self._start_times.keys()) & set(self._end_times.keys())
        results = {}
        for key in keys:
            results[key + "_time"] = self._end_times[key] - self._start_times[key]

        return results


def pose_to_transform_array(pose):
    frame = tf_conversions.fromMsg(pose)
    transform = tf_conversions.toMatrix(frame)
    return transform


def add_object_to_planning_scene(mesh_name, mesh_filepath, pose_stamped, add_to_planning_scene=True):
    wm_add_service_proxy = rospy.ServiceProxy("/world_manager/add_object",
                                              world_manager.srv.AddObject)
    wm_add_service_proxy.wait_for_service()
    wm_add_service_proxy(mesh_name, mesh_filepath, pose_stamped, add_to_planning_scene)


def fix_grasps_for_moveit(grasp_results, object_pose_in_world):
    grasps_for_moveit = []
    for g_original in grasp_results.grasps:
        g = copy.deepcopy(g_original)
        g.approach_direction.vector = Vector3(1, 0, 0)
        g.dofs = [0.0, 0.0]
        g.pose = transform_pose(g_original.pose, object_pose_in_world)
        grasps_for_moveit.append(g)

    return grasps_for_moveit


def transform_pose(grasp_pose_in_world, object_pose_in_world):
    grasp_pose_in_world_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(grasp_pose_in_world))
    object_pose_in_world_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(object_pose_in_world))
    grasp_pose_in_object_tran_matrix = np.dot(np.linalg.inv(object_pose_in_world_tran_matrix),
                                              grasp_pose_in_world_tran_matrix)
    grasp_pose_in_object = tf_conversions.toMsg(tf_conversions.fromMatrix(grasp_pose_in_object_tran_matrix))

    return grasp_pose_in_object


def write_results(args, result_info):

    if not os.path.exists(args.result_dir):
        os.makedirs(args.result_dir)
        shutil.copy(__file__, args.result_dir)

    yaml.dump(result_info, open(args.result_filepath, 'w'))


def get_args():
    parser = argparse.ArgumentParser(description='Grasp planning with reachability')
    parser.add_argument('--mesh_root', type=str,
                        default=os.path.join(rospkg.RosPack().get_path('reachability_experiments'), 'meshes'))
    # parser.add_argument('--reachability_config_dir', type=str,
    #                     default=os.path.join(rospkg.RosPack().get_path('reachability_energy_plugin'), 'data'))
    # parser.add_argument('--search_energy', type=str,
    #                     default='REACHABLE_FIRST_HYBRID_GRASP_ENERGY')
    # parser.add_argument('--max_steps', type=int,
    #                     default=70000)
    parser.add_argument('--object_name', type=str,
                        default='object_0')
    parser.add_argument('--mesh_filename', type=str,
                        default='frenchs_classic_yellow_mustard_14oz.ply')

    args = parser.parse_args()

    args.obj_pose_filename = os.path.join(args.reachability_config_dir, 'object_pose_in_reference_frame.csv')
    args.mesh_filepath = os.path.join(args.mesh_root, args.mesh_filename)
    args.result_filepath = os.path.join(args.result_dir, "result.yaml")

    object_ps = PoseStamped()
    object_ps.header.frame_id = "root"
    object_ps.pose =  Pose(Point(0.841788947582, -0.0385499969125, 0.706323564053), Quaternion(3.62054478669e-08, -3.92341219469e-08, 0.278758395564, 0.960361263745))
    args.object_ps = object_ps

    return args

def main():
    rospy.init_node("graspit_to_moveit_demo")
    args = get_args()

    gc = graspit_commander.GraspitCommander()
    mgc_arm = moveit_commander.MoveGroupCommander('arm')
    mgc_gripper = moveit_commander.MoveGroupCommander("gripper")

    skills.open_gripper(mgc_gripper)
    skills.home_arm(mgc_arm)

    add_object_to_planning_scene(args.object_name, args.mesh_filepath, args.object_ps, True)

    # get grasps from graspit
    prof.start("graspit")
    grasp_results = fetch_skills.get_grasps_from_graspit_sim_ann(
        mesh_filepath=args.mesh_filepath,
        target_object_pose=args.object_ps.pose,
        search_energy=args.search_energy,
        max_steps=args.max_steps,
        robot="fetch_gripper",
        obstacle_info=[])
    prof.end("graspit")

    # check grasps for reachability
    grasps_for_moveit = fix_grasps_for_moveit(grasp_results, args.object_ps.pose)

    # execute grasps
    prof.start("moveit")
    for i, moveit_grasp in enumerate(grasps_for_moveit):
        prof.start("moveit" + str(i))
        success, pick_plan = fetch_skills.get_pick_plan(mgc_arm, moveit_grasp, args.object_name)
        prof.end("moveit" + str(i))

        if success:
            prof.end("moveit")
            result_info["index_first_reachable"] = i
            prof.start("moveit_execution")
            success = fetch_skills.execute_pickup_plan(mgc_arm, mgc_gripper, pick_plan, args.object_name)
            prof.end("moveit_execution")
            break

    while True:
        grasp_success = raw_input("Was the grasp successfull: 'y' / 'n' / 'g' for garbage \t")
        if grasp_success == 'y':
            result_info['grasp_success'] = True
            break
        elif grasp_success == 'n':
            result_info['grasp_success'] = False
            break
        elif grasp_success == 'g':
            result_info['grasp_success'] = None
            break
        else:
            print "must enter 'y' or 'n'"

    timer_results = prof.collect()
    result_info.update(timer_results)
    result_info.update(args.__dict__)

    write_results(args, result_info)


if __name__ == '__main__':
    main()