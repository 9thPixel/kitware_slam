'''
to use this file :
-3 folders must exist:
    -references : with SLAM reference results of data
    -data : with pcap data
    -test_output : where the results will be saved
-Arguments of this script :
    -Input_file_path : Path to Output results of new tested version
    -Reference_file_path : Path to the reference results (of master version)
    -Loop_closure_transfo_file_path : Path to the file containing the loop closure transformation matrix
'''

import csv
import numpy as np
import os.path
import math
import argparse

#Parameters for evaluation----------------------------------------------------------------------------
threshold_trans_pose_rel = 0.01 # 1cm
threshold_rot_pose_rel = 1*np.pi/180 # 1deg
#-----------------------------------------------------------------------------------------------------

def FileToTimes(file_name):
    # read SLAM log file and extract processing times (in log : [...] vtkslam took : 10.032 ms [...]
    proc_times = []
    with open(file_name, 'r') as f:
        lines = f.readlines()
        for l in lines:
            nl = l.find("vtkSlam t")
            if nl != -1:
                proc_times.append(float(l[nl + 15: nl + 15 + 6])) # we suppose 6 digit are sufficient to have precise time
    return proc_times


def FileToList(file_name):
    # Read csv file containing SLAM results and put it to a list of poses
    first = 1
    poses = []
    with open(file_name, mode='r') as f_res:
        reader_res = csv.reader(f_res, delimiter=',')
        for row in reader_res:
            if (first):
                first = 0
                continue
            pose = np.zeros(len(row))
            for j in range(len(row)):
                pose[j] = float(row[j])
            poses.append(pose)
    return poses

#-----------------------------------------------------------------------------------------------------

def ReadTransfo(file_name):
    # Read a file with a saved transform corresponding to an ICP output of loop closure
    transfo = np.zeros((4, 4))
    f = open(file_name)
    lines = f.readlines()
    for i in range(len(lines)):
        values = lines[i].split(' ')
        for j in range(len(values)):
            transfo[i][j] = values[j]
    return transfo

#-----------------------------------------------------------------------------------------------------

def Transfo2RYPXYZ(transfo):
    # Convert 4*4 transform to pose vector
    rx = math.atan2(transfo[2][1], transfo[2][2])
    ry = -math.asin(transfo[2][0])
    rz = math.atan2(transfo[1][0], transfo[0][0])
    tx = transfo[0][3]
    ty = transfo[1][3]
    tz = transfo[2][3]
    return np.array([rx, ry, rz, tx, ty, tz])

#-----------------------------------------------------------------------------------------------------

def CompareToLoopClosure(pose, transfo):
    pose_lp = Transfo2RYPXYZ(transfo)
    return pose - pose_lp

#-----------------------------------------------------------------------------------------------------

def Evaluate(p1, p2):
    # Comparison between master and new version on a dataset.
    # p1 is a list of poses from new version,
    # p2 is a list of poses from master version.
    diff = np.zeros(6)
    diff_rel = np.zeros(6)
    n_diff = 0
    n_diff_rel = 0
    idx_res = 0
    idx_ref = 0
    first = True
    pb_rate = 0.0
    no_computed = 0.0
    failed_previous = False
    while idx_ref < len(p1) and idx_res < len(p2): # all poses are browsed looking for time matches to compare
        pose_ref = p1[idx_ref]
        pose_res = p2[idx_res]
        if(pose_ref.sum()<1e-6):
            idx_ref += 1
            continue
        if (pose_res.sum() < 1e-6):
            idx_res += 1
            continue
        if(pose_ref[0] - pose_res[0]<-1e-6):
            print("Error : the new version misses the pose at t = {} from dataset {}".format(p1[idx_ref][0], dataset_name))
            failed_previous = True
            idx_ref += 1
            pb_rate += 1.0
            no_computed += 1.0
        elif(pose_ref[0] - pose_res[0]>1e-6):
            print("Wirdness : the new version computes one more pose at t = {} from dataset {}".format(p2[idx_res][0], dataset_name))
            idx_res += 1
        else:
            error = pose_ref[1:]-pose_res[1:]
            diff += np.square(error)
            if not(first) and not(failed_previous): # if first=True, it can not compute relative, if failed_previous=True, the relative will automatically not correspond to reference situation
                pose_ref_prev = p1[idx_ref-1]
                pose_res_prev = p2[idx_res-1]
                error_rel = (pose_ref[1:]-pose_ref_prev[1:]) - (pose_res[1:]-pose_res_prev[1:])
                diff_rel += np.square(error_rel)
                n_diff_rel += 1
                if np.linalg.norm(error_rel[3:]) > threshold_trans_pose_rel or np.abs(error_rel[0:3]).max() > threshold_rot_pose_rel:
                    print("Error : bad registration of scan from dataset {} at t = {}".format(dataset_name, pose_res[0]))
                    pb_rate += 1.0
            elif(first):
                first = False # reset boolean
            elif(failed_previous):
                failed_previous = False # reset boolean
            n_diff += 1
            idx_ref += 1
            idx_res += 1

    diff = np.sqrt(diff / float(n_diff))
    diff_rel = np.sqrt(diff_rel / float(n_diff_rel))
    pb_rate /= len(poses_ref)
    no_computed /= len(poses_ref)

    return diff, diff_rel, pb_rate, no_computed

#-----------------------------------------------------------------------------------------------------
parser = argparse.ArgumentParser(description='Process poses files')
parser.add_argument('Input_file_path', metavar='path', type=str, help='Path to Output results of new tested version')
parser.add_argument('Reference_file_path', metavar='path', type=str, help='Path to the reference results (of master version)')
parser.add_argument('Loop_closure_transfo_file_path', metavar='path', type=str, help='Path to the file containing the loop closure transformation matrix')
parser.add_argument('Input_times_file_path', metavar='path', type=str, help='Path to the file containing the new processing times')
parser.add_argument('Reference_times_file_path', metavar='path', type=str, help='Path to the file containing the reference times')

args = parser.parse_args()

dataset_name = os.path.basename(args.Input_file_path).split('.')[0]

#load files
poses_res = FileToList(args.Input_file_path)
poses_ref = FileToList(args.Reference_file_path)

# # introducing errors to test----------------------------------------------------------------------
# poses_res[30:] = [np.array([0, 0, 0, 0, 10, 0, 0]) + x for x in poses_res[30:]]
# poses_res[50:] = [np.array([0, 2 * np.pi / 180, 0, 0, 0, 0, 0]) + x for x in poses_res[50:]]
# N = 15
# poses_res = poses_res[:N] + poses_res[N + 1:]
# print("Introducing error at t={}".format(poses_res[30][0]))
# print("Introducing error at t={}".format(poses_res[50][0]))
# print("Introducing a lack at t={}".format(poses_res[N][0]))
# print("")
# print("")
# ---------------------------------------------------------------------------------------------------

# Compare between master reference and new version---------------------------------------------------
diff, diff_rel, pb_rate, not_computed = Evaluate(poses_res, poses_ref)
# ---------------------------------------------------------------------------------------------------

# Compare with loop closure if it exists-------------------------------------------------------------
error_lp_res = 0
error_lp_ref = 0
if os.path.exists(args.Loop_closure_transfo_file_path):
    transfo = ReadTransfo(args.Loop_closure_transfo_file_path)
    error_lp_res = CompareToLoopClosure(poses_res[-1][1:], transfo)
    error_lp_ref = CompareToLoopClosure(poses_ref[-1][1:], transfo)
    improvement = True
    for i in range(len(error_lp_res)):
        if np.linalg.norm(error_lp_res[3:]) < np.linalg.norm(error_lp_ref[3:]):
            print("New version may improve results on dataset {} : translation drift lowered".format(dataset_name))
# ---------------------------------------------------------------------------------------------------

time_has_changed = False
# Compare processing time----------------------------------------------------------------------------
proc_times_res = FileToTimes(args.Input_times_file_path)
proc_times_ref = FileToTimes(args.Reference_times_file_path)
mean_proc_time_res = np.array(proc_times_res).mean()
mean_proc_time_ref = np.array(proc_times_ref).mean()
if(abs(mean_proc_time_res - mean_proc_time_ref) > 100):
    time_has_changed = True
    print("Error : the new version is significantly slower")
# ---------------------------------------------------------------------------------------------------

# Display results------------------------------------------------------------------------------------
if (pb_rate > 0 or time_has_changed):
    print("-------------------Test failed-----------------------")
else:
    print("-------------------Test succeeded-------------------")
print("")
print("Comparison with master version : ")
print("\t Mean Squared Error element to element between absolute poses: {}m, {}m, {}m, {}d, {}d, {}d".format(diff[3], diff[4], diff[5], diff[0] * 180 / np.pi, diff[1] * 180 / np.pi, diff[2] * 180 / np.pi))
print("\t Mean Squared Error element to element between relative poses: {}m, {}m, {}m, {}d, {}d, {}d".format(diff_rel[3], diff_rel[4], diff_rel[5], diff_rel[0] * 180 / np.pi, diff_rel[1] * 180 / np.pi, diff_rel[2] * 180 / np.pi))
print("\t Percentage of notable different poses : {}%".format(pb_rate * 100))
print("\t Percentage of frame not processed : {}%".format(not_computed*100))
print("")
print("Mean processing times : ")
print("\t Reference : {}ms".format(mean_proc_time_ref))
print("\t New version : {}ms".format(mean_proc_time_res))
print("")
if os.path.exists(args.Loop_closure_transfo_file_path):
    print("Loop closing difference <-> drift : ")
    print("\t Reference : ")
    print("\t\t-Translation error : {}m".format(np.linalg.norm(error_lp_ref[3:])))
    print("\t\t -Rotation error : {}deg".format(np.abs(error_lp_ref[:3]).max() * 180 / np.pi))
    print("\t New version : ")
    print("\t\t -Translation error : {}m".format(np.linalg.norm(error_lp_res[3:])))
    print("\t\t -Rotation error: {}deg".format(np.abs(error_lp_res[:3]).max() * 180 / np.pi))