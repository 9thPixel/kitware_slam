'''
to use this file :
-LD_LIBRARY_PATH must contain "Path_superbuild/build/install/lib/lidarview-3.6:Path_superbuild/build/install/lib/paraview-5.4"
-PYTHONPATH must contain "Path_superbuild/build/install/lib/lidarview-3.6/site_packages:Path_superbuild/build/install/lib/paraview-5.4/site_packages:Path_superbuild/build/install/lib/paraview-5.4/site_packages/vtk"
-3 folders must exist:
    -calibrations : with the calibration files to use
    -references : with SLAM reference results of data
    -data : with pcap data
    -test_output : where the results will be saved
'''

from paraview import simple as smp
smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libLidarPlugin.so', remote=False)
smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/plugins/libLidarSlamPlugin.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libLidarPluginPythonD.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/LidarPluginPython.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libVelodynePlugin.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libVelodynePluginPythonD.so', remote=False)
smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/VelodynePluginPython.so', remote=False)

import csv
import numpy as np
import os.path
import math

#Parameters for evaluation----------------------------------------------------------------------------
threshold_trans_pose_rel = 0.01 # 1cm
threshold_rot_pose_rel = 1*np.pi/180 # 1deg
#-----------------------------------------------------------------------------------------------------

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
            print("Error : the new version misses the pose at t = {} from dataset {}".format(p1[idx_ref][0], d.File_name))
            failed_previous = True
            idx_ref += 1
            pb_rate += 1.0
        elif(pose_ref[0] - pose_res[0]>1e-6):
            print("Wirdness : the new version computes one more pose at t = {} from dataset {}".format(p2[idx_res][0], d.File_name))
            idx_res += 1
            pb_rate += 1.0
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
                    print("Error : bad registration of scan from dataset {} at t = {}".format(d.File_name, pose_res[0]))
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
    pb_rate /= n_diff

    return diff, diff_rel, pb_rate

#-----------------------------------------------------------------------------------------------------

class data:
    # Class corresponding to datasets to test
    def __init__(self, fn):
        self.File_name = fn
        self.File_path = "data/" + self.File_name
        self.Reference_result = "references/" + self.File_name.split('.')[0] + ".poses"
        self.Calibration_file = "calibrations/VLP-16.xml"
        self.Interpreter = "Velodyne Meta Interpreter"
        self.Output_file = "test_output/" + self.File_name.split('.')[0] + ".poses"
        self.Loop_closure_file="loop_closing_transforms/"+self.File_name.split('.')[0] + ".txt"

#-----------------------------------------------------------------------------------------------------


#initializing datasets to test------------------------------------------------------------------------
data1 = data("car_loop.pcap")
data2 = data("forest_walking.pcap")
data_set = [data1, data2]
#-----------------------------------------------------------------------------------------------------

for d in data_set:
    # Initialize the reader with the pcap and the calibration file------------------------------------
    reader = smp.LidarReader()
    reader.FileName = d.File_path
    reader.Interpreter = d.Interpreter
    reader.CalibrationFile = d.Calibration_file
    reader.Interpreter.IgnoreZeroDistances = 1
    reader.Interpreter.IgnoreEmptyFrames = 1
    reader.Interpreter.EnableAdvancedArrays = 1
    reader.ShowFirstAndLastFrame = 1
    smp.Show(reader)
    reader.UpdatePipeline()
    reader.UpdatePipelineInformation()
    renderView1 = smp.GetActiveViewOrCreate('RenderView')
    #--------------------------------------------------------------------------------------------------

    # Create the SLAM filter---------------------------------------------------------------------------
    slam = smp.SLAMonline(PointCloud=reader, Calibration=smp.OutputPort(reader, 1))
    slam.Resetstate()
    # set SLAM filter parameters
    # slam.Keypointsextractor = 'Spinning Sensor Keypoint Extractor'
    # slam.NbedgesneighborsminimumafterRansac = 4
    # slam.Initlossscale = 0.7
    # slam.Finallossscale = 0.05
    # Init animation linked with SLAM filter
    animationScene = smp.GetAnimationScene()
    animationScene.UpdateAnimationUsingDataTimeSteps()
    #--------------------------------------------------------------------------------------------------

    # Apply and play-----------------------------------------------------------------------------------
    smp.Show(smp.OutputPort(slam, 1), renderView1)
    renderView1.Update()
    animationScene.Play()
    #--------------------------------------------------------------------------------------------------

    # save data----------------------------------------------------------------------------------------
    smp.SaveData(d.Output_file, proxy=smp.OutputPort(slam, 1))
    #--------------------------------------------------------------------------------------------------

    # Check results------------------------------------------------------------------------------------

    #load files
    poses_res = FileToList(d.Output_file)
    poses_ref = FileToList(d.Reference_result)

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
    diff, diff_rel, pb_rate = Evaluate(poses_res, poses_ref)
    # ---------------------------------------------------------------------------------------------------

    # Compare with loop closure if it exists-------------------------------------------------------------
    error_lp_res = 0
    error_lp_ref = 0
    if os.path.exists(d.Loop_closure_file):
        transfo = ReadTransfo(d.Loop_closure_file)
        error_lp_res = CompareToLoopClosure(poses_res[-1][1:], transfo)
        error_lp_ref = CompareToLoopClosure(poses_ref[-1][1:], transfo)
    # ---------------------------------------------------------------------------------------------------

    # Display results------------------------------------------------------------------------------------
    if (pb_rate > 0):
        print("-------------------Test failed-----------------------")
    else:
        print("-------------------Test succeeded-------------------")
    print("")
    print("Mean Squared Error element to element between absolute poses: {}m, {}m, {}m, {}d, {}d, {}d".format(diff[3], diff[4], diff[5], diff[0] * 180 / np.pi, diff[1] * 180 / np.pi, diff[2] * 180 / np.pi))
    print("Mean Squared Error element to element between relative poses: {}m, {}m, {}m, {}d, {}d, {}d".format(diff_rel[3], diff_rel[4], diff_rel[5], diff_rel[0] * 180 / np.pi, diff_rel[1] * 180 / np.pi, diff_rel[2] * 180 / np.pi))
    print("Percentage of notable poses difference : {}%".format(pb_rate * 100))
    if os.path.exists(d.Loop_closure_file):
        print("Loop closing distance for reference <-> drift : ")
        print("\t -translation : {}m".format(np.linalg.norm(error_lp_ref[3:])))
        print("\t -rotation : {}deg".format(np.abs(error_lp_ref[:3]).max()))
        print("Loop closing distance for new output <-> drift: ")
        print("\t -translation : {}m".format(np.linalg.norm(error_lp_res[3:]) * 180 / np.pi))
        print("\t -rotation : {}deg".format(np.abs(error_lp_res[:3]).max() * 180 / np.pi))

    # Delete display elements-----------------------------------------------------------------------------
    smp.Delete(slam)
    smp.Delete(renderView1)
    smp.Delete(animationScene)
    smp.Delete(reader)
    # -----------------------------------------------------------------------------------------------------