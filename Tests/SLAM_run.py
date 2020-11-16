'''
to use this file :
-LD_LIBRARY_PATH must contain "Path_superbuild/build/install/lib/lidarview-3.6:Path_superbuild/build/install/lib/paraview-5.4"
-PYTHONPATH must contain "Path_superbuild/build/install/lib/lidarview-3.6/site_packages:Path_superbuild/build/install/lib/paraview-5.4/site_packages:Path_superbuild/build/install/lib/paraview-5.4/site_packages/vtk"
-3 folders must exist:
    -calibrations : with the calibration files to use
    -data : with pcap data
    -test_output : where the results will be saved
-Arguments of this script :
    -Input_file_path : Path to the input pcap file
    -Interpreter : Interpreter Name
    -Calibration_file_path' : Path to the calibration file
    -Output_file_path : 'Path of file where to save the output of SLAM (poses)
'''

from paraview import simple as smp
smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libLidarPlugin.so', remote=False)
smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/plugins/libLidarSlamPlugin.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libLidarPluginPythonD.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/LidarPluginPython.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libVelodynePlugin.so', remote=False)
# smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/libVelodynePluginPythonD.so', remote=False)
smp.LoadPlugin('/home/julia/Desktop/my_programs/LidarView/LidarView-build/install/lib/lidarview-3.6/VelodynePluginPython.so', remote=False)

import argparse

parser = argparse.ArgumentParser(description='Process poses files')
parser.add_argument('Input_file_path', metavar='path', type=str, help='Path to the input pcap file')
parser.add_argument('Interpreter', type=str, help='Interpreter Name')
parser.add_argument('Calibration_file_path', metavar='path', type=str, help='Path to the calibration file')
parser.add_argument('Output_file_path', metavar='path', type=str, help='Path of file where to save the output of SLAM (poses)')
args = parser.parse_args()
# Initialize the reader with the pcap and the calibration file------------------------------------
reader = smp.LidarReader()
reader.FileName = args.Input_file_path
reader.Interpreter = args.Interpreter
reader.CalibrationFile = args.Calibration_file_path
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
slam.Verbositylevel = '1) 0 + Frame total processing duration'
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
smp.SaveData(args.Output_file_path, proxy=smp.OutputPort(slam, 1))
#--------------------------------------------------------------------------------------------------