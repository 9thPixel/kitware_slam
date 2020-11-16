'''
to use this file :
-LD_LIBRARY_PATH must contain "Path_superbuild/build/install/lib/lidarview-3.6:Path_superbuild/build/install/lib/paraview-5.4"
-PYTHONPATH must contain "Path_superbuild/build/install/lib/lidarview-3.6/site_packages:Path_superbuild/build/install/lib/paraview-5.4/site_packages:Path_superbuild/build/install/lib/paraview-5.4/site_packages/vtk"
-3 folders must exist:
    -calibrations : with the calibration files to use
    -references : with SLAM reference results of data
    -data : containing pcap data
    -test_output : where SLAM tests results were saved
'''

import subprocess

class data:
    # Class corresponding to datasets to test
    def __init__(self, fn):
        self.File_name = fn
        self.File_name_base = self.File_name.split('.')[0]
        self.File_path = "data/" + self.File_name
        self.Reference_result = "references/" + self.File_name_base + ".poses"
        self.Calibration_file = "calibrations/VLP-16.xml"
        self.Interpreter = "Velodyne Meta Interpreter"
        self.Output_file = "test_output/" + self.File_name_base + ".poses"
        self.Loop_closure_file = "loop_closing_transforms/"+self.File_name_base + ".txt"
        self.Reference_times_file = "references/" + self.File_name_base + ".times"
        self.Output_times_file = "test_output/" + self.File_name_base + ".times"

#-----------------------------------------------------------------------------------------------------

#initializing datasets to test------------------------------------------------------------------------
data1 = data("car_loop.pcap")
data2 = data("forest_walking.pcap")
data_set = [data1, data2]
#-----------------------------------------------------------------------------------------------------

for d in data_set:
    with open(d.Output_times_file, "w+") as output:
        subprocess.call(["venv/bin/python2.7", "./SLAM_run.py", d.File_path, d.Interpreter, d.Calibration_file, d.Output_file], stdout=output)

    subprocess.call(["venv/bin/python2.7", "./SLAM_evaluate.py", d.Output_file, d.Reference_result, d.Loop_closure_file, d.Output_times_file, d.Reference_times_file])
