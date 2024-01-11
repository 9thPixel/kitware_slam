import argparse
import os
import re

def extract_arr(file):
    # Regex patterns
    pattern_kp_extraction = r"\[.*?\] *-> Localization : map keypoints extraction took : (\d+\.\d+) ms"
    pattern_whole_ICP = r"\[.*?\] *-> Localization : whole ICP-LM loop took : (\d+\.\d+) ms"

    time_kp_extraction = []
    time_whole_ICP = []

    with open(file, "r") as log_file:
        for line in log_file:
           match = re.search(pattern_kp_extraction, line)
           if match:
              time_kp_extraction.append(float(match.group(1)))
           match = re.search(pattern_whole_ICP, line)
           if match:
              time_whole_ICP.append(float(match.group(1)))
    return (time_kp_extraction, time_whole_ICP)

def Average(lst): 
    return sum(lst) / len(lst)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(
        prog=os.path.basename(__file__), description="Give localization time from a logfile of the slam node"
    )
    parser.add_argument("file1", type=str, help="first file to compare")
    parser.add_argument("file2", type=str, help="second file to compare")
    args = parser.parse_args()

    (file1_kp_extract_arr, file1_ICP_arr) = extract_arr(args.file1)
    (file2_kp_extract_arr, file2_ICP_arr) = extract_arr(args.file2)

    print(f"******* Diff {args.file1} - {args.file2} *******")
    print(f"For each frame : ")
    for i in range(min(len(file1_kp_extract_arr), len(file2_kp_extract_arr))):
       print(f"difference kdtree construction time : {file1_kp_extract_arr[i] - file2_kp_extract_arr[i]}ms")
       print(f"difference whole ICP time : {file1_ICP_arr[i] - file2_ICP_arr[i]}ms")

    print(f"Average diff : ")
    print(f"average kdtree construction time : {Average(file1_kp_extract_arr) - Average(file2_kp_extract_arr)}ms")
    print(f"average whole ICP time : {Average(file1_ICP_arr) - Average(file2_ICP_arr)}ms")


if __name__== "__main__":
  main()