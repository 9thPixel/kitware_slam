import csv
import sys

def open_csv(filename):
    file = open(filename, "r")
    csvreader = csv.reader(file)
    arr = []
    for row in csvreader:
        arr.append(row)
    if len(arr) == 0:
        print("empty data")
        exit(1)

    return arr

def main():
    data = [[float(timestamp) for timestamp in result[0:2]] for result in open_csv(sys.argv[1])]
    size_data = len(data)
    dict_results = {}
    # Use first timestamp as the reference
    ref_second = data[0][0]

    for timestamp in data:
        result = round((timestamp[0] - ref_second) + timestamp[1] * 1e-9, 4)
        if (dict_results.get(result) == None):
            dict_results[result] = 1
        else:
            dict_results[result] += 1

    print(f"Numbre timestamp : {len(dict_results)}")
    for key in dict_results.keys():
        print(f"relative time {key}, occurences {dict_results[key]}")

    print(f"Correct timestamp : {max(dict_results.values()) / size_data * 100}%")

if __name__ == "__main__":
    main()