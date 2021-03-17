#!/usr/bin/env python
import csv, sys, os


def get_pose(skip_header=True):
    dirctory = os.path.dirname(os.path.realpath(__file__))
    path = dirctory + '/path/s1_new4waypoint_carmaker.csv'
    # path = dirctory + '/path/s2_new4waypoint_carmaker.csv'
    # path = dirctory + '/path/s3_new4waypoint_carmaker.csv'
    # path = dirctory + '/path/s4_new4waypoint_carmaker.csv'


    # print(my_path)
    with open(path) as csvfile:
        csvreader = csv.reader(csvfile)
        headers = None
        if skip_header: headers = next(csvreader, None)
        rows = []
        for row in csvreader:
            rows.append(row)

    return rows, headers




# if __name__ == '__main__':
#     try:
#         get_pose()
#     except rospy.ROSInterruptException: pass
