#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Point, Quarternion
import pandas as pd


path = Path()
df_TM = pd.read_csv('./file2.csv')
def get_csv():
	df_list = []
#	df_orientation_list = []
	df_list.append(df_TM['field.header.stamp'])
	df_list.append(df_TM['field.pose.pose.position.x'])
	df_list.append(df_TM['field.pose.pose.position.y'])
	df_list.append(df_TM['field.pose.pose.position.z'])

	df_list.append(df_TM['field.pose.pose.orientation.x'])
	df_list.append(df_TM['field.pose.pose.orientation.y'])
	df_list.append(df_TM['field.pose.pose.orientation.z'])
	df_list.append(df_TM['field.pose.pose.orientation.w'])
	return df_list


'''
def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)
'''
def callback():
	rospy.init_node('path_node')
	P = PoseStamped()
	P.header.frame_id = "odom"
	# path_pub = rospy.Publisher('/path', Path, queue_size=10)
	path_pub = rospy.Publisher('/path', PoseStamped, queue_size=10)
	# rate = rospy.Rate(1)
	pose_list = get_csv()
	# print("aaaaaaa",len(pose_list))
	# print(len(df_TM['field.header.stamp']))
	# for i in range(len(pose_list)):
	# 	for j in range(len(df_TM['field.header.stamp'])):
	# 		print(pose_list[i][j])
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	# path.header.stamp = current_time
	P.header.stamp.secs = current_time
	while not rospy.is_shutdown():
		this_pose = PoseStamped()

		for i in range(len(df_TM['field.header.stamp'])):
			# path.header.stamp = pose_list[0][i]
			this_pose.pose.position.x = pose_list[1][i]
			this_pose.pose.position.y = pose_list[2][i]
			this_pose.pose.position.z = pose_list[3][i]
			this_pose.pose.orientation.x = pose_list[4][i]
			this_pose.pose.orientation.y = pose_list[5][i]
			this_pose.pose.orientation.z = pose_list[6][i]
			this_pose.pose.orientation.w = pose_list[7][i]

			#path.poses.append(this_pose)
			# path_pub.publish(path)
			path_pub.publish(this_pose)
			# path_pub.publish(P)

	rospy.spin()
	#rospy.sleep()



if __name__ == '__main__':
	callback()
    # rospy.spin()
	#rospy.sleep()
