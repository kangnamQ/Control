#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Point, Quarternion
import pandas as pd



df_TM = pd.read_csv('./file2.csv')
def get_csv():
	# df_list = []
	df_tf = {}
	df_dic = {}
	key_name = ['index', 'x', 'y', 'z',
				'o_x','o_y','o_z','o_w']
	for key in key_name:
		df_dic[key] = []
		df_tf[key] = []


	df_tf['index']=df_TM['field.header.stamp'].to_dict()
	df_tf['x']=df_TM['field.pose.pose.position.x'].to_dict()
	df_tf['y']=df_TM['field.pose.pose.position.y'].to_dict()
	df_tf['z']=df_TM['field.pose.pose.position.z'].to_dict()
	df_tf['o_x']=df_TM['field.pose.pose.orientation.x'].to_dict()
	df_tf['o_y']=df_TM['field.pose.pose.orientation.y'].to_dict()
	df_tf['o_z']=df_TM['field.pose.pose.orientation.z'].to_dict()
	df_tf['o_w']=df_TM['field.pose.pose.orientation.w'].to_dict()

	df_dic['index'] = df_tf['index'].keys()
	df_dic['x'] = df_tf['x'].values()
	df_dic['y'] = df_tf['y'].values()
	df_dic['z'] = df_tf['z'].values()
	df_dic['o_x'] = df_tf['o_x'].values()
	df_dic['o_y'] = df_tf['o_y'].values()
	df_dic['o_z'] = df_tf['o_z'].values()
	df_dic['o_w'] = df_tf['o_w'].values()

#	df_orientation_list = []
	# df_list.append(df_TM['field.header.stamp'])
	# df_list.append(df_TM['field.pose.pose.position.x'])
	# df_list.append(df_TM['field.pose.pose.position.y'])
	# df_list.append(df_TM['field.pose.pose.position.z'])
	#
	# df_list.append(df_TM['field.pose.pose.orientation.x'])
	# df_list.append(df_TM['field.pose.pose.orientation.y'])
	# df_list.append(df_TM['field.pose.pose.orientation.z'])
	# df_list.append(df_TM['field.pose.pose.orientation.w'])

	return df_dic


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
	path = Path()
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()

	# path_pub = rospy.Publisher('/path', PoseStamped, queue_size=10)
	path_pub = rospy.Publisher('/path', Path, queue_size=10)
	rate = rospy.Rate(1)
	# pose_list = get_csv()
	pose_dict = get_csv()

	# P.header.stamp.secs = (current_time).to_sec()
	dts = 0
	loop = 0
	switch = False

	# while not rospy.is_shutdown():
	while not switch:
		print(loop, len(df_TM['field.header.stamp']))
		connections = path_pub.get_num_connections()
		if connections > 0:
			for i in range(len(pose_dict['index'])):
				print(loop)
				pose = PoseStamped()
				pose.header.frame_id = "/path"
				current_time = rospy.Time.now()
				pose.pose.position.x = pose_dict['x'][i]
				pose.pose.position.y = pose_dict['y'][i]
				pose.pose.position.z = pose_dict['z'][i]
				pose.pose.orientation.x = pose_dict['o_x'][i]
				pose.pose.orientation.y = pose_dict['o_y'][i]
				pose.pose.orientation.z = pose_dict['o_z'][i]
				pose.pose.orientation.w = pose_dict['o_w'][i]

				# for i in range(len(df_TM['field.header.stamp'])):
				# 	P = PoseStamped()
				# 	P.header.frame_id = "/path"
				# 	current_time = rospy.Time.now()
				# 	# P.header.stamp.secs = (current_time).to_sec()
				# 	P.pose.position.x = pose_list[1][i]
				# 	P.pose.position.y = pose_list[2][i]
				# 	P.pose.position.z = pose_list[3][i]
				# 	P.pose.orientation.x = pose_list[4][i]
				# 	P.pose.orientation.y = pose_list[5][i]
				# 	P.pose.orientation.z = pose_list[6][i]
				# 	P.pose.orientation.w = pose_list[7][i]

				pose.header.seq = path.header.seq
				path.header.frame_id = "/path"
				path.header.stamp = current_time
				dt = (current_time - last_time).to_sec()
				dts = dts + dt
				pose.header.stamp = path.header.stamp
				# path.poses.append(P)
				path.poses.append(pose)
				path_pub.publish(path)

				# P.header.stamp.nsecs = dts
				last_time = current_time
				# path_pub.publish(P)

				loop += 1

			switch = True
			print(path.poses)
			print(loop)


			print("1 time")
		else:
			rate.sleep()

	rospy.spin()



if __name__ == '__main__':
	callback()
	# get_csv()
