import numpy as np
import math
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
	return df_dic

pose_dict = get_csv()

A ={}
B ={}
C = {}
col = ['index', 'x', 'y']
for key in col:
	A[key] = []
	B[key] = []
	C[key] = []


rear_x = [1,2,3,4,5,6,7,8,9,10,11]
rear_y = [1,2,3,4,5,6,7,8,9,10,11]
point_x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
point_y = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


for i in range(len(rear_x)):
	A['index'].append(i)
	A['x'].append(rear_x[i])
	A['y'].append(rear_y[i])

for j in range(len(point_x)):
	B['index'].append(j)
	B['x'].append(point_x[j])
	B['y'].append(point_y[j])
print(A)
print(B)
print(C)


for i in range(len(rear_x)):
	dx = [A['x'][3] - B['x'][i]]
	C['x'].append(dx)
	dy = [A['y'][3] - B['y'][i]]
	C['y'].append(dy)

print(A['x'])
print(B['x'])
print(B['x'][3])

print(C['x'])
print(C['y'])
d = np.hypot(C['x'], C['y'])  # hypot : method of triangle
print(d)
ind = np.argmin(d)  # minimum of d
print("ind" ,ind)

print("-------------------------------------------------------------------")


np.array(rear_x)
np.array(rear_y)
np.array(point_x)
np.array(point_y)

dx_ = []
dy_ = []
print(rear_x)
print(point_x)
print(rear_x[3])
for i in range(len(rear_x)):
	dx = [rear_x[3] - point_x[i]]
	dx_.append(dx)
	dy = [rear_y[3] - point_y[i]]
	dy_.append(dy)

print(dx_)
print(dy_)
d = np.hypot(dx_, dy_)  # hypot : method of triangle

d = np.hypot(C['x'], C['y'])  # hypot : method of triangle
print(d)
ind = np.argmin(d)  # minimum of d
print(ind)

ax = A['x']
print(ax)
print("-------------------------------------------------------------------")

