#!/usr/bin/env python

"""
ENPM661: Project 2

Akhilrajan Vethirajan (v.akhilrajan@gmail.com)
Vishaal Kanna Sivakumar (vishaal@terpmail.umd.edu)
M.Eng. Student, Robotics
University of Maryland, College Park

"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import cv2
import math


def map_gen():
	map = np.zeros((250,400))
	c=15
	for y in range(1,map.shape[0]+1):
		for x in range(1, map.shape[1]+1):
			if x>=165-c and x<=235+c and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(235+c/1.414)))*(x-(235+c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(140+c)-map.shape[0]+(120+c/1.414))/((200)-(165-c/1.414)))*(x-(165-c/1.414))+map.shape[0]-(120+c/1.414)<=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((165-c/1.414)-(200)))*(x-(200))+map.shape[0]-(60-c)>=y and ((map.shape[0]-(80-c/1.414)-map.shape[0]+(60-c))/((235+c/1.414)-200))*(x-200)+map.shape[0]-(60-c)>=y:
				map[y-1][x-1]=10000
			if ((map.shape[0]-(210+c/1.414)-map.shape[0]+(185))/((115+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185<=y and ((map.shape[0]-(100-c/1.414)-map.shape[0]+185)/((105+c/1.414)-(36-c)))*(x-(36-c))+map.shape[0]-185>=y and ((map.shape[0]-(210+c/1.414)-map.shape[0]+180)/((115+c/1.414)-(75+c))*(x-(75+c))+map.shape[0]-180>=y or ((map.shape[0]-180-map.shape[0]+(100-c/1.414))/((75+c)-(105+c/1.414)))*(x-(105+c/1.414))+map.shape[0]-(100-c/1.414)<=y):
				map[y-1][x-1]=10000
			if (x-300)**2+(y-map.shape[0]+185)**2<=(40+c)**2:
				map[y-1][x-1]=10000
			#if x>250 and x<275:
			#	map[y-1][x-1]=1
			if x>0 and x<=c:
				map[y - 1][x - 1] = 10000
			if x>400-c and x<=400:
				map[y - 1][x - 1] = 10000
			if y>0 and y<=c:
				map[y - 1][x - 1] = 10000
			if y>250-c and y<=250:
				map[y - 1][x - 1] = 10000
	return map


def Move_dir(dup_map,map, action, state, index,Parent_cost,i):
	x,y,theta = state[0], state[1], state[2]
	cost=0
	y_ = y+action[1]
	if y_%1>=0.5:
		y_=math.floor(y+action[1])+0.5
	else:
		y_ = math.floor(y + action[1])
	x_ = x+action[0]
	if x_%1>=0.5:
		x_=math.floor(x+action[0])+0.5
	else:
		x_ = math.floor(x + action[0])

	theta_ = ((theta+(i-2)*30)%360)%12

	y_ = int(2 * y_)
	x_ = int(2 * x_)

	if x + action[0]>=0 and x + action[0]<400 and y + action[1]>=0 and y + action[1]<250:
		if map[math.floor(y+action[1])][math.floor(x+action[0])] == 0 and map[math.ceil(y+action[1])][math.ceil(x+action[0])] == 0: # and dup_map[y_][x_][theta_]==0:
			#dup_map[y_][x_][theta_] = 1
			x = x + action[0]
			y = y + action[1]
			cost = Parent_cost+(action[0]**2 + action[1]**2)**0.5
			index=index+1
			theta = (theta+(i-2)*30)%360
			return dup_map,map, x, y, theta, cost, index, 1
	return dup_map,map, x, y, theta, cost, index, 0


def New_node(map, OpenList, ClosedList, Goal_node_x, Goal_node_y, Goal_angle, index, dup_map,a):
	M = heapq.heappop(OpenList)
	ClosedList.append(M)
	y_ = M[3][1]
	if y_ % 1 >= 0.5:
		y_ = math.floor(y_) + 0.5
	else:
		y_ = math.floor(y_)
	x_ = M[3][0]
	if x_ % 1 >= 0.5:
		x_ = math.floor(x_) + 0.5
	else:
		x_ = math.floor(x_)

	theta_ = int((M[3][2] % 360) / 30)
	theta1 = M[3][2]*np.pi/180

	y_ = int(2 * y_)
	x_ = int(2 * x_)

	if ((M[3][0]-Goal_node_x)**2+(M[3][1]-Goal_node_y)**2)**0.5 <=1.5 and M[3][2] == Goal_angle:
		return dup_map, map, OpenList, ClosedList, index, 1
	elif dup_map[y_][x_][theta_]==2:
		return dup_map, map, OpenList, ClosedList, index, 0

	dup_map[y_][x_][theta_] = 2
	# theta1 = theta_ * np.pi/180
	Action_space = ((a*math.cos(theta1+2*3.14/6), a*math.sin(theta1+2*3.14/6)), (a*math.cos(theta1+3.14/6), a*math.sin(theta1+3.14/6)), (a*math.cos(theta1+0*3.14/6), a*math.sin(theta1+0*3.14/6)), (a*math.cos(theta1-1*3.14/6), a*math.sin(theta1-1*3.14/6)), (a*math.cos(theta1-2*3.14/6), a*math.sin(theta1-2*3.14/6)))
	for i in range(0,5):
		dup_map, map, new_node_x, new_node_y, new_node_angle, C2C, index, t = Move_dir(dup_map, map, Action_space[i], M[3], index, M[4], i)

		count = 0
		if t==1:
			if count == 0:
				new_node = (((Goal_node_x-new_node_x)**2+(Goal_node_y-new_node_y)**2)**0.5+C2C, index, M[1], (new_node_x, new_node_y, new_node_angle), C2C)
				heapq.heappush(OpenList, new_node)

	return dup_map, map, OpenList, ClosedList, index, 0


def generate_path(lst, ClosedList, idx):
	for i in range(0, len(ClosedList)):
		if ClosedList[i][1] == idx:
			idx = i
			break
	if ClosedList[idx][2] ==-1:
		lst.append(ClosedList[idx][3])
		return lst
	else:
		lst.append(ClosedList[idx][3])
		generate_path(lst, ClosedList, ClosedList[idx][2])


def main():
	map = map_gen()
	flag=1

	while(flag):
		print('Enter the Starting Coordinates')
		start_node_x = int(input("Enter the X coordinate of start position :")) #250
		start_node_y = int(input("Enter the Y coordinate of start position :")) #150
		start_angle = int(input("Enter theta of start position :"))
		print('Enter the Goal Coordinates')
		goal_node_x = int(input("Enter the X coordinate of goal position :")) #30
		goal_node_y = int(input("Enter the Y coordinate of goal position :")) #30
		goal_angle = int(input("Enter theta of goal position :"))
		# print('Enter the action length')
		a = int(input("Enter the action length :"))
		if start_node_x < 0 or start_node_y < 0 or start_node_x > 399 or start_node_y > 249 or goal_node_x < 0 or goal_node_y < 0 or goal_node_x > 399 or goal_node_y > 249:
			print("Coordinates entered are outside the map")
		elif start_node_x==goal_node_x and start_node_y==goal_node_y:
			print('Starting node and Goal node are same. Please enter different sets of matrices for each.')
		elif map[start_node_y][start_node_x] == 10000 or map[goal_node_y][goal_node_x] == 10000:
			print("Coordinates are inside the Obstacle")
		elif goal_angle % 30 != 0:
			print("Goal Theta value is wrong")
		elif start_angle % 30 != 0:
			print("Start Theta value is wrong")
		else:
			flag = 0

	print('Starting Node: ', (start_node_x, start_node_y, start_angle))
	print('Goal Node: ', (goal_node_x, goal_node_y, goal_angle))

	# total cost, current index, parent index, (x, y, theta), cost to come
	start_node = (0, 0, -1, (start_node_x, start_node_y, start_angle), 0)
	OpenList = [start_node]
	ClosedList = []
	heapq.heapify(OpenList)
	goal_reached = 0
	dup_map = np.zeros((500,800,12))
	map_color_r = np.zeros((250, 400))
	map_color_b = np.zeros((250, 400))
	map_color_g = np.zeros((250, 400))
	map_color1 = np.zeros((250, 400,3))
	index = 0
	i=0
	while len(OpenList) and not goal_reached:
		dup_map, map, OpenList, ClosedList, index, goal_reached = New_node(map, OpenList, ClosedList, goal_node_x, goal_node_y,goal_angle,index, dup_map,a)
		if len(OpenList) ==0 and not goal_reached:
			print('Solution Not Found')
			quit()
		i+=1

	print('Map explored')

	lst = []
	lst.append(ClosedList[len(ClosedList) - 1][3])
	generate_path(lst, ClosedList, ClosedList[len(ClosedList) - 1][2])

	print('Map explored and optimal path found.')

	map = map_gen()
	exp = []
	for	i in range(0, len(ClosedList)-1):
		# map[ClosedList[i][3][1]][ClosedList[i][3][0]]=0.7
		if i ==0:
			map_color = map
			map_color_r[map_color == 0] = 1
			map_color_b[map_color == 0.7] = 1
			map_color_g[map_color == 10000] = 1
			map_color1[:, :, 0] = map_color_r * 255
			map_color1[:, :, 1] = map_color_b * 255
			map_color1[:, :, 2] = map_color_g * 255
			map_color1 = cv2.resize(map_color1, (800, 500))
		c = 0
	#
		for j in range(i, len(ClosedList)):
			if ClosedList[i][1] == ClosedList[j][2]:
				# x1 = 2 * int(ClosedList[i][3][0])
				# y1 = 2 * int(ClosedList[i][3][1])
				# x2 = 2 * int(ClosedList[j][3][0])
				# y2 = 2 * int(ClosedList[j][3][1])
				# exp.append([[x1, y1], [x2, y2]])
				cv2.line(map_color1, (2*int(ClosedList[i][3][0]), 2*int(ClosedList[i][3][1])), (2*int(ClosedList[j][3][0]), int(2*ClosedList[j][3][1])), [0, 200, 0], 1)
				cv2.imshow("A*", map_color1)
				cv2.waitKey(1)
				c += 1
			if c == 5:
				break

	path = [elem for elem in lst][::-1]


	for i in range(0, len(path)-1):
		cv2.line(map_color1, (2*int(path[i][0]), 2*int(path[i][1])), (2*int(path[i + 1][0]), 2*int(path[i+1][1])), [255, 255, 255], 2)
		cv2.imshow("A*", map_color1)
		cv2.waitKey(100)



if __name__ == '__main__':
	main()
