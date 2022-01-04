import sys
from api import *
from time import sleep
import numpy as np
import random as r
import math


#######    YOUR CODE FROM HERE #######################
import random
grid =[]
neigh=[[-1,-1],[-1,0],[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1]]
greenZone_list = []
green_list_path = []
rows, cols = (200, 200)

class Node:

    def __init__(self,value,point):
        self.value = value  #0 for blocked,1 for unblocked, 2 for redZone
        self.point = point
        self.parent = None
        self.move = None
        self.H = 0
        self.G = 0
        self.F = 0

    def move_cost(self):
        return self.value


def isValid(pt):
    return pt[0]>=0 and pt[1]>=0 and pt[0]<200 and pt[1]<200

def neighbours(point):  #returns valid neighbours
	global grid,neigh
	x,y = point.point
	links=[]
	for i in range(len(neigh)):
		newX=x+neigh[i][0]
		newY=y+neigh[i][1]
		if not isValid((newX,newY)):
			continue
		links.append((i+1,grid[newX][newY]))
	return links
	
def euclidean(point,point2):     #returns the euclidean distance between two points
    return math.sqrt(((point.point[0] - point2.point[0]) ** 2) + ((point.point[1]-point2.point[1]) ** 2))

def aStar(start, goal):    #returns the shortest path between start and target using the aStar algorithm
	#The open and closed sets
	openset = set()
	closedset = set()	
	#
	start.parent = None
	goal.parent = None
	#Current point is the starting point
	current = start
	#Add the starting point to the open set
	openset.add(current)
	#While the open set is not empty
	while openset:
		#Find the item in the open set with the lowest G + H score
		current = min(openset, key=lambda o:o.G + o.H)
		#If it is the item we want, retrace the path and return it
		if current == goal:
			path = []
			while current.parent:
				path.append(current)
				current = current.parent
			path.append(current)
			return path[::-1]
		#Remove the item from the open set
		openset.remove(current)
		#Add it to the closed set
		closedset.add(current)
		#Loop through the node's children/siblings
		for move, node in neighbours(current):
			#If it is already in the closed set, skip it
			if node in closedset:
				continue
			#if cell is blocked
			if node.value == 0:
				continue
			#Otherwise if it is already in the open set
			if node in openset:
				#Check if we beat the G score 
				#diagonal moves and red regions have a greater cost
				new_g = current.G + (node.move_cost())*(euclidean(current, node))
				if node.G > new_g:
					#If so, update the node to have a new parent
					node.G = new_g
					node.parent = current
					node.move = move
			else:
				#If it isn't in the open set, calculate the G and H score for the node
				node.G = current.G + (node.move_cost())*(euclidean(current, node))
				node.H = euclidean(node, goal)
				#Set the parent to our current item
				node.parent = current
				node.move = move
				#Add it to the set
				openset.add(node)
	#Throw an exception if there is no path
	raise ValueError('No Path Found')


########## Default Level 1 ##########
def level1(botId):
	global grid
	global rows
	global cols
	botsPose = get_botPose_list()
	obstaclePose = get_obstacles_list()
	greenZone = get_greenZone_list()
	for i in range(rows):
		grid.append([])
		for j in range(cols):
			grid[i].append(Node(1, (i, j)))
	#initialising the grid to have node value as 0 for blocked, 1 for unblocked and 2 for redZone
	for obst in obstaclePose:
		for i in range(obst[0][0],obst[2][0]+1):
			for j in range(obst[0][1],obst[2][1]+1):
				grid[i][j] = Node(0, (i, j))

	start = grid[botsPose[0][0]][botsPose[0][1]]
	goal = grid[greenZone[0][0][0]][greenZone[0][0][1]]
	
	path = aStar(start, goal)
	print("aStar returned path")

	for i in range(1,len(path)):
		successful_move, mission_complete = send_command(botId,path[i].move)
		if successful_move:
			print("YES")
		else:
			print("NO")
		if mission_complete:
			print("MISSION COMPLETE")
		pos=get_botPose_list()
		print(pos[0])


def level2(botId):
	global greenZone_list
	global grid
	global rows
	global cols
	global green_list_path
	obstaclePose = get_obstacles_list()
	botsPose = get_botPose_list()
	greenZone = get_greenZone_list()

	for i in range(rows):
		grid.append([])
		for j in range(cols):
			grid[i].append(Node(1, (i, j)))
	for obst in obstaclePose:
		for i in range(obst[0][0],obst[2][0]+1):
			for j in range(obst[0][1],obst[2][1]+1):
				grid[i][j] = Node(0, (i, j))
	
	for i in range(len(greenZone)):     #stores the centroids of the rectangular green regions
		x = int((greenZone[i][0][0] + greenZone[i][2][0]) / 2)
		y = int((greenZone[i][0][1] + greenZone[i][2][1]) / 2)
		green = grid[x][y]
		greenZone_list.append(green)
		print(green.point)

	start = grid[botsPose[0][0]][botsPose[0][1]]
	
	while(len(greenZone_list) != 0):   #stores the order in which the green regions need to be covered with a greedy approach
		min = 400
		min_idx = 0
		for i in range(len(greenZone_list)):
			d = euclidean(start, greenZone_list[i])
			if d < min:
				min = d
				min_idx = i
		green_list_path.append(greenZone_list[min_idx])
		start = greenZone_list[min_idx]
		greenZone_list.pop(min_idx)
		print(start.point)

	goal = green_list_path[0]
	start = grid[botsPose[0][0]][botsPose[0][1]]
 
	for i in range(len(green_list_path)):
		
		print(start.point)
		print(goal.point)
		path = aStar(start, goal)
		print("aStar returned path")
		print(len(path))

		for j in range(1, len(path)):
			successful_move, mission_complete = send_command(botId, path[j].move)
			if successful_move:
				print("YES")
			else:
				print("NO")
			if mission_complete:
				print("MISSION COMPLETE")
			pos = get_botPose_list()
			print(pos[0])

		start = green_list_path[i]
		if i != len(greenZone)-1:
			goal = green_list_path[i+1]


def level3(botId):
	global greenZone_list
	global grid
	global rows
	global cols
	global green_list_path
	all_greenZone_list = []
	obstaclePose = get_obstacles_list()
	botsPose = get_botPose_list()
	greenZone = get_greenZone_list()

	for i in range(rows):
		grid.append([])
		for j in range(cols):
			grid[i].append(Node(1, (i, j)))
	for obst in obstaclePose:
		for i in range(obst[0][0],obst[2][0]+1):
			for j in range(obst[0][1],obst[2][1]+1):
				grid[i][j] = Node(0, (i, j))
	
	for i in range(len(greenZone)):
		x = int((greenZone[i][0][0] + greenZone[i][2][0]) / 2)
		y = int((greenZone[i][0][1] + greenZone[i][2][1]) / 2)
		green = grid[x][y]
		all_greenZone_list.append(green)
		print(green.point)

	for i in range(len(all_greenZone_list)):    #loops over all the bots' initial position 
		                                        #checks which bot is the closest to each green region initially, not a dynamic path
		min = 400
		min_idx = 0
		for j in range(len(botsPose)):
			botLoc = grid[botsPose[j][0]][botsPose[j][1]]
			d = euclidean(botLoc, all_greenZone_list[i])
			if d < min:
				min = d
				min_idx = j
		if min_idx == botId:
			greenZone_list.append(all_greenZone_list[i])

	start = grid[botsPose[botId][0]][botsPose[botId][1]]
	
	while(len(greenZone_list) != 0):
		min = 400
		min_idx = 0
		for i in range(len(greenZone_list)):
			d = euclidean(start, greenZone_list[i])
			if d < min:
				min = d
				min_idx = i
		green_list_path.append(greenZone_list[min_idx])
		start = greenZone_list[min_idx]
		greenZone_list.pop(min_idx)
		print(start.point)

	goal = green_list_path[0]
	start = grid[botsPose[botId][0]][botsPose[botId][1]]
 
	for i in range(len(green_list_path)):
		
		print(start.point)
		print(goal.point)
		path = aStar(start, goal)
		print("aStar returned path")
		print(len(path))

		for j in range(1, len(path)):
			successful_move, mission_complete = send_command(botId, path[j].move)
			if successful_move:
				print("YES")
			else:
				print("NO")
			if mission_complete:
				print("MISSION COMPLETE")
			pos = get_botPose_list()
			print(pos[0])

		start = green_list_path[i]
		if i != len(green_list_path)-1:
			goal = green_list_path[i+1]


def level4(botId):
	level3(botId)

def level5(botId):
	global greenZone_list
	global grid
	global rows
	global cols
	global green_list_path
	all_greenZone_list = []
	obstaclePose = get_obstacles_list()
	botsPose = get_botPose_list()
	greenZone = get_greenZone_list()
	redZone = get_redZone_list()

	for i in range(rows):
		grid.append([])
		for j in range(cols):
			grid[i].append(Node(1, (i, j)))
	for obst in obstaclePose:
		for i in range(obst[0][0],obst[2][0]+1):
			for j in range(obst[0][1],obst[2][1]+1):
				grid[i][j] = Node(0, (i, j))
	#initialising the grid to have a value of 2 for red regions
	#aStar incorporates this to find the shortest path
	for red in redZone:
		for i in range(red[0][0], red[2][0]+1):
			for j in range(red[0][1], red[2][1]+1):
				grid[i][j] = Node(2, (i, j))

	for i in range(len(greenZone)):
		x = int((greenZone[i][0][0] + greenZone[i][2][0]) / 2)
		y = int((greenZone[i][0][1] + greenZone[i][2][1]) / 2)
		green = grid[x][y]
		all_greenZone_list.append(green)
		print(green.point)

	for i in range(len(all_greenZone_list)):
		min = 400
		min_idx = 0
		for j in range(len(botsPose)):
			botLoc = grid[botsPose[j][0]][botsPose[j][1]]
			d = euclidean(botLoc, all_greenZone_list[i])
			if d < min:
				min = d
				min_idx = j
		if min_idx == botId:
			greenZone_list.append(all_greenZone_list[i])

	start = grid[botsPose[botId][0]][botsPose[botId][1]]
	
	while(len(greenZone_list) != 0):
		min = 400
		min_idx = 0
		for i in range(len(greenZone_list)):
			d = euclidean(start, greenZone_list[i])
			if d < min:
				min = d
				min_idx = i
		green_list_path.append(greenZone_list[min_idx])
		start = greenZone_list[min_idx]
		greenZone_list.pop(min_idx)
		print(start.point)

	goal = green_list_path[0]
	start = grid[botsPose[botId][0]][botsPose[botId][1]]
 
	for i in range(len(green_list_path)):
		
		print(start.point)
		print(goal.point)
		path = aStar(start, goal)
		print("aStar returned path")
		print(len(path))

		for j in range(1, len(path)):
			successful_move, mission_complete = send_command(botId, path[j].move)
			if successful_move:
				print("YES")
			else:
				print("NO")
			if mission_complete:
				print("MISSION COMPLETE")
			pos = get_botPose_list()
			print(pos[0])

		start = green_list_path[i]
		if i != len(green_list_path)-1:
			goal = green_list_path[i+1]

def level6(botId):    
	level5(botId)

#######    DON'T EDIT ANYTHING BELOW  #######################

if  __name__=="__main__":
	botId = int(sys.argv[1])
	level = get_level()
	if level == 1:
		level1(botId)
	elif level == 2:
		level2(botId)
	elif level == 3:
		level3(botId)
	elif level == 4:
		level4(botId)
	elif level == 5:
		level5(botId)
	elif level == 6:
		level6(botId)
	else:
		print("Wrong level! Please restart and select correct level")




