#!/usr/bin/env python

import math
from Queue import PriorityQueue


def aStar(spawnPoint, goal, map_cells):
	visitedNot = PriorityQueue()
	visitedNot.put((0, spawnPoint))
	parent				= {}
	cost				= {}
	path 				= []
	parent[spawnPoint] 	= None
	cost[spawnPoint]	= 0

	while not visitedNot.empty():
		pos = visitedNot.get()[1]

		if pos == goal:
			while pos != spawnPoint:
				path.insert(0, (pos[0], pos[1]))
				pos = parent[pos]

			return path
			
		for n, d in getNeighbours(pos, parent[pos], map_cells):
			movement_cost = 1
			if d == True:
				movement_cost = math.sqrt(2) # diagonal is approx 1.34
			
			costAdapted = cost[pos] + movement_cost
			if n not in cost or costAdapted < cost[n]:
				cost[n] 	= costAdapted
				pri			= costAdapted + getCostHeuristic(n, goal, spawnPoint)
				parent[n]	= pos 
				visitedNot.put((pri, n))
				
	raw_input("No viable path")
	return False	
	
# Salesman
def prioritize(spawnPoint, goalQ, goalsSorted = []):
	toGo = PriorityQueue()
	
	if goalQ.qsize() > 1:
		for n in range(goalQ.qsize()):
			pos = goalQ.get()[1]
			pri	= getCostHeuristic(pos, spawnPoint, spawnPoint)	
			toGo.put((pri, pos))
		
		nearestP = toGo.get()
		goalsSorted.append(nearestP[1])
		return prioritize(nearestP[1], toGo, goalsSorted)
		
	else:
		goalsSorted.append(goalQ.get()[1])

		return goalsSorted

def getNeighbours(pos, parent, map_cells):
	n = []
	# change for cell size
	posX	 	= round(pos[0],2)
	posY	 	= round(pos[1],2)
	acc 	 	= .02
	p 			= parent
	m			= map_cells
	
	# False options are for non-diognal movement and vice-versa
	options = [	[-acc, 	  0, False], 
				[ acc, 	  0, False], 
				[ 	0, -acc, False], 
				[	0,	acc, False],
			   	[ acc, 	acc,  True], 
			   	[ acc, -acc,  True], 
			   	[-acc, -acc,  True], 
			   	[-acc,  acc,  True]
			  ]
			   
	for o in options:
		n = getNeighbour(posX+o[0], posY+o[1], p, m, n, o[2])
	
	return n

# helps reduce clutter
def getNeighbour(posX, posY, parent, map_cells, nbrs, d = False, safeD = .061):
	# safeD 0.6 for hardest p
	if isSafe((posX, posY), map_cells) and (posX, posY) != parent:
		# change round for cell size
		x = (round(posX,2))
		y = (round(posY,2))

		# if the location has a "safe" circle around it, append it as a location
		# make a circle because safer
		options = [	[-safeD, 	  0], 
					[ safeD, 	  0], 
					[ 	  0, -safeD], 
					[	  0,  safeD],
			   		[ safeD,  safeD], 
			   		[ safeD, -safeD], 
			   		[-safeD, -safeD],
			   		[-safeD,  safeD] ]
		
		allBools = []
		for o in options:
			allBools.append(isSafe((x + o[0], y + o[1]), map_cells))
			
		allBools.append(isSafe((x+.2, y), map_cells))
		allBools.append(isSafe((x-.2, y), map_cells))
		
		if False not in allBools:
			nbrs.append([(x, y), d])
			
	return nbrs

def getCostHeuristic(pos, g, sp):
	# manhattan did not provide "beautiful" paths
	posX	= round(pos[0], 1) 	
	posY 	= round(pos[1], 1)
	gX		= g[0]
	gY		= g[1]
	spX		= sp[0]
	spY		= sp[1]
	
	# manhattan
	h = abs(posX - gX) + abs(posY - gY)
	
	#sets a straight line from start to goal and tries to follow it
	dx1	= posX 	- gX
	dy1	= posY 	- gY
	dx2	= spX 	- gX
	dy2	= spY	- gY
	direct = abs(dx1*dy2 - dx2*dy1)
	#return (h +direct*0.001)
	return h
	

def isSafe(pos, map_cells):
	grid = map_cells.info
	
	#get index of specific x:y coordinate on the map (faster than subdiving)
	col = int(round(pos[0] / grid.resolution + .5*grid.width))
	row = int(round(pos[1] / grid.resolution + .5*grid.height))

	i = col+row*grid.width
	# i is the actual index of the original map.data
	
	try:
		# if the pixel is not empty
		if ( map_cells.data[i] != 0 ):
			return False
			
	#just in case
	except IndexError:
		return False
		
	#return true if pixel is "free"
	return True	

