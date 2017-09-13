########## Project 1 -- Dongkyu Kim, 2015.10.13 ##########
#
# Dongkyu Kim
# 113298196
# dk2823
#
# Function Descriptions
#
# My find_path function is based on A* algorithm that visits and extends 
# the states which has priority order,g value + heuristic. And marks every 
# distinct,(state,velocity), as visited once it is done comparing and store
# minimum cost path. At the end, if find the solution, then it will return 
# the path. Otherwise return false.
#
# My heuristic function is almost similar to h_edist heuristic function
# which is given. I was trying to change a lot, but it lead to non-optimal
# solution and gives slow performance. However, I have found a way to reduce
# performance time. The way I found is that if the car is heading to the other
# direction from the finish line, then it will have heuristic value as  
# distance from the state to finish line multiply by 2. 
# This might give little bit non-optimal path, but the performance is 
# much higher than h_edist function, especially in big map. except the map 
# which contains a lot of walls such as squiral1 or squiral2. 
# There is little enhancement on the maps, squiral1 and squiral2.
# As I said there will be small different on small map, but once the map is 
# enlarged, it will show you a big difference to simple h_edist heuristic
# function.
#
# velocities function and crash function are same as described on power point
#
# For styling, I limited 80 columns as max column and modified some 
# code as desired, including comments. 
# and added comments to help reading the code.
#
##########

import math
import heapq
		
def find_path(start, finalEdge, walls, heur):
	
	#This is for Priority Queue
	frontier = []
	heapq.heappush(frontier, (0, Node(start,False,(0,0),0)))

	explored = []                # This will check distinct (state,velocity)
	path = {} 	   				 # Path for result
	cost_so_far = {}			 # minimum cost for each path
	path[(start,(0,0))] = None   
	cost_so_far[(start,(0,0))] = 0
	find_it = False	
	debug = 0
	tracking_path = []           # To reverse from finish line to start point 
	
	#start looping to find path
	while not len(frontier) == 0:
		(order,curr) = heapq.heappop(frontier)
		explored.append((curr.state, curr.v))

		# This if statement is for final path 
		# This will check if it is done, and will store final path
		if (curr.parent and curr.v == (0,0) 
				and (intersect((curr.parent.state, curr.state), finalEdge))):
			# if find, then get ready for finding path.
			find_it = True
			tracker = curr
			tracking_path.insert(0, curr.state)
			
			# reverse the path to make ordered one
			while tracker:
				if not path[(tracker.state, tracker.v)] == None:
					(ts,tv) = path[(tracker.state, tracker.v)]
					tracking_path.insert(0,ts)
				tracker = tracker.parent
			break;
		
		# get its children
		next_n = expand(curr, get_next_nodes, explored,debug, walls)
		for next in next_n:
			# get new cost
			ncost = cost_so_far[(curr.state, curr.v)] + (next.g - next.parent.g)
			
			# if the state and velocity is new or has minimal length, then do update
			if ((next.state,next.v) not in cost_so_far 
					or ncost < cost_so_far[(next.state, next.v)]):
				# do update or add the state, velocity and its cost.
				cost_so_far[(next.state, next.v)] = ncost
				priority = ncost + heur(next, finalEdge, walls)
				heapq.heappush(frontier, (priority,next))
				explored.append((next.state, next.v))
				path[(next.state, next.v)] = (curr.state, curr.v)
	
	# after while loop, if it has solution, then return solution otherwise return false
	if find_it:
		return tracking_path
	else: 
		return False

		
def heur(s, e, w):
	# get state x, y and parent's x,y.
	(x, y) = s.state
	((x1, y1), (x2, y2)) = e
	(px, py) = s.parent.state
	
	# get distance from two points of the finish line
	d1 = math.sqrt((x1-x)**2 + (y1-y)**2)
	d2 = math.sqrt((x2-x)**2 + (y2-y)**2)

	# if it is same, then get the middle point of the finish line
	if d1 == d2:
		(xm,ym) = ((x1+x2)/2.0, (y1+y2)/2.0)
		curr_len = math.sqrt((xm-x)**2 + (ym-y)**2)
		parent_len = math.sqrt((xm-px)**2 + (ym-py)**2)

		# check if the current state heading to other direction. 
		# if it is, then return its value * 2
		if curr_len < parent_len:
			return curr_len 
		return curr_len * 2
	
	else:
		pd1 = math.sqrt((x1-px)**2 + (y1-py)**2)
		pd2 = math.sqrt((x2-px)**2 + (y2-py)**2)

		# check if the current state heading to other direction. 
		# if it is, then return its value * 2
		if min(d1,d2) < min(pd1,pd2):
			return min(d1, d2) 
		return min(d1, d2) * 2


# This function is a helper function for extend function from given code 		
def get_next_nodes(curr, walls):
	s = (curr.state, curr.v)
	next_nodes = []
	(px, py) = curr.state
	
	# get corresponding velocity set and add it to curr node
	for (vx, vy) in velocities(s, walls):
		nx = px + vx
		ny = py + vy
		dist = math.hypot(px - nx, py - ny)
		next_nodes.append(((nx,ny), dist, (vx,vy)))
	
	return next_nodes

# crash function described as power point	
def crash(e,w):   
	
	# check if any edge is intersect to other
	for each_w in w:
		if intersect(e, each_w): 
			return True
	return False    

# velocities function that return set of the velocity	
def velocities(s, w):
	diff = [-1,0,1]
	
	(p, v) = s
	(px, py) = p
	(sx, sy) = v
	next_speed = set()

	for x in diff:
		for y in diff:
			if not crash((p,(px + (sx+x), py + (sy+y))), w): 
				next_speed.add((sx+x, sy+y))

	return next_speed

# Node from given code, I modified little bit			
class Node():
	def __init__(self, state, parent, v, g):
		self.state = state
		self.parent = parent
		self.g = g
		self.v = v
		self.children = []

# expand function from given code, I modified little bit
def expand(x, get_children, explored, debug, walls):
	pruned = []
	for (state, cost, v) in get_children(x, walls):
		if (state, v) in explored:
			if debug >= 3:
				pruned.append(state)
		else:
			y = Node(state, x, v, x.g+cost)
			x.children.append(y)
	if debug >= 3:
		print '       explored:', ', '.join(explored)
		print '         pruned:', ', '.join(pruned)
	return x.children

# intersect function which is given from professor	
# after this comment, I didn't limit columns since this is given from professor
# I limited 80 columns for styling for above code.
#
# =====================the codes were given from professor=================
#
def intersect(e1,e2):
	"""Return True if edges e1 and e2 intersect, False otherwise."""	   
	
	# First, grab all the coordinates
	((x1a,y1a), (x1b,y1b)) = e1
	((x2a,y2a), (x2b,y2b)) = e2
	dx1 = x1a-x1b
	dy1 = y1a-y1b
	dx2 = x2a-x2b
	dy2 = y2a-y2b

	if (dx1 == 0) and (dx2 == 0):		# both lines vertical
		if x1a != x2a: return False
		else: return collinear_point_in_edge((x1a,y1a),e2) or collinear_point_in_edge((x1b,y1b),e2)
	if (dx2 == 0):		# e2 is vertical, so m2 = infty
		x = x2a
		# compute y = m1 * x + b1, but minimize roundoff error
		y = (x2a-x1a)*dy1/float(dx1) + y1a
		return collinear_point_in_edge((x,y),e1) and collinear_point_in_edge((x,y),e2) 
	elif (dx1 == 0):		# e1 is vertical, so m1 = infty
		x = x1a
		# compute y = m2 * x + b2, but minimize roundoff error
		y = (x1a-x2a)*dy2/float(dx2) + y2a
		return collinear_point_in_edge((x,y),e1) and collinear_point_in_edge((x,y),e2) 
	else:		# neither line is vertical
		# check m1 = m2, without roundoff error:
		if dy1*dx2 == dx1*dy2:		# same slope, so either parallel or collinear
			# check b1 != b2, without roundoff error:
			if dx2*dx1*(y2a-y1a) != dy2*dx1*x2a - dy1*dx2*x1a:
				return False
			return collinear_point_in_edge((x1a,y1a),e2) or collinear_point_in_edge((x1b,y1b),e2)
		# compute x = (b2-b1)/(m1-m2) but minimize roundoff error:
		x = (dx2*dx1*(y2a-y1a) - dy2*dx1*x2a + dy1*dx2*x1a)/float(dx2*dy1 - dy2*dx1)
		# compute y = m1*x + b1 but minimize roundoff error
		y = (dy2*dy1*(x2a-x1a) - dx2*dy1*y2a + dx1*dy2*y1a)/float(dy2*dx1 - dx2*dy1)
	return collinear_point_in_edge((x,y),e1) and collinear_point_in_edge((x,y),e2) 


def collinear_point_in_edge((x,y),((xa,ya),(xb,yb))):
	"""
	Helper function for intersect.
	If (x,y) is collinear with the edge from (xa,ya) to (xb,yb), then
	(x,y) is in the edge if
		x is between xa and xb, inclusive, and
		y is between ya and yb, inclusive.
	The test of y is redundant except when line is vertical.
	"""
	if ((xa <= x <= xb) or (xb <= x <= xa)) \
	   and ((ya <= y <= yb) or (yb <= y <= ya)):
	   return True
	return False

# heuristic functions that were given	
	
def h_edist(node, ((x1,y1),(x2,y2)), walls):
	"""Euclidean distance from state to the finish line."""
	(x,y) = node.state
	d1 = math.sqrt((x1-x)**2 + (y1-y)**2)
	d2 = math.sqrt((x2-x)**2 + (y2-y)**2)
	
	## If the finish line's endpoints are equidistant,
	## then return the distance to the center of the finish line.
	## Otherwise, return the distance to the closer endpoint.
	if d1 == d2:
		(xm,ym) = ((x1+x2)/2.0, (y1+y2)/2.0)
		return math.sqrt((xm-x)**2 + (ym-y)**2)
	else:
		return min(d1,d2)

def h_xymax(node, f_line, walls):
	"""Return the max of the x and y distances from state to center of finish line."""
	(x,y) = node.state
	((x1,y1),(x2,y2)) = f_line
	d1 = xymax_helper((x,y),(x1,y1))
	d2 = xymax_helper((x,y),(x2,y2))
	if d1 == d2:
		(xm,ym) = ((x1+x2)/2.0, (y1+y2)/2.0)
		return xymax_helper((x,y),(xm,ym))
	return min(d1,d2)

def xymax_helper((x1,y1), (x2,y2)):
	"""
	Helper function for xymax; returns the max of the x-distance and y-distance
	between two points.
	"""
	xdist = abs(x1 - x2)	  # distance in the x direction
	ydist = abs(y1 - y2)	  # distance in the y direction
	return max(xdist, ydist)  # max distance in either direction



def h_moves0(node, f_line, walls):
	"""Number of moves it would take to reach center of finish line if stopped at current loc."""
	# get the max of the x distance and y distance to the goal
	dist = h_xymax(node.state, f_line, walls)
	
	# Suppose we start at speed 0, increase our speed by 1 at each move until
	# we reach some speed s, then decrease our speed by 1 at each move until
	# we're back to 0. If we don't crash into a wall, we've traveled a distance
	# of s^2. We want the largest s such that s^2 <= the distance to the goal.
	s = 0
	while s**2 <= dist:
		s += 1
	# We now have the smallest s such that s^2 >= the distance to the goal, so
	# we need to reduce s by 1.
	s -= 1
	
	# If we start at speed 0, go up to speed s, then go back down to 0, that's a
	# sequence of 2s moves. By inserting at most two more moves (of speed s or less)
	# into the sequence, we can get a sequence of moves that reaches the goal.
	difference = dist - s**2
	if difference == 0:
		return 2*s		 # no more moves, we're already at the goal
	if difference <= s:
		return 2*s+1	 # need to insert 1 more move somewhere
	else:
		return 2*s + 2	 # we need to insert 2 moves
		
		
