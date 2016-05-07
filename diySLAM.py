import random
import numpy as np
import mmap

class landmark:
	instances = 0
	tolerance = 20;
	def __init__():
		self.seen_count = 0
		self.line = None
		self.misses = 0
		++instances

	def update_landmark_hit(n_slope,n_intercept):
		self.slope = n_slope
		self.intercept = n_intercept
		++self.seen_count
	def update_landmark_miss():
		++misses
		if misses > tolerance
		--instances
		del(self)

class robot:

	def __init__():
		self.bearing = 0
		self.x = 0
		self.y = 0
	def return_state():
		return [self.x,self.y,self.bearing]

class obstacle:

	instances = 0

	def __init__():
		self.point_set = []
		self.connected_lines = []
		++instances


def polar_to_cartesian(r,theta):
	x = r*cosine(theta)
	y = r*sin(theta)
	return [x,y]


def mean_square_regression(sample):
	x_points = [x[0] for x in sample]
	y_points = [x[1] for x in sample]
	x_bar = map(sum,x_points)/len(x_points)
	y_bar = map(sum,y_points)/len(y_points)
	m = map(sum,[(x-x_bar for x in x_points)*(y-y_bar for y in y_points)])
	b = y_bar-m*x_bar
	return [m,b] 



def ransac(point_vector, n, dg, dist, c):
	count = 0
	lines = {}
	cartesian_points = [polar_to_cartesian(point_vector[x],x) for x in range(0,len(point_vector))]
	while(len(cartesian_points) > c):
		if count > n: break
		point = randrange(0,len(cartesian_points))
		sample = [cartesian_points[x] for x in range(point-dg,point+dg)]
		best_fit = mean_square_regression(sample)
		on_line = [p for p in sample if dist(p,b,m) < dist]
		cartesian_points = [x for x in cartesian_points if x not in on_line]
		best_fit = mean_square_regression(on_line)
		lines[count] = best_fit
		++count
	return lines


def line_dist(line_1,line_2):
	y_1 = line_1[1]
	y_2 = line_2[1]
	return sqrt((y_1-y_2)**2)

def point_dist(p_1,p_2):
	return Math.sqrt((p_1[0]-p_2[0])**2+(p_1[1]-p_2[1])**2)

def association(possible_new_landmark,land_mark_list,threshold):
	current_best = None 
	last_dist = 0
	for x in land_mark_list:
		distance = line_dist(x.line,possible_new_landmark)
		if distance < threshold and distance <= last_dist:
			last_dist = distance
			current_best = x

	if current_best is not None:
		current_best.line = possible_new_landmark
		++current_best.seen_count


def largest_distance_points(points_set):

	points_set.sort(cmp=lambda p_1,p_2: p_1[0]>p_2[0])
	
	if len(points_set) is 1:
		return 0

	l_1 = [points_set[x] for x in range(0,len(points_set)/2)]
	l_2 = [points_set[x] for x in range(len(points_set)/2,len(points_set))]
	subset_left = largest_distance_points(l_1)
	subset_right = largest_distance_points(l_2) 




def extended_kalman_filter:
	pass;

def read_file():
	scan_file = open("./lazer_scan","r+")
	m_mapped = mmap.mmap(scan_file.fileno(),0)
	scan_data = []
	for line in m_mapped:
		scan_data.append(line.split())

	m_mapped.flush()
	m_mapped.close()
	scan_file.close()
	return scan_data












		







