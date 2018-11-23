import math
import copy
import numpy as np

def AngleToCoor(angle):
	coor = np.array([np.cos((angle)*np.pi / 180.0), np.sin((angle)*np.pi / 180.0)])
	return coor

def RadianToCoor(rad):
	coor = np.array([np.cos(rad), np.sin(rad)])
	return coor

def RadianToDegree(rad):
	return rad/3.141592*180.0

def DegreeToRadian(degree):
	return degree/180.0*3.141592

def CoorToAngle(coor):
	angle = np.arctan2(coor[1], coor[0])*180.0 / np.pi
	return angle

def CoorToRadian(coor):
	rad = np.arctan2(coor[1], coor[0])
	return rad

def Rotate2d(cos, sin, point):
	return np.array([cos*point[0]+sin*point[1], -1*sin*point[0]+cos*point[1]])

def RotateCoor(coor, angle):
	return np.array([np.cos((angle)*np.pi / 180.0) *coor[0] - np.sin((angle)*np.pi / 180.0) * coor[1],
		np.sin((angle)*np.pi / 180.0) * coor[0] + np.cos((angle)*np.pi / 180.0) * coor[1]])

def RayToSphereDistance(p1, p2, angle, r):
	p12 = p2 - p1

	d = p12 - np.inner(p12, angle) * angle
	d_len = np.sqrt(np.inner(d, d))


	if r < d_len or np.inner(p12, angle) < 0:
		return False, None

	dist = np.absolute(np.inner(p12, angle)) - np.sqrt(r*r - d_len*d_len)
	if dist<0:
		dist=0
	return True, dist

# def RayToEllipseDistance(p1, p2, angle, rl, rs):

def CrossProduct(v1, v2):
	return v1[0]*v2[1] - v1[1]*v2[0]

def InnerProduct(v1, v2):
	return v1[0]*v2[0] + v1[1]*v2[1]

def Line(p1, p2):
	A = (p1[1] - p2[1])
	B = (p2[0] - p1[0])
	C = (p1[0]*p2[1] - p2[0]*p1[1])
	return A, B, -C

def LineIntersection(L1, L2):
	D  = L1[0] * L2[1] - L1[1] * L2[0]
	Dx = L1[2] * L2[1] - L1[1] * L2[2]
	Dy = L1[0] * L2[2] - L1[2] * L2[0]
	if D != 0:
		x = Dx / D
		y = Dy / D
		return x,y
	else:
		return False

def MinWallOffset(wall, agent_state):
	p = agent_state['p']
	r = agent_state['r']

	if wall.st[0] == wall.ed[0]:
		if wall.st[0] > p[0]:
			return np.array([-r, 0])
		else:
			return np.array([r, 0])

	if wall.st[1] == wall.ed[1]:
		if wall.st[1] > p[1]:
			return np.array([0, -r])
		else:
			return np.array([0, r])

	return np.array([0, 0])

# u :  angle
# p : agent p
# v : line vec
# q : line start
def LineLineDistance(u, p, v, q):
	#Cramer's Rule
	a = u[0]
	b = v[0]
	c = u[1]
	d = v[1]
	e = q[0]-p[0]
	f = q[1]-p[1]

	if a*d - b*c == 0 :
		return None

	x = [(d*e - b * f) / (a*d - b*c), (c * e - a * f) / (a*d - b*c)]

	return x

#u : Line Start Position
#p : Line Vector
#l : Line Length
#q : Sphere pos
#r : Sphere radius
def LineSphereIntersection(u, p, l, q, r):
	x = np.linalg.norm(q-u)
	y = np.inner(p, (q-u))

	if y +r < 0:
		return False

	if y -r > l:
		return False

	if x*x - y*y < r*r:
		return True

	return False

def PointWallDistance(agent_p, wall):
	edge_num = wall.edge_num
	dist_1_min =1000
	dist_2_min = 1000
	dist_min = 1000
	for i in range(edge_num):
		dist1 = LineLineDistance([0.0, 1.0], agent_p, wall.edges[i][0], wall.edges[i][1] , 500)
		dist2 = LineLineDistance([0.0, -1.0], agent_p, wall.edges[i][0], wall.edges[i][1] , 500)
		dist3 = LineLineDistance([0.0, 1.0], agent_p, wall.edges[i][0], wall.edges[i][1] , 500)
		dist4 = LineLineDistance([0.0, -1.0], agent_p, wall.edges[i][0], wall.edges[i][1] , 500)

		if dist1< dist_1_min:
			dist_1_min = dist1
		if dist2 < dist_2_min:
			dist_2_min = dist2


	if dist_1_min < dist_2_min:
		return dist_1_min
	else:
		return dist_2_min

# angle[0] : cos theta
# angle[1] : sin theta
def LineWallDistance(angle, agent_p, wall, depth):
	edge_num = wall.edge_num
	dist_min = copy.copy(depth)
	for i in range(edge_num):
		dist = LineLineDistance(angle, agent_p, wall.edges[i][0], wall.edges[i][1] , depth)
		if dist < dist_min:
			dist_min = dist

	return dist_min
