import sys
sys.path.append('../base')

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from crowdobject import CrowdObject
from constants import Constants as cst

import numpy as np
import mMath
import math
import copy
import random

def drawTriangle(p_x, p_y, p_z):
	glBegin(GL_TRIANGLES)
	glVertex3f( p_x,  p_y, p_z)
	glVertex3f( p_y, -p_x, p_z)
	glVertex3f(-p_y,  p_x, p_z)
	glEnd()

def drawQuad(p_x, p_y, p_z):
	glBegin(GL_QUADS)
	glVertex3f( p_x,  p_y, p_z)
	glVertex3f( p_y, -p_x, p_z)
	glVertex3f(-p_y,  p_x, p_z)
	glEnd()

class Agent(CrowdObject):
	def __init__(self, state, color='RED'):
		self.reset(state, color)

	def getState(self):
		state = {}
		state['p'] = self.p
		state['q'] = self.q
		state['d'] = self.d
		state['v'] = self.v
		state['r'] = self.r
		state['fov'] = self.agent_fov
		state['color'] = self.color
		state['front'] = self.front
		state['q_lim'] = self.q_lim
		state['d_map'] = self.d_map
		state['v_map'] = self.v_map
		state['interval'] = self.interval
		state['v_depth'] = self.vision_depth

		return copy.copy(state)

	def depthMap(self):
		return self.d_map

	def setP(self, p = None):
		if p is None:
			p = [-200, 0]
		self.p = np.array(p, dtype=float)

	def setQ(self, q = None):
		if q is None:
			q = [0, 0]
		self.q = np.array(q, dtype=float)

	def setD(self, d = None):
		if d is None:
			d = [0, 0]
		self.d = np.array(d, dtype=float)

	def setFront(self, f = None):
		if f is None:
			f = 0.0
		self.front = np.array(f, dtype=float)

	def setDmap(self, d_map = None):
		self.d_map = np.array(d_map, dtype=float)

	def setVmap(self, d_map = None):
		self.v_map = np.array(v_map, dtype=float)

	def setColor(self, color = None):
		self.color = np.array(color, dtype=float)

	def reset(self, state, shape='CIRCLE'):
		self.p = state['p']
		self.q = state['q']
		self.d = state['d']
		self.v = state['v']
		self.color = state['color']
		self.front = state['front']
		self.v_map = state['v_map']
		self.d_map = state['d_map']

		self.r = cst.AGENT_RADIUS
		self.interval = cst.AGENT_SENSOR_INTERVAL
		self.agent_fov = cst.AGENT_FOV
		self.vision_depth = cst.VISION_DEPTH
		self.q_lim = cst.AGENT_SENSOR_DIMENSION
		self.q_val = np.ndarray(shape=(self.q_lim))
		self.trajectory = []
		self.trajectory_q = []

	def setView(self, new_d_map):
		self.d_map = new_d_map

	def render(self, depth=False, trajectory_=False, idx=0):
		#render Agent
		glPushMatrix()
		self.render_agent(idx)
		if depth:
			self.render_depth_map()
		glPopMatrix()

		if trajectory_:
			glLineWidth(4.0)
			# glColor3f(self.color[0],self.color[1],self.color[2])
			# glColor3f(0.2, 1.0, 0.2)
			if idx%8==0:
				glColor3f(1.0, 0.0,0.0)
			elif idx%8==1:
				glColor3f(0.0, 1.0,0.0)
			elif idx%8==2:
				glColor3f(1.0, 1.0,0.0)
			elif idx%8==3:
				glColor3f(1.0, 0.0,1.0)
			elif idx%8==4:
				glColor3f(0.0, 1.0,1.0)
			elif idx%8==5:
				glColor3f(0.5, 0.5,0.0)
			elif idx%8==6:
				glColor3f(0.4, 0.6,0.0)
			elif idx%8==7:
				glColor3f(0.3, 0.7,0.0)

			glBegin(GL_LINES)
			l = len(self.trajectory)

			for i in range(l):
				glVertex3f(self.trajectory[i][0], self.trajectory[i][1], 0.5)
			glEnd()

		self.render_destination()

	def render_agent(self, idx):
		# render agent

		glColor3f(self.color[0],self.color[1],self.color[2])
		glTranslatef(self.p[0], self.p[1], 1)
		quad = gluNewQuadric()
		gluSphere(quad, self.r, 50, 50)
		# gluCylinder(quad, self.r, self.r, 40, 50, 50)

		#render Triangle of Agent
		glColor3f(0.3, 0.3, 1.0)
		glPushMatrix()
		# glTranslatef(8.0*self.q[0], 8.0*self.q[1], 0.0)

		len_ = np.linalg.norm(self.q)
		cur_x = self.q[0]*self.r/len_*0.8
		cur_y = self.q[1]*self.r/len_*0.8

		drawTriangle(cur_x, cur_y, 1.0)

		# glColor3f(1, 0, 0)
		# glBegin(GL_LINES)
		# glVertex3f(0, 0, 0)
		# glVertex3f(self.q[0]*300, self.q[1]*300, 0)
		# glEnd()

		# glColor3f(0, 1, 0)
		# glBegin(GL_LINES)
		# glVertex3f(0, 0, 0)
		# glVertex3f(self.q[1]*-300, self.q[0]*300, 0)
		# glEnd()

		glPopMatrix()

	def render_destination(self):
		glColor3f(1.0, 1.0, 0.0)
		glPushMatrix()
		glTranslatef(self.d[0], self.d[1], 3)
		quad = gluNewQuadric()
		gluSphere(quad, 0.3, 50, 50)
		glPopMatrix()

	def render_depth_map(self):
		glLineWidth(2)
		for i in range(self.q_lim):
			angle = self.front + math.radians(self.agent_fov/2 - self.interval*i)
			glBegin(GL_LINES)
			glVertex3f(self.r*math.cos(angle), self.r*math.sin(angle), 0)
			glVertex3f((self.d_map[i]+self.r)*math.cos(angle), (self.d_map[i]+self.r)* math.sin(angle), 0)
			glEnd()

class Obstacle(CrowdObject):
	def __init__(self, state):
		self.reset(state)

	def reset(self, state):
		self.p = state['p']

		self.r = cst.OBSTACLE_RADIUS

	def setP(self, p = None):
		if p is None:
			p = [0.0, 0.0]

		self.p = np.array(p, dtype=float)

	def getState(self):
		state = {}
		state['p'] = self.p
		state['r'] = self.r
		return copy.deepcopy(state)

	def action(self, action):
		return

	def render(self):
		#render Obstacle
		glPushMatrix()
		glColor3f(0.2, 0.2, 0.25)
		glTranslatef(self.p[0], self.p[1], 0)
		quad = gluNewQuadric()
		gluSphere(quad, self.r, 50, 50)
		glPopMatrix()

#Rectangle Form
class Wall():
	def __init__(self, u, p, l):
		self.st = u
		p /= np.linalg.norm(p)
		self.ed = u + np.array(p) * l

		self.vec = p
		self.l = l

		self.u = u
		self.p = p
		self.q = np.array([p[1]*-1, p[0]])

	def render(self):
		glPushMatrix()
		glColor3f(0.7, 0.7, 0.7)
		glBegin(GL_LINES)
		glVertex3f(self.st[0], self.st[1], 0)
		glVertex3f(self.ed[0], self.ed[1], 0)
		glEnd()
		glPopMatrix()


