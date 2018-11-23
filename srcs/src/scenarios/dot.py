import sys
sys.path.append('../base')

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from agent import Agent, Obstacle, Wall
from constants import Constants as cst

import numpy as np
import queue
import copy
import math

class Dot:
	def __init__(self, obs):
		self.name = 'Dot'
		self.init_agents(obs['agent'])
		self.init_obstacles(obs['obstacle'])
		self.init_walls()
		self.init_record()

	def init_agents(self, agent_obs):
		self.agents = []
		self.agent_count = len(agent_obs)
		for i in range(self.agent_count):
			state={}
			state['r'] = np.array(agent_obs[i]['r'])
			state['p'] = np.array(agent_obs[i]['p'])
			state['d'] = np.array(agent_obs[i]['d'])
			state['front'] = agent_obs[i]['front']
			state['color'] = np.array(agent_obs[i]['color'])
			state['vision'] = np.array(agent_obs[i]['sensor_state'])
			state['offset'] = np.array(agent_obs[i]['offset_data'])
			self.agents.append(Agent(state))

	def init_obstacles(self, obstacle_obs):
		self.obstacles = []
		self.obstacle_count = len(obstacle_obs)
		for i in range(self.obstacle_count):
			state={}
			state['p'] = np.array(obstacle_obs[i]['p'])
			state['r'] = np.array(obstacle_obs[i]['r'])
			state['front'] = np.array(obstacle_obs[i]['front'])
			self.obstacles.append(Obstacle(state))

	def init_walls(self):
		self.walls = []

	def init_record(self):
		self.record_buffer_p = queue.Queue(maxsize=5000)
		self.record_buffer_q = queue.Queue(maxsize=5000)
		self.record_obs_p = []
		self.record_agent_p = []
		self.record_agent_q = []
		self.record_flag = False
		self.record_size = 0

	def setObjectData(self, obs):
		agent_obs = obs['agent']
		for i in range(self.agent_count):
			r = np.array(agent_obs[i]['r'])
			p = np.array(agent_obs[i]['p'])
			d = np.array(agent_obs[i]['d'])
			front = agent_obs[i]['front']
			color = np.array(agent_obs[i]['color'])
			vision = np.array(agent_obs[i]['sensor_state'])
			offset = np.array(agent_obs[i]['offset_data'])
			self.agents[i].setR(r)
			self.agents[i].setP(p)
			self.agents[i].setD(d)
			self.agents[i].setFront(front)
			self.agents[i].setColor(color)
			self.agents[i].setVision(vision)
			self.agents[i].setOffset(offset)

		obs_data = obs['obstacle']
		for i in range(self.obstacle_count):
			self.obstacles[i].setP(obs_data[i]['p'])
			self.obstacles[i].setR(obs_data[i]['r'])
			self.obstacles[i].setFront(obs_data[i]['front'])

	def render(self, vision=False, trajectory=False):
		for i in range(self.obstacle_count):
			self.obstacles[i].render()

		self.agents[0].render(vision, trajectory, 0)
		for i in range(1, self.agent_count):
			self.agents[i].render(False, trajectory, i)

		for i in range(len(self.walls)):
			self.walls[i].render()

	def record(self, flag=False):
		if flag:
			if not self.record_flag:
				self.record_flag = True
				self.record_buffer_p.queue.clear()
				self.record_buffer_q.queue.clear()
				del self.record_obs_p[:]
				del self.record_agent_p[:]
				del self.record_agent_q[:]
				for i in range(self.obstacle_count):
					self.record_obs_p.append(self.obstacles[i].p)

			data_p, data_q = [], []
			for i in range(self.agent_count):
				data_p.append(copy.copy(self.agents[i].p))
				data_q.append(copy.copy(self.agents[i].q))

			self.record_buffer_p.put(data_p)
			self.record_buffer_q.put(data_q)
		else:
			if self.record_flag:
				self.record_flag = False
				self.record_size = 0
				for i in range(5000):
					if self.record_buffer_p.empty():
						self.record_size = i
						break;

					self.record_agent_p.append(self.record_buffer_p.get())
					self.record_agent_q.append(self.record_buffer_q.get())

	def render_record(self, frame):
		frame = frame % self.record_size
		for i in range(self.agent_count):
			self.agents[i].setP(self.record_agent_p[frame][i])
			self.agents[i].setQ(self.record_agent_q[frame][i])

		for i in range(self.obstacle_count):
			self.obstacles[i].setP(self.record_obs_p[i])

		self.render()
