import csim
from gym import spaces
from gym.utils import seeding
from time import localtime, strftime
import numpy as np
import copy
import sys
import matplotlib.pyplot as plt

# SCENARIO = "Basic"
# SCENARIO = "Passing"
SCENARIO = "Dot"

class Env:
	def __init__(self):
		self.AGENT_NUM = 1
		self.OBSTACLE_NUM = 9
		self.Parser = csim.Parser(SCENARIO, self.AGENT_NUM, self.OBSTACLE_NUM)

		ob_low = 0.0 * np.ones(59)
		ob_high =1.0 * np.ones(59)

		ac_low = -1.2 * np.ones(3)
		ac_high = 1.2 * np.ones(3)

		for i in range(2):
			ob_low[i] = -1.0

		ob_low[3] = -1.0
		ob_high[3] = 1.0
		ob_low[4] = -0.5
		ob_high[4] = 2.0
		ob_low[5] = -1.5
		ob_high[5] = 1.5

		# for i in range(6,14):
		# 	ob_low[i] = 0.0
		# 	ob_high[i] = 4.0

		self.observation_space = spaces.Box(
			low = ob_low,
			high = ob_high
		)

		self.action_space = spaces.Box(
			low = ac_low,
			high = ac_high
		)

		self.seed()
		self.reward_range = (-float('inf'), float('inf'))
		self.metadata = {'render.modes' : []}
		self.spec = None

		self.cur_step = 0
		self.initPlot()

# ================plot=============================
	def initPlot(self):
		self.fig = plt.figure()
		self.ax1 = self.fig.add_subplot(111)
		self.ax1.plot([], label='reward', color='black')
		self.ax1.plot([], label='target', color='r')
		self.ax1.plot([], label='pref', color='g')
		self.ax1.plot([], label='smooth', color='b')
		self.ax1.plot([], label='col', color='yellow')
		self.ax1.legend()

		self.resetRewardCum()
		self.reward_plot = []
		self.target_plot = []
		self.bubble_plot = []
		self.smooth_plot = []
		self.col_plot = []

	def resetRewardCum(self):
		self.reward_cum = 0
		self.reward_target_cum = 0
		self.reward_bubble_cum = 0
		self.reward_smooth_cum = 0
		self.reward_col_cum = 0

	def setRewardPlot(self, reward, rewards):
		self.reward_target_cum += rewards[0]

		if rewards[1] < -10:
			pref  = -10
		else:
			pref = rewards[1]
		self.reward_bubble_cum += pref
		self.reward_smooth_cum += rewards[2]
		self.reward_col_cum += rewards[3]

		self.reward_cum += rewards[0] + pref + rewards[2] + rewards[3]

	def drawPlot(self):
		self.target_plot.append(self.reward_target_cum/self.cur_step)
		self.bubble_plot.append(self.reward_bubble_cum/self.cur_step)
		self.smooth_plot.append(self.reward_smooth_cum/self.cur_step)
		self.col_plot.append(self.reward_col_cum/self.cur_step)
		self.reward_plot.append(self.reward_cum/self.cur_step)
		plt.ion()
		self.ax1.plot(self.reward_plot, label='reward', color='black')
		self.ax1.plot(self.target_plot, label='target', color='r')
		self.ax1.plot(self.bubble_plot, label='pref', color='g')
		self.ax1.plot(self.smooth_plot, label='smooth', color='b')
		self.ax1.plot(self.col_plot, label='col', color='yellow')
		plt.draw()
		plt.pause(0.1)
		plt.show()

	def plotSave(self):
		plt.savefig("../data/ckpt/network/graph"+str(strftime("%Y%m%d_%I%M", localtime())))

# =================================================

	def step(self, a):
		memory = self.Parser.Step(self.actions, False)

		obs = memory['obs']
		obs = self.convert_to_numpy(obs)

		body_state = obs['agent'][0]['body_state']
		sensor_state = obs['agent'][0]['sensor_state']
		state = np.concatenate((body_state, sensor_state), axis=None)
		reward = memory['reward']
		isTerm = memory['isTerm']

		rewards = memory['reward_sep']
		self.cur_step += 1

		self.setRewardPlot(reward, rewards)
		if isTerm:
			self.drawPlot()
			self.resetRewardCum()
			self.cur_step = 0

		return state, reward, isTerm, {}

	def steps(self, a_):
		self.actions = copy.copy(a_)

	def step_render(self, a_):
		memory = self.Parser.Step(a_, True)

		obs = memory['obs']
		obs = self.convert_to_numpy(obs)

		state = obs
		reward = memory['reward']
		isTerm = memory['isTerm']

		return state, reward, isTerm, {}

	def states(self):
		obs = self.Parser.Observe()
		obs = self.convert_to_numpy(obs)

		states = []
		for i in range(len(obs['agent'])):
			body_state = obs['agent'][i]['body_state']
			sensor_state = obs['agent'][i]['sensor_state']
			state = np.concatenate((body_state, sensor_state), axis=None)
			states.append(state)

		return states

	def seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def reset(self):
		self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)
		obs = self.Parser.Observe()
		obs = self.convert_to_numpy(obs)

		body_state = obs['agent'][0]['body_state']
		sensor_state = obs['agent'][0]['sensor_state']
		state = np.concatenate((body_state, sensor_state), axis=None)

		return state

	def observe(self):
		obs = self.Parser.Observe()
		obs = self.convert_to_numpy(obs)

		return obs

	def convert_to_numpy(self, obs):
		for i in range(self.AGENT_NUM): # r, p, d, front, color
			r_data = obs['agent'][i]['render_data']
			obs['agent'][i]['r'] = np.array([r_data[0], r_data[1]])
			obs['agent'][i]['p'] = np.array([r_data[2], r_data[3]])
			obs['agent'][i]['d'] = np.array([r_data[4], r_data[5]])
			obs['agent'][i]['front'] = r_data[6]
			obs['agent'][i]['color'] = np.array([r_data[7], r_data[8], r_data[9]])
			obs['agent'][i]['body_state'] = np.array(obs['agent'][i]['body_state'])
			obs['agent'][i]['sensor_state'] = np.array(obs['agent'][i]['sensor_state'])
			obs['agent'][i]['offset'] = np.array(obs['agent'][i]['offset_data'])


		obstacle_num = len(obs['obstacle'])
		for i in range(obstacle_num):
			obs['obstacle'][i]['p'] = np.array(obs['obstacle'][i]['p'])
			obs['obstacle'][i]['r'] = np.array(obs['obstacle'][i]['r'])
			obs['obstacle'][i]['front'] = np.array(obs['obstacle'][i]['front'])

		return obs

