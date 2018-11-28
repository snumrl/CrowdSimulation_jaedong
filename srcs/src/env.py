import csim
from gym import spaces
from gym.utils import seeding
from time import localtime, strftime
import numpy as np
import copy
import sys
import matplotlib.pyplot as plt
from norm import Normalizer

# SCENARIO = "Basic"
# SCENARIO = "Passing"
SCENARIO = "Dot"
# SCENARIO = "Hallway"

class Env:
	def __init__(self):
		self.AGENT_NUM = 1
		self.OBSTACLE_NUM = 9
		self.Parser = csim.Parser(SCENARIO, self.AGENT_NUM, self.OBSTACLE_NUM)

		ob_low = 0.0 * np.ones(86) # 14 + 36 + 36 = 86
		ob_high =1.0 * np.ones(86)

		# ac_low = -1.2 * np.ones(3)
		# ac_high = 1.2 * np.ones(3)

		self.normalizer_ac = Normalizer(
			real_val_max=1.0*np.ones(3),
			real_val_min=-1.0*np.ones(3),
			norm_val_max=4*np.ones(3),
			norm_val_min=-4*np.ones(3)
		)

		for i in range(2):
			ob_low[i] = -1.0

		ob_low[3] = -1.0
		ob_high[3] = 1.0
		ob_low[4] = -0.5
		ob_high[4] = 2.0
		ob_low[4] = -1.5
		ob_high[4] = 1.5

		for i in range(50,86):
			ob_low[i] = -2.5
			ob_high[i] = 2.5

		self.observation_space = spaces.Box(
			low = ob_low,
			high = ob_high
		)

		self.action_space = spaces.Box(
			low = self.normalizer_ac.norm_val_min,
			high = self.normalizer_ac.norm_val_max
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
		self.prefV_plot = []
		self.smooth_plot = []
		self.col_plot = []

	def resetRewardCum(self):
		self.reward_cum = 0
		self.reward_target_cum = 0
		self.reward_prefV_cum = 0
		self.reward_smooth_cum = 0
		self.reward_col_cum = 0

	def setRewardPlot(self, reward, rewards):
		self.reward_target_cum += rewards[0]
		self.reward_prefV_cum += rewards[1]
		self.reward_smooth_cum += rewards[2]
		self.reward_col_cum += rewards[3]

		self.reward_cum += rewards[0]*rewards[1]*rewards[2]*rewards[3]

	def drawPlot(self):
		self.target_plot.append(self.reward_target_cum/self.cur_step)
		self.prefV_plot.append(self.reward_prefV_cum/self.cur_step)
		self.smooth_plot.append(self.reward_smooth_cum/self.cur_step)
		self.col_plot.append(self.reward_col_cum/self.cur_step)
		self.reward_plot.append(self.reward_cum/self.cur_step)
		print("reward : ",self.reward_cum/self.cur_step)
		plt.ion()
		self.ax1.plot(self.reward_plot, label='reward', color='black')
		self.ax1.plot(self.target_plot, label='target', color='r')
		self.ax1.plot(self.prefV_plot, label='pref', color='g')
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
		velocity_state = obs['agent'][0]['velocity_state']
		state = np.concatenate((body_state, sensor_state, velocity_state), axis=None)
		reward = memory['reward']
		isTerm = memory['isTerm']

		if np.isnan(state).any():
			print("state nan")
			sys.exit(1)

		print_ = False
		for ac in self.actions:
			for ac_ in ac:
				if ac_ > 1.5 or ac_ < -1.5:
					print_= True

		if state[4] > 1.2 or state[4] < -0.5:
			print_= True

		# if print_:
		# 	print("action : ",self.actions)
		# 	print("state : ", state)
		# 	print("reward : ", reward)
		# 	print("\n")

		rewards = memory['reward_sep']
		self.cur_step += 1

		self.setRewardPlot(reward, rewards)
		if isTerm:
			self.drawPlot()
			self.resetRewardCum()
			self.cur_step = 0

		return state, reward, isTerm, {}

	def steps(self, a_):
		a_norm = []
		for ac in a_:
			a_norm.append(self.normalizer_ac.norm_to_real(ac))

		self.actions =np.array(a_norm, dtype=np.float32)

	def step_render(self, a_):
		a_norm = []
		for ac in a_:
			a_norm.append(self.normalizer_ac.norm_to_real(ac))

		memory = self.Parser.Step(np.array(a_norm, dtype=np.float32), True)

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
			velocity_state = obs['agent'][i]['velocity_state']
			state = np.concatenate((body_state, sensor_state, velocity_state), axis=None)
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
		velocity_state = obs['agent'][0]['velocity_state']
		state = np.concatenate((body_state, sensor_state, velocity_state), axis=None)

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
			obs['agent'][i]['velocity_state'] = np.array(obs['agent'][i]['velocity_state'])
			obs['agent'][i]['offset'] = np.array(obs['agent'][i]['offset_data'])

		obstacle_num = len(obs['obstacle'])
		for i in range(obstacle_num):
			obs['obstacle'][i]['p'] = np.array(obs['obstacle'][i]['p'])
			obs['obstacle'][i]['r'] = np.array(obs['obstacle'][i]['r'])
			obs['obstacle'][i]['front'] = np.array(obs['obstacle'][i]['front'])

		wall_num = len(obs['wall'])
		for i in range(wall_num):
			obs['wall'][i]['p'] = np.array(obs['wall'][i]['p'])
			obs['wall'][i]['w'] = obs['wall'][i]['w']
			obs['wall'][i]['h'] = obs['wall'][i]['h']

		return obs

