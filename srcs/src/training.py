import sys
sys.path.append('../base')
sys.path.append('scenarios')

from time import localtime, strftime
from constants import Constants as cst
from replaymemory import ReplayMemory

from ddpg import DDPG
from basic import Basic
from corridor import Corridor
from bottleneck import Bottleneck
from crossway import Crossway
from circle import Circle
from mix import Mix

import csim

import os
import matplotlib.pyplot as plt
import numpy as np
import mMath
import time

FLAG_REPLAY_LOAD = True
FLAG_CKPT_LOAD = True
FLAG_EVAL_LOAD = True
FLAG_EVAL_SAVE = True
FLAG_WARMUP = False

# SCENARIO = 'Basic'
# SCENARIO = 'Corridor'
# SCENARIO = 'Bottleneck'
# SCENARIO = 'Crossway'
# SCENARIO = 'Circle'
SCENARIO = 'Mix'

AGENT_NUM = 60
OBSTACLE_NUM = 0

actor_dir = "../data/ckpt/actor/tmp/"
critic_dir = "../data/ckpt/critic/tmp/"

class Experiment_Offline:
	def __init__(self):
		self.SCENARIO = SCENARIO
		self.AGENT_NUM = AGENT_NUM
		self.OBSTACLE_NUM = OBSTACLE_NUM

		self.setEnvironment(SCENARIO)
		self.setNetwork()

		self.isTerm = False
		self.eval_flag = False
		self.eval_iter = 0
		self.train_iter = 0
		self.episode_iter = 0
		self.eval_col = []
		self.eval_end = []

		self.actor_dir = actor_dir
		self.critic_dir = critic_dir

		if FLAG_EVAL_SAVE:
			os.makedirs(actor_dir)
			os.makedirs(critic_dir)

		self.st_time = time.time()

	def setEnvironment(self, SCENARIO):
		if SCENARIO == 'Basic':
			self.Parser = csim.Parser("Basic", self.AGENT_NUM, self.OBSTACLE_NUM)
		elif SCENARIO == 'Corridor':
			self.Parser = csim.Parser("Corridor", self.AGENT_NUM, self.OBSTACLE_NUM)
		elif SCENARIO == 'Bottleneck':
			self.Parser = csim.Parser("Bottleneck", self.AGENT_NUM, self.OBSTACLE_NUM)
		elif SCENARIO == 'Crossway':
			self.Parser = csim.Parser("Crossway", self.AGENT_NUM, self.OBSTACLE_NUM)
		elif SCENARIO == 'Circle':
			self.Parser = csim.Parser("Circle", self.AGENT_NUM, self.OBSTACLE_NUM)
		elif SCENARIO == 'Mix':
			self.Parser = csim.Parser("Mix", self.AGENT_NUM, self.OBSTACLE_NUM)

		obs = self.Observe()

		if SCENARIO == 'Basic':
			self.Scenario = Basic(obs)
		elif SCENARIO == 'Corridor':
			self.Scenario = Corridor(obs)
		elif SCENARIO == 'Crossway':
			self.Scenario = Crossway(obs)
		elif SCENARIO == 'Circle':
			self.Scenario = Circle(obs)
		elif SCENARIO == 'Bottleneck':
			self.Scenario = Bottleneck(obs)
		elif SCENARIO == 'Mix':
			self.Scenario = Mix(obs)

	def setNetwork(self):
		network_dim = []
		network_dim.append(cst.AGENT_BODY_DIMENSION)
		network_dim.append(cst.AGENT_SENSOR_DIMENSION*3)
		network_dim.append(cst.AGENT_ACTION_DIMENSION)
		self.Algorithm = DDPG(network_dim)

	def Observe(self):
		obs = self.Parser.Observe()
		obs = self.convert_to_numpy(obs)

		return obs

	def Execute(self, action_type, run_type):
		obs = self.Observe()
		action = self.Algorithm.Action(obs, action_type, run_type)
		action = self.convert_to_action_double(action)
		memory = self.Parser.Step(action, run_type == 'TEST')

		return obs, action, memory

	def Update(self, action_type, obs, action, memory):
		self.Algorithm.addMemory(action_type=='GREEDY', obs, action, memory['obs'], memory['reward'], memory['isTerm']);
		if action_type == 'ACTOR':
			self.Algorithm.Update()

	def WarmUp(self):
		print "\n\n"
		print "start time is ... ", strftime("%Y/%m/%d_%I:%M", localtime())
		print "Warming up....."
		print "========================================="

		self.warmup_iter = cst.WARMUP_ITERATION
		for i in range(self.warmup_iter):
			if self.warmup_iter<10 or i%(self.warmup_iter/10)==0:
				print "Warmup Generation...\t", (i / (float)(self.warmup_iter))*100,"%"

			self.isTerm = False
			self.Parser.Reset(-1, self.AGENT_NUM , self.OBSTACLE_NUM)
			while not self.isTerm:
				obs, action, memory = self.Execute(action_type='GREEDY', run_type='TRAIN')
				memory['obs'] = self.convert_to_numpy(memory['obs'])

				if memory['isTerm']:
					self.isTerm = True

				self.Update('GREEDY', obs, action, memory)

		print "========================================="
		print "Warmup Done!"
		print "Replay Memory Size : \t", self.Algorithm.rm.getMemorySize()
		print "Total Warmup Time : \t", time.time()-self.st_time,  "seconds"
		return

	def eval_save(self):
		self.Algorithm.criticNN.save(self.critic_dir, str(self.eval_iter))
		self.Algorithm.actorNN.save(self.actor_dir, str(self.eval_iter))

	def Train(self):
		self.st_time = time.time()
		self.train_iter = cst.TRAIN_ITERATION
		print "Training..."
		print "Iteration : ", self.train_iter
		print "========================================="

		self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)
		self.isTerm = False
		step_count=0
		for iteration in range(1, self.train_iter):
			if self.isTerm:
				self.isTerm = False
				print "Reset New Episode : ", step_count-1," Steps"
				step_count=0
				self.Algorithm.expl_rate_decay()
				self.episode_iter += 1
				if self.eval_flag:
					self.eval_flag = False
					self.evaluation()
					self.eval_iter += 1
					if FLAG_EVAL_SAVE:
						self.eval_save()

				self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)

			obs, action, memory = self.Execute(action_type='ACTOR', run_type='TRAIN')
			memory['obs'] = self.convert_to_numpy(memory['obs'])

			self.Update('ACTOR', obs, action, memory)

			if memory['isTerm']:
				self.isTerm = memory['isTerm']

			step_count += 1

			if self.train_iter<10 or iteration%(self.train_iter/10) == 0:
				print "\n", iteration/(float)(self.train_iter/10)*10, "% Done...\n"

			if iteration % 1000 == 0:
				self.eval_flag=True

		training_time = time.time() - self.st_time
		self.Algorithm.save(self.eval_end, self.eval_col)
		if FLAG_EVAL_SAVE:
			self.save_plot()
			self.change_dir()

		print "========================================="
		print "Training Done!"
		print "Total Training Time : \t", training_time, "seconds"
		self.plot_graph()

	def save_plot(self):
		plt.plot(self.eval_end)
		plt.plot(self.eval_col)
		plt.savefig("../data/ckpt/actor/tmp/graph.png")
		plt.savefig("../data/ckpt/critic/tmp/graph.png")

	def change_dir(self):
		os.rename(self.actor_dir, "../data/ckpt/actor/"+str(self.Algorithm.cur_time)+"/")
		os.rename(self.critic_dir, "../data/ckpt/critic/"+str(self.Algorithm.cur_time)+"/")

	def plot_graph(self):
		plt.plot(self.eval_end)
		plt.plot(self.eval_col)
		plt.show()

	def eval_save(self):
		self.Algorithm.criticNN.save(self.critic_dir, str(self.eval_iter))
		self.Algorithm.actorNN.save(self.actor_dir, str(self.eval_iter))

	def evaluation(self):
		eval_num = 10
		col_flag = False
		step_end, step_col = 0, 0
		avg_end, avg_col = 0, 0
		total_end, total_col = 0, 0
		for i in range(eval_num):
			self.Parser.Reset(i, 0, 0)
			col_flag = False
			for j in range(300):
				obs, action, memory = self.Execute(action_type='ACTOR', run_type='TEST')

				if not col_flag:
					total_col += memory['reward']
					step_col += 1

				total_end += memory['reward']
				step_end += 1

				if memory['isCol']:
					col_flag = True

				if memory['isTerm']:
					break

		avg_col = total_col / float(step_col)
		avg_end = total_end / float(step_end)

		self.eval_end.append(avg_end)
		self.eval_col.append(avg_col)

		print "====================================="
		print "Avg Reward End : ", avg_end
		print "Avg Reward Col : ", avg_col
		print "====================================="

	def convert_to_numpy(self, obs):
		agent_num = len(obs['agent'])
		for i in range(agent_num):
			obs['agent'][i]['v'] = obs['agent'][i]['v'][0]
			obs['agent'][i]['p'] = np.array(obs['agent'][i]['p'])
			obs['agent'][i]['q'] = np.array(obs['agent'][i]['q'])
			obs['agent'][i]['d'] = np.array(obs['agent'][i]['d'])
			obs['agent'][i]['front'] = obs['agent'][i]['front'][0]
			obs['agent'][i]['v_map'] = np.array(obs['agent'][i]['v_map'])
			obs['agent'][i]['d_map'] = np.array(obs['agent'][i]['d_map'])

		obstacle_num = len(obs['obstacle'])
		for i in range(obstacle_num):
			obs['obstacle'][i]['p'] = np.array(obs['obstacle'][i]['p'])

		return obs

	def convert_to_action_double(self, action):
		action_len = len(action)
		for i in range(action_len):
			action[i]['theta'] = float(action[i]['theta'])
			action[i]['velocity'] = float(action[i]['velocity'])

		return action

if __name__=="__main__":
	experiment = Experiment_Offline()

	if FLAG_CKPT_LOAD:
		experiment.Algorithm.load_network(type='actor')
		experiment.Algorithm.load_network(type='critic')
		if FLAG_REPLAY_LOAD:
			experiment.Algorithm.load_memory()

		if FLAG_EVAL_LOAD:
			experiment.Algorithm.load_eval()
			experiment.eval_end = experiment.Algorithm.eval_end
			experiment.eval_col = experiment.Algorithm.eval_col


	if FLAG_WARMUP:
		experiment.WarmUp()

	experiment.Train()
