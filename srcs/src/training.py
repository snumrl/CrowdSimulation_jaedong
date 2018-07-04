import sys
sys.path.append('../base')

from time import localtime, strftime
from constants import Constants as cst
from replaymemory import ReplayMemory

from ddpg import DDPG
from basic import Basic
from corridor import Corridor
from bottleneck import Bottleneck
from crossway import Crossway
from circle import Circle

import csim

import matplotlib.pyplot as plt
import numpy as np
import mMath
import time

FLAG_USE_RECENT_CKPT = False
FLAG_M_REPLAY_LOAD = False
FLAG_M_REPLAY_SAVE = True

class Experiment_Offline:
	def __init__(self):

		# self.SCENARIO = 'Basic'
		self.SCENARIO = 'Corridor'
		# self.SCENARIO = 'Bottleneck'
		# self.SCENARIO = 'Crossway'
		# self.SCENARIO = 'Circle'

		self.setEnvironment(self.SCENARIO)
		self.setNetwork()

		self.isTerm = False
		self.flag_eval=False
		self.EvaluateList = []
		self.st_time = time.time()

	def setEnvironment(self, SCENARIO):
		if SCENARIO == 'Basic':
			self.Parser = csim.Parser("Basic")
		elif SCENARIO == 'Corridor':
			self.Parser = csim.Parser("Corridor")
		elif SCENARIO == 'Bottleneck':
			self.Parser = csim.Parser("Bottleneck")
		elif SCENARIO == 'Crossway':
			self.Parser = csim.Parser("Crossway")
		elif SCENARIO == 'Circle':
			self.Parser = csim.Parser("Circle")

		obs = self.Observe()

		if SCENARIO == 'Basic':
			self.Scenario = Basic(obs)
		elif SCENARIO == 'Corridor':
			self.Scenario = Corridor(obs)
		elif SCENARIO == 'Bottleneck':
			self.Scenario = Bottleneck(obs)
		elif SCENARIO == 'Crossway':
			self.Scenario = Crossway(obs)
		elif SCENARIO == 'Circle':
			self.Scenario = Circle(obs)

	def setNetwork(self):
		network_dim = []
		network_dim.append(3)
		network_dim.append(60)
		network_dim.append(2)
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
			self.Parser.Reset(-1)
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

	def Train(self):
		self.st_time = time.time()
		self.train_iter = cst.TRAIN_ITERATION
		print "Training..."
		print "Iteration : ", self.train_iter
		print "========================================="

		self.Parser.Reset(-1)
		self.isTerm = False
		step_count=0
		for iteration in range(1, self.train_iter):
			if self.isTerm:
				print "Reset New Episode : ", step_count," Steps"
				self.Algorithm.expl_rate_decay()

				self.isTerm = False
				step_count=0
				if self.flag_eval:
					self.flag_eval=False
					self.episode_evaluation()
				self.Parser.Reset(-1)

			obs, action, memory = self.Execute(action_type='ACTOR', run_type='TRAIN')
			memory['obs'] = self.convert_to_numpy(memory['obs'])
			self.Update('ACTOR', obs, action, memory)

			if memory['isTerm']:
				self.isTerm = memory['isTerm']

			step_count += 1

			if self.train_iter<10 or iteration%(self.train_iter/10) == 0:
				print "\n", iteration/(float)(self.train_iter/10)*10, "% Done...\n"

			if iteration % 1000 == 0:
				self.flag_eval=True

		training_time = time.time() - self.st_time
		self.Algorithm.save(FLAG_M_REPLAY_SAVE, training_time, self.EvaluateList)
		self.learning_graph()
		print "========================================="
		print "Training Done!"
		print "Total Training Time : \t", training_time, "seconds"

		return

	def learning_graph(self):
		plt.plot(self.EvaluateList)
		plt.show()

	def episode_evaluation(self):
		evaluation_num = cst.EVALUTAION_SET

		step_count = 0
		avg_reward = 0
		total_reward = 0
		for i in range(evaluation_num):
			self.Parser.Reset(i)
			for j in range(200):
				obs, action, memory = self.Execute(action_type='ACTOR', run_type='TRAIN')

				total_reward += memory['reward']
				step_count += 1
				if memory['isTerm']:
					break

		avg_reward = total_reward / float(step_count)
		self.EvaluateList.append(avg_reward)
		print "======================================="
		print "Avg Reward : ", avg_reward
		print "======================================="

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

	if FLAG_USE_RECENT_CKPT:
		experiment.Algorithm.load_network(type='actor')
		experiment.Algorithm.load_network(type='critic')
		if FLAG_M_REPLAY_LOAD:
				experiment.Algorithm.load_memory()
				experiment.Algorithm.load_eval()

		experiment.EvaluateList = experiment.Algorithm.eval

	if not FLAG_M_REPLAY_LOAD:
		experiment.WarmUp()

	experiment.Train()
