import sys
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
sys.path.append('../base')

import numpy as np
import random
import mMath
import math
import copy
import pickle
import tensorflow as tf

from nn import CriticNetwork, ActorNetwork
from time import localtime, strftime
from constants import Constants as cst
from replaymemory import ReplayMemory

def random_variables(n, min, max, isInteger=False):
	arr = np.random.random(n) * (double)(max-min) + (double)(min)
	return arr

class DDPG:
	def __init__(self, dim):
		self.critic_path = cst.CN_CKPT_PATH
		self.actor_path = cst.AN_CKPT_PATH
		self.replaymemory_path = cst.RM_PATH

		self.dim_body = dim[0]
		self.dim_sensor = dim[1]
		self.dim_state = dim[0] + dim[1]*3
		self.dim_action = dim[2]

		self.sess = tf.InteractiveSession()
		self.act_lr = cst.ACT_LEARNING_RATE
		self.cri_lr = cst.CRI_LEARNING_RATE
		self.tau = cst.TAU
		self.batch_size = cst.BATCH_SIZE
		self.gamma = cst.REWARD_DECAY

		self.actorNN = ActorNetwork(self.sess,self.dim_state, self.dim_action, self.act_lr, self.tau, self.batch_size)
		self.criticNN = CriticNetwork(self.sess,self.dim_state, self.dim_action, self.cri_lr, self.tau, self.gamma, self.actorNN.get_num_trainable_vars())

		self.sess.run(tf.global_variables_initializer())

		self.actorNN.update_target_network()
		self.criticNN.update_target_network()

		self.rm = ReplayMemory('DDPG')

		self.agent_count = cst.AGENT_COUNT
		self.exploration_rate = cst.EXPLORATION_RATE
		self.epsilon = cst.CRITIC_EPSILON
		self.LOSS_ITERATION = cst.LOSS_ITERATION

		self.expl_noise = OUNoise(self.dim_action)

		self.expl=False
		self.expl_decay = cst.EXPLORATION_DECAY

	#=====================action===========================

	def Action(self, obs,action_type, run_type):
		if action_type == 'GREEDY':
			return self.action_greedy(obs)

		self.isExploration(run_type=='TRAIN')

		action_list = []
		agent_num = len(obs['agent'])
		for i in range(0, agent_num):
			agent_obs = obs['agent'][i]
			if np.linalg.norm(agent_obs['d']-agent_obs['p']) < cst.AGENT_RADIUS + 10:
				action = {}
				action['theta'] = 0
				action['velocity'] = 0
				action['stop'] = True
			else:
				action = self.get_action(agent_obs, run_type=='TEST')
				if self.expl:
					action = self.action_random(action)

			action_list.append(action)

		return action_list

	def action(self, obs, action_type, run_type):
		if action_type == 'GREEDY':
			return self.action_greedy(obs)

		self.isExploration(run_type=='TRAIN')

		action_list = []
		for i in range(0, self.agent_count):
			agent_obs = obs['agent'][i]
			if np.linalg.norm(agent_obs['d']-agent_obs['p']) < agent_obs['r'] + 10:
				action = {}
				action['theta'] = 0
				action['velocity'] = 0
				action['stop'] = True
			else:
				action = self.get_action(agent_obs, run_type=='TEST')
				if self.expl:
					print "action : ", action
					action = self.action_random(action)
					print "action noise : ", action

			action_list.append(action)

		# for i in range(self.agent_count):
		#   agent_obs = obs['agent'][i]
		#   if np.linalg.norm(agent_obs['d']-agent_obs['p']) < agent_obs['r'] + 5:
		#       action = {}
		#       action['theta'] = 0
		#       action['velocity'] = 0
		#       action['stop'] = True
		#   else:
		#       if i == 0:
		#           action = self.get_action(agent_obs, run_type=='TEST')
		#           if self.expl:
		#               action = self.action_random()
		#       else:
		#           action = self.get_action_greedy(agent_obs)

		#   action_list.append(action)

		return action_list

	def get_action(self, agent_obs, action_target=False):
		state_ = {}
		state_ = self.preprocess(agent_obs)
		state_body = np.reshape(state_['body'], (1, self.dim_body))
		state_sensor = np.reshape(state_['sensor'], (1, self.dim_sensor))

		if action_target:
			prediction = self.actorNN.predict_target(state_body, state_sensor)
		else:
			prediction = self.actorNN.predict(state_body, state_sensor)

		action = {}
		action['theta'] = prediction[0][0]
		action['velocity'] = prediction[0][1]
		action['stop'] = False

		return action

	def action_greedy(self, obs):
		action_list = []
		agent_num = len(obs['agent'])
		for i in range(agent_num):
			agent_obs = obs['agent'][i]

			action = self.get_action_greedy(agent_obs)
			action_list.append(action)

		return action_list

	def get_action_greedy(self, agent_obs):
		if np.linalg.norm(agent_obs['d']-agent_obs['p']) < 10 + 10:
			action = {}
			action['theta'] = 0
			action['velocity'] = 0
			action['stop'] = True
			return action

		greedy_dis = None

		angle_num = 20
		next_angle = (190/2.0)

		offset = 2
		direction = np.array(agent_obs['d']) - np.array(agent_obs['p'])
		direction /= np.linalg.norm(direction)

		greedy_dir = 0
		if random.random() < 0.5:
			greedy_dir = 1

		for angle in range(angle_num):
			if agent_obs['d_map'][angle] < 10+offset:
				continue

			curr_angle = 190/2 - angle*10
			curr_q = mMath.AngleToCoor(curr_angle + agent_obs['front']) * agent_obs['d_map'][angle]
			curr_dis = direction[0]*curr_q[0] +  direction[1]*curr_q[1]
			if greedy_dir == 0:
				if (greedy_dis is None) or (greedy_dis < curr_dis):
					next_angle = curr_angle
					greedy_dis = curr_dis
					next_q = curr_q
			else:
				if (greedy_dis is None) or (greedy_dis <= curr_dis):
					next_angle = curr_angle
					greedy_dis = curr_dis
					next_q = curr_q

		action={}
		action['theta'] = np.clip(next_angle, -10, 10) / 10.0

		if greedy_dis is None:
			action['velocity'] = -1
		else:
			action['velocity'] = 1

		action['stop'] = False

		return action

	def action_random(self, action=None):
		if action is None:
			action=dict()
			action['theta'] = np.random.normal()
			action['velocity'] = np.random.normal()
		else:
			noise_theta, noise_vel = self.expl_noise.noise()
			action['theta'] = action['theta']+noise_theta
			action['velocity'] = action['velocity']+noise_vel

		action['stop'] = False

		return action

	#=====================update==========================

	def Update(self):
		if len(self.rm.memory['critic'])>0 and len(self.rm.memory['actor'])>0:
			self.update_network()

	def update_network(self):
		rm_critic_batch = self.rm.getRandomMemories('critic')

		s_body_batch, s_sensor_batch, a_batch, r_batch, t_batch, s2_body_batch, s2_sensor_batch = [], [], [], [], [], [], []
		for m in rm_critic_batch:
			state_ = copy.copy(self.preprocess(m['state']['agent'][0]))
			state_body = copy.copy(state_['body'])
			state_sensor = copy.copy(state_['sensor'])
			action = copy.copy(np.array([m['action'][0]['theta'], m['action'][0]['velocity']]))
			next_state_ = copy.copy(self.preprocess(m['next_state']['agent'][0]))
			next_state_body = copy.copy(next_state_['body'])
			next_state_sensor = copy.copy(next_state_['sensor'])

			s_body_batch.append(state_body[0])
			s_sensor_batch.append(state_sensor[0])
			a_batch.append(action)
			r_batch.append(m['reward'])
			t_batch.append(m['term'])
			s2_body_batch.append(next_state_body[0])
			s2_sensor_batch.append(next_state_sensor[0])

		target_q = self.criticNN.predict_target(s2_body_batch, s2_sensor_batch, self.actorNN.predict_target(s2_body_batch, s2_sensor_batch))

		y_i = []
		c_batch_size = len(rm_critic_batch)
		for k in range(c_batch_size):
			if t_batch[k]:
				y_i.append(r_batch[k])
			else:
				y_i.append(r_batch[k] + self.gamma * target_q[k])

		# Update the critic given the targets
		predicted_q_value, _ = self.criticNN.train(
				s_body_batch, s_sensor_batch, a_batch, np.reshape(y_i, (int(c_batch_size), 1)))

		# Update the actor policy using the sampled gradient
		rm_actor_batch = self.rm.getRandomMemories('actor')

		actor_body_batch, actor_sensor_batch, actor_a_batch = [], [], []
		for m in rm_actor_batch:
			state_ = copy.copy(self.preprocess(m['state']['agent'][0]))
			state_body = copy.copy(state_['body'])
			state_sensor = copy.copy(state_['sensor'])
			action = copy.copy(np.array([m['action'][0]['theta'], m['action'][0]['velocity']]))
			actor_body_batch.append(state_body[0])
			actor_sensor_batch.append(state_sensor[0])
			actor_a_batch.append(action)

		act_batch = self.actorNN.predict(actor_body_batch, actor_sensor_batch)
		grads = self.criticNN.action_gradients(actor_body_batch, actor_sensor_batch, act_batch)
		self.actorNN.train(actor_body_batch, actor_sensor_batch, grads[0])

		# Update target networks
		self.actorNN.update_target_network()
		self.criticNN.update_target_network()

	#===================evaluate===========================

	def evaluate(self, obs, agent_idx, action, run_type='TRAIN'):
		state_ = {}
		agent_obs = obs['agent'][agent_idx]

		state_['body'] = np.array(self.preprocess_body(agent_obs['p'], agent_obs['q'], agent_obs['v'], agent_obs['d']))
		state_['action'] = np.array([action['theta'], action['velocity']])
		state_['sensor'] = np.array(self.preprocess_sensor(agent_obs['d_map'], agent_obs['v_map'], agent_obs['q_lim'], agent_obs['v_depth']))

		state_body = np.reshape(state_['body'], (1, self.dim_body))
		state_sensor = np.reshape(state_['sensor'], (1, self.dim_sensor))
		action = np.reshape(state_['action'], (1, self.dim_action))

		if run_type == 'TEST':
			prediction = self.criticNN.predict_target(state_body, state_sensor, action)[0]
		else:
			prediction = self.criticNN.predict(state_body, state_sensor, action)[0]

		return prediction

	def expl_rate_decay(self):
		if self.exploration_rate > 0.2:
			self.exploration_rate *= self.expl_decay
			print "exploration rate : ", self.exploration_rate

	#=====================replay_memory===========================

	def addMemory(self, is_greedy, obs, act, next_state, reward, is_term):
		if is_greedy:
			self.rm.addMemory('actor', obs, act, next_state, reward, is_term)
			self.rm.addMemory('critic', obs, act, next_state, reward, is_term)
		else:
			if self.expl:
				self.rm.addMemory('actor', obs, act, next_state, reward, is_term)
				self.expl = False
			else:
				self.rm.addMemory('critic', obs, act, next_state, reward, is_term)

	#==================save & load==========================

	def save(self, m_replay=False, training_time = 0, eval_list = None):
		cur_time = strftime("%Y%m%d_%I%M.ckpt", localtime())

		print "Save Critic Network : ",
		self.criticNN.save(self.critic_path, cur_time)

		print "Save Actor Network : ",
		self.actorNN.save(self.actor_path, cur_time)

		print "Parameters Saved...!"
		self.save_parameters(cur_time, training_time)

		print "Networks Saved...!"

		if m_replay:
			print "Replay Memories Saved...!"
			self.save_replaymemory(cur_time)

		if eval_list != None:
			print "Evaluation List Saved...!"
			self.save_evaluation(cur_time, eval_list)

	def save_replaymemory(self, cur_time):
		f = open(cst.RM_PATH+"checkpoint", 'w')
		f.write(cur_time)
		f.close()

		f = open(cst.RM_PATH+"rm_"+cur_time, 'w')
		pickle.dump(self.rm, f, protocol=pickle.HIGHEST_PROTOCOL)
		f.close()

	def save_evaluation(self, cur_time, eval_list=None):
		f = open(cst.EVAL_PATH+"checkpoint", 'w')
		f.write(cur_time)
		f.close()

		f = open(cst.EVAL_PATH+"eval_"+cur_time, 'w')
		pickle.dump(eval_list, f, protocol=pickle.HIGHEST_PROTOCOL)
		f.close()

	def save_parameters(self, cur_time, training_time):
		f_read = open(cst.PM_READ_PATH, 'r')
		f_write = open(cst.PM_WRITE_PATH+"pm_"+cur_time+".txt", 'w')
		f_write.write("traning time : "+str(training_time))
		while True:
			line = f_read.readline()
			if not line:
				break
			f_write.write(line)
		f_read.close()
		f_write.close()

	def load_network(self, type):
		if type=='actor':
			print "Load Recent Actor Network : ",
			self.actorNN.load(self.actor_path)
		elif type=='critic':
			print "Load Recent Critic Network : ",
			self.criticNN.load(self.critic_path)

	def load_memory(self):
		f = open(cst.RM_PATH+"checkpoint", 'r')
		recent_file_name = f.readline()
		f.close()

		f_rm = open(cst.RM_PATH+"rm_"+recent_file_name, 'r')
		self.rm = pickle.load(f_rm)
		f_rm.close()

		print "Load Replay Memory :  ", cst.RM_PATH,"rm_",recent_file_name

	def load_eval(self):
		f = open(cst.EVAL_PATH+"checkpoint", 'r')
		recent_file_name = f.readline()
		f.close()

		f_eval = open(cst.EVAL_PATH+"eval_"+recent_file_name, 'r')
		self.eval = pickle.load(f_eval)
		f_eval.close()

		print "Load Eval List :  ", cst.EVAL_PATH,"eval_",recent_file_name

	#=================other===============================

	def preprocess(self, agent_obs):
		state = {}
		state['body'] = np.array(self.preprocess_body(agent_obs['p'], agent_obs['q'], agent_obs['v'], agent_obs['d'])).reshape((1, self.dim_body))
		state['sensor'] = np.array(self.preprocess_sensor(agent_obs['d_map'], agent_obs['v_map'], 20, cst.VISION_DEPTH)).reshape((1, 60))

		return state

	def preprocess_body(self, p, q, v, d):
		p_ = np.array(p)
		q_ = np.array(q)
		d_ = np.array(d)

		width = cst.WINDOW_WIDTH/2.0
		height = cst.WINDOW_HEIGHT/2.0

		p_[0] = p_[0] / width
		p_[1] = p_[1] / height

		d_[0] = d_[0] / width
		d_[1] = d_[1] / height

		q_norm = np.linalg.norm(q_)
		q_ = (q_ / q_norm)

		pd = np.array(d_-p_)
		pd_len = np.linalg.norm(pd)
		pd_vec = pd / pd_len

		inner = mMath.InnerProduct(q_, pd_vec)
		cross = mMath.CrossProduct(q_, pd_vec)

		cross_val = 1.0
		if cross < 0:
			cross_val = 0.0

		return [v, inner, cross_val, pd_len]

	def preprocess_sensor(self, d_map, v_map, q_lim, vision_depth):
		d_map_ = np.divide(d_map, vision_depth, dtype=float)
		v_map_ = np.reshape(v_map, 40)

		sensor = np.append(d_map_, v_map_)

		return sensor

	def get_agent_count(self, is_train, obs):
		if is_train:
			return 1
		else:
			return len(obs['agent'])

	def isExploration(self, flag):
		self.expl = (flag and random.random() < self.exploration_rate)

class OUNoise:
	"""docstring for OUNoise"""
	def __init__(self, action_dimension, mu=0, theta=0.15, sigma=0.2):
		self.ou_action_dimension = action_dimension
		self.mu = mu
		self.theta = theta
		self.sigma = sigma
		self.ou_state = np.ones(action_dimension) * self.mu
		self.reset()

	def reset(self):
		self.ou_state = np.ones(self.ou_action_dimension) * self.mu

	def noise(self):
		x = self.ou_state
		dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(len(x))
		self.ou_state = x + dx
		return self.ou_state
