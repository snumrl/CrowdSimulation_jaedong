import sys
sys.path.append('../base')
sys.path.append('scenarios')

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import os
import csim
import numpy as np
import math
import copy
import matplotlib.pyplot as plt
from time import localtime, strftime
from constants import Constants as cst
from ddpg import DDPG
from mix import Mix
from basic import Basic
from circle import Circle
from corridor import Corridor
from crossway import Crossway
from bottleneck import Bottleneck

FLAG_REPLAY_LOAD = False
FLAG_CKPT_LOAD = True
FLAG_EVAL_LOAD = False
FLAG_EVAL_SAVE = False
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

class Experiment:
	def __init__(self, WIDTH=1280, HEIGHT=720):
		self.WIDTH = WIDTH
		self.HEIGHT = HEIGHT
		self.initGL()
		self.initFlag()

		self.SCENARIO = SCENARIO
		self.AGENT_NUM = AGENT_NUM
		self.OBSTACLE_NUM = OBSTACLE_NUM

		self.setEnvironment(SCENARIO)
		self.setNetwork()

		self.isTerm = False
		self.eval_flag = False
		self.fps = 120
		self.eval_iter = 0
		self.train_iter = 0
		self.episode_iter = 0
		self.eval_col = []
		self.eval_end = []

		self.actor_dir = actor_dir
		self.critic_dir = critic_dir

		if FLAG_CKPT_LOAD:
			self.load_network()
		else:
			print "New Network"

		if FLAG_EVAL_SAVE:
			os.makedirs(actor_dir)
			os.makedirs(critic_dir)

		if FLAG_WARMUP:
			self.WarmUp()

		self.timer_func()
		glutMainLoop()

	def initGL(self):
		argv = sys.argv
		glutInit(argv)
		glutInitWindowPosition(0,0)
		glutInitWindowSize(self.WIDTH, self.HEIGHT)
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
		self.windowID = glutCreateWindow("Crowd Simulation")
		glutDisplayFunc(self.display)
		glutReshapeFunc(self.reshape)
		glutKeyboardFunc(self.keyCB)

	def initFlag(self):
		self.flag={}
		self.flag['plot'] = False
		self.flag['quit'] = False
		self.flag['step'] = False
		self.flag['play'] = False
		self.flag['train'] = False
		self.flag['depth'] = False
		self.flag['record'] = False
		self.flag['replay'] = False
		self.flag['weight'] = False
		self.flag['greedy'] = False
		self.flag['trajectory'] = False

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

	def load_network(self):
		self.Algorithm.load_network(type='actor')
		self.Algorithm.load_network(type='critic')
		if FLAG_REPLAY_LOAD:
			self.Algorithm.load_memory()
		if FLAG_EVAL_LOAD:
			self.Algorithm.load_eval()
			self.eval_end = self.Algorithm.eval_end
			self.eval_col = self.Algorithm.eval_col

	def save_network(self, training_time=0):
		self.Algorithm.save(self.eval_end, self.eval_col)

	def Reset(self):
		self.flag['play'] = False
		self.flag['train'] = False
		self.flag['record'] = False
		self.flag['replay'] = False
		self.flag['greedy'] = False
		self.train_iter = 0
		self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)
		self.Scenario.setObjectData(self.Observe())
		for i in range(len(self.Scenario.agents)):
			self.Scenario.agents[i].trajectory = []
			self.Scenario.agents[i].trajectory_q = []

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

	def convert_to_numpy(self, obs):
		# agent_num = len(obs['agent'])
		for i in range(self.AGENT_NUM):
			obs['agent'][i]['v'] = obs['agent'][i]['v'][0]
			obs['agent'][i]['front'] = obs['agent'][i]['front'][0]
			obs['agent'][i]['d'] = np.array(obs['agent'][i]['d'])
			obs['agent'][i]['q'] = np.array(obs['agent'][i]['q'])
			obs['agent'][i]['p'] = np.array(obs['agent'][i]['p'])
			obs['agent'][i]['color'] = np.array(obs['agent'][i]['color'])
			obs['agent'][i]['d_map'] = np.array(obs['agent'][i]['d_map'])
			obs['agent'][i]['v_map'] = np.array(obs['agent'][i]['v_map'])

		obstacle_num = len(obs['obstacle'])
		for i in range(obstacle_num):
			obs['obstacle'][i]['p'] = np.array(obs['obstacle'][i]['p'])

		return obs

	def convert_to_action_double(self, action):
		action_len = len(action)
		d_theta = 0;
		d_pos = 0;
		for i in range(action_len):
			d_theta = float(action[i]['theta'])
			d_pos = float(action[i]['velocity'])
			action[i]['theta'] = copy.copy(d_theta)
			action[i]['velocity'] = copy.copy(d_pos)

		return action

	def evaluation(self):
		eval_num = 10
		col_flag = False
		avg_end, avg_col = 0, 0
		step_end, step_col = 0, 0
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

	def WarmUp(self):
		print "Warming up..."

		self.warmup_iter = cst.WARMUP_ITERATION
		for i in range(self.warmup_iter):
			if self.warmup_iter<10 or i%(self.warmup_iter/10)==0:
				print "Warmup Generation...\t"+str((i / (float)(self.warmup_iter))*100)+"%"

			self.isTerm = False
			self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)
			while not self.isTerm:
				obs, action, memory = self.Execute(action_type='GREEDY', run_type='TRAIN')
				memory['obs'] = self.convert_to_numpy(memory['obs'])

				if memory['isTerm']:
					self.isTerm = True

				self.Scenario.setObjectData(memory['obs'])
				self.Update('GREEDY', obs, action, memory)

		self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)
		self.flag['warmup']=False

		print "Warmup Done!"

	def eval_save(self):
		self.Algorithm.criticNN.save(self.critic_dir, str(self.eval_iter))
		self.Algorithm.actorNN.save(self.actor_dir, str(self.eval_iter))

	def timer_func(self, fps=120):
		if self.flag['plot']:
			plt.plot(self.eval_end)
			plt.plot(self.eval_col)
			plt.show()
			self.flag['plot'] = False

		if self.flag['replay']:
			fps = self.fps
			self.frame += 1
		else:
			fps = self.fps
			self.frame = 0
			if self.flag['train']:
				if self.isTerm:
					self.isTerm = False
					print "New Episode"
					self.Algorithm.expl_rate_decay()
					self.episode_iter += 1
					if self.episode_iter % 10 == 0:
						print "episode : ", self.episode_iter
					if self.eval_flag:
						self.eval_flag = False
						self.evaluation()
						self.eval_iter += 1
						if FLAG_EVAL_SAVE:
							self.eval_save()

					# self.AGENT_NUM += 1
					# self.OBSTACLE_NUM += 1
					self.Parser.Reset(-1, self.AGENT_NUM, self.OBSTACLE_NUM)

				obs, action, memory = self.Execute(action_type='ACTOR', run_type='TRAIN')
				memory['obs'] = self.convert_to_numpy(memory['obs'])

				self.Scenario.setObjectData(memory['obs'])

				if memory['isTerm']:
					self.isTerm = True

				self.Update('ACTOR', obs, action, memory)
				self.train_iter += 1
				if self.train_iter % 500 == 0:
					print "train iter : ", self.train_iter
					self.eval_flag=True

			elif self.flag['play']:
				if self.flag['greedy']:
					obs, action, memory = self.Execute(action_type='GREEDY', run_type='TEST')
				else:
					obs, action, memory = self.Execute(action_type='ACTOR', run_type='TEST')

				memory['obs'] = self.convert_to_numpy(memory['obs'])
				self.Scenario.setObjectData(memory['obs'])

				if memory['isTerm']:
						self.flag['play'] = False
						print "Scenario Ended"

				if self.flag['step']:
						self.flag['play'] = not self.flag['play']
						self.flag['step'] = False

			self.Scenario.record(self.flag['record'])

		glutPostRedisplay()
		glutTimerFunc(int(1000/fps), self.timer_func, fps)

	def save_plot(self):
		plt.plot(self.eval_end)
		plt.plot(self.eval_col)
		plt.savefig("../data/ckpt/actor/tmp/graph.png")
		plt.savefig("../data/ckpt/critic/tmp/graph.png")

	def change_dir(self):
		os.rename(self.actor_dir, "../data/ckpt/actor/"+str(self.Algorithm.cur_time)+"/")
		os.rename(self.critic_dir, "../data/ckpt/critic/"+str(self.Algorithm.cur_time)+"/")

	def keyCB(self, key, x, y):
		if key:
			if key == 'q':
				self.flag['quit'] = True
				glutDestroyWindow (self.windowID)
			elif key == 'd':
				if self.flag['train']:
					print "Stopped Training...!"
					self.save_network()
					if FLAG_EVAL_SAVE:
						self.save_plot()
						self.change_dir()
				else:
					print "Start Training...!"
				self.flag['train'] = not self.flag['train']
				self.flag['play'] = False
			elif key == 'z':
				self.flag['plot'] = True
			elif key == '=':
				self.fps += 10
				print "fps : ", self.fps
			elif key == '-':
				self.fps -= 10
				print "fps : ", self.fps
			elif key == 's':
				if self.flag['record']:
					print "STOP record"
				else:
					print "START record"
				self.flag['record'] = not self.flag['record']
			elif key == 'e':
				self.flag['depth'] = not self.flag['depth']
			elif self.flag['train']:
				print "Training... press 'd' to stop training"
			elif key == ' ':
				if self.flag['play']:
					print "STOP"
				else:
					print "PLAY"
				self.flag['play'] = not self.flag['play']
				self.flag['greedy'] = False
				self.flag['train'] = False
			elif key == 'r':
				self.Reset()
			elif key == 'p':
				print "START replay"
				self.frame = 0
				self.flag['replay'] = True
				self.flag['play'] = False
				self.flag['greedy'] = False
				self.flag['train'] = False
			elif key == 't':
				self.flag['trajectory'] = not self.flag['trajectory']
				print "trajectory : ", self.flag['trajectory']
			elif key == 'n':
				print "One Step"
				self.flag['play'] = True
				self.flag['greedy'] = False
				self.flag['step'] = True
			elif key == 'g':
				print "Policy : Greedy"
				self.flag['play'] = True
				self.flag['greedy'] = True
			elif key == 'm':
				self.coord_save()

	def coord_save(self):
		f = open("coord.txt", 'w')
		# f.write("agent "+str(AGENT_NUM)+"\n\n")
		traj_len = len(self.Scenario.agents[0].trajectory)
		for i in range(traj_len):
			for j in range(self.AGENT_NUM):
				f.write(str(self.Scenario.agents[j].trajectory[i][0]) + "," + str(self.Scenario.agents[j].trajectory[i][1]) + "," + str(self.Scenario.agents[j].trajectory_q[i][0]) + "," + str(self.Scenario.agents[j].trajectory_q[i][1]))
				f.write('\n')
			f.write('\n')
		f.close()

	def reshape(self, w, h):
		glViewport(0, 0, w, h)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		glOrtho(-cst.WIDTH, cst.WIDTH, -cst.HEIGHT, cst.HEIGHT, -cst.DEPTH, cst.DEPTH)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

	def render_base(self):
		for i in range(10):
			for j in range(10):
				glPushMatrix()
				glTranslatef(-50 + i * 10 + 5, -50 + j * 10 + 5 , 0)
				if (i+j) % 2 ==0:
					glColor3f(0.4, 0.4, 0.4)
				else:
					glColor3f(0.7, 0.7, 0.7)
				glBegin(GL_QUADS)
				glVertex3f( -10.0,  -10.0, 0)
				glVertex3f( -10.0,   10.0, 0)
				glVertex3f(  10.0,   10.0, 0)
				glVertex3f(  10.0,  -10.0, 0)
				glEnd()
				glPopMatrix()

	def display(self):
		glClearColor(0.8, 0.8, 0.8, 0.0)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

		gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0)

		glPushMatrix()
		self.render_base()
		glPopMatrix()

		glPushMatrix()
		if self.flag['replay'] and self.Scenario.record_size != 0:
			self.Scenario.render_record(self.frame)
		else:
			self.Scenario.render(depth = self.flag['depth'], trajectory = self.flag['trajectory'])
		glPopMatrix()

		glutSwapBuffers()

if __name__=="__main__":
	exp = Experiment(cst.WINDOW_WIDTH, cst.WINDOW_HEIGHT)

