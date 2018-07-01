import sys
sys.path.append('../base')

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import csim
import numpy as np
import math
import copy
from constants import Constants as cst
from ddpg import DDPG
from basic import Basic
from corridor import Corridor
from circle import Circle
from crossway import Crossway
from bottleneck import Bottleneck

FLAG_USE_RECENT_CKPT = True
FLAG_USE_REPLAY_MEMORY = True
FLAG_WARMUP_FOR_TRAINING = False

class Experiment:
	def __init__(self, WIDTH=1200, HEIGHT=800):
		self.WIDTH = WIDTH
		self.HEIGHT = HEIGHT
		self.initGL()
		self.initFlag()

		# SCENARIO = 'Basic'
		SCENARIO = 'Corridor'
		# SCENARIO = 'Bottleneck'
		# SCENARIO = 'Crossway'
		# SCENARIO = 'Circle'
		# SCENARIO = 'Valley'

		self.scen = SCENARIO

		self.setEnvironment(SCENARIO)
		self.setNetwork()

		if FLAG_USE_RECENT_CKPT:
			print "Load Network"
			self.load_network(m_replay = FLAG_USE_REPLAY_MEMORY)
		else:
			print "New Network"

		if FLAG_WARMUP_FOR_TRAINING:
			self.WarmUp()

		self.isTerm = False
		self.train_iter = 0
		self.episode_iter = 0

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
		self.flag['quit'] = False
		self.flag['play'] = False
		self.flag['train'] = False
		self.flag['greedy'] = False
		self.flag['step'] = False
		self.flag['weight'] = False
		self.flag['depth'] = False
		self.flag['record'] = False
		self.flag['replay'] = False
		self.flag['trajectory'] = False

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
		elif SCENARIO == 'Crossway':
			self.Scenario = Crossway(obs)
		elif SCENARIO == 'Circle':
			self.Scenario = Circle(obs)
		elif SCENARIO == 'Bottleneck':
			self.Scenario = Bottleneck(obs)


	def setNetwork(self):
		network_dim = []
		network_dim.append(cst.AGENT_BODY_DIMENSION)
		network_dim.append(cst.AGENT_SENSOR_DIMENSION*3)
		network_dim.append(cst.AGENT_ACTION_DIMENSION)
		self.Algorithm = DDPG(network_dim)

	def Reset(self):
		self.flag['play'] = False
		self.flag['train'] = False
		self.flag['record'] = False
		self.flag['replay'] = False
		self.flag['greedy'] = False
		self.train_iter = 0
		self.episode_iter = 0
		self.Parser.Reset(-1)
		obs = self.Observe()
		self.Scenario.setObjectData(obs)
		for i in range(len(self.Scenario.agents)):
			self.Scenario.agents[i].trajectory = []

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
		print "Warming up..."

		self.warmup_iter = cst.WARMUP_ITERATION
		for i in range(self.warmup_iter):
			if self.warmup_iter<10 or i%(self.warmup_iter/10)==0:
				print "Warmup Generation...\t"+str((i / (float)(self.warmup_iter))*100)+"%"

			self.isTerm = False
			self.Parser.Reset(-1)
			while not self.isTerm:
				obs, action, memory = self.Execute(action_type='GREEDY', run_type='TRAIN')
				memory['obs'] = self.convert_to_numpy(memory['obs'])

				if memory['isTerm']:
					self.isTerm = True

				self.Scenario.setObjectData(memory['obs'])
				self.Update('GREEDY', obs, action, memory)

		self.flag['warmup']=False
		self.Parser.Reset(-1)

		print "Warmup Done!"

	def convert_to_numpy(self, obs):
		agent_num = len(obs['agent'])
		for i in range(agent_num):
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
			# if d_theta > 0.25 * math.pi:
			# 	d_theta = 0.25 * math.pi
			# if d_theta < -0.25 * math.pi:
			# 	d_theta = -0.25 * math.pi

			# if d_pos > 2.0:
			# 	d_pos = 2.0
			# if d_pos <-0.2:
			# 	d_pos = -0.2

			action[i]['theta'] = copy.copy(d_theta)
			action[i]['velocity'] = copy.copy(d_pos)

		return action

	def timer_func(self, fps=120):
		if self.flag['replay']:
			fps = 40
			self.frame += 1
		else:
			fps = 240
			self.frame = 0
			if self.flag['train']:
				if self.isTerm:
					print "New Episode"
					self.Algorithm.expl_rate_decay()
					self.isTerm = False
					self.episode_iter += 1
					if self.episode_iter % 10 == 0:
						print "episode : ", self.episode_iter

				obs, action, memory = self.Execute(action_type='ACTOR', run_type='TRAIN')
				memory['obs'] = self.convert_to_numpy(memory['obs'])

				print "Action  : ", action[0]

				self.Scenario.setObjectData(memory['obs'])

				if memory['isTerm']:
					self.isTerm = True
					self.Parser.Reset(-1)

				self.Update('ACTOR', obs, action, memory)
				self.train_iter += 1
				if self.train_iter % 2000 == 0:
					print "train iter : ", self.train_iter

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

	def load_network(self, m_replay=False):
		self.Algorithm.load_network(type='actor')
		self.Algorithm.load_network(type='critic')
		if m_replay:
			self.Algorithm.load_memory()
			self.Algorithm.load_eval()

	def save_network(self, m_replay=False, training_time=0, eval_list=None):
		self.Algorithm.save(m_replay, training_time, eval_list)

	def mouseCB(self, button, state, x, y):
		pass

	def keyCB(self, key, x, y):
		if key:
			if key == 'q':
				self.flag['quit'] = True
				glutDestroyWindow (self.windowID)
			elif key == 'd':
				if self.flag['train']:
					print "Stopped Training...!"
					self.save_network(m_replay = True)
				else:
					print "Start Training...!"
				self.flag['train'] = not self.flag['train']
				self.flag['play'] = False
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
				self.flag['replay'] = True
				self.frame = 0
				print "START replay"
				self.flag['play'] = False
				self.flag['greedy'] = False
				self.flag['train'] = False
			elif key == 't':
				self.flag['trajectory'] = not self.flag['trajectory']
				print "traj : ", self.flag['trajectory']
			elif key == 'n':
				print "One Step"
				self.flag['play'] = True
				self.flag['greedy'] = False
				self.flag['step'] = True
			elif key == 'g':
				print "Policy : Greedy"
				self.flag['play'] = True
				self.flag['greedy'] = True
			# elif key == 'm':
			# 	print "memorize"
			# 	if len(self.Scenario.record_agent_p) > 0:
			# 		self.Memorize()

	# def Memorize(self):
	# 	f = open(".../data/ckpt/memorize/" + "memory_"+self.scen+, 'w')
	# 	f.write("agent "+len(self.Scenario.))
	# 	f.close()

	def reshape(self, w, h):
		glEnable(GL_DEPTH_TEST)
		glDepthFunc(GL_LESS)
		glViewport(0, 0, w, h)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		glOrtho(-30, 30, -20, 20, -5, 5)
		# gluPerspective(5.0, 1260.0/680.0, 0.1, 1000)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

	def render_base(self):
		for i in range(10):
			for j in range(10):
				glPushMatrix()
				glTranslatef(-1000 + i * 200 + 100, -1000 + j * 200 + 100, 0)
				if (i+j) % 2 ==0:
					glColor3f(0.5, 0.5, 0.5)
				else:
					glColor3f(0.7, 0.7, 0.7)
				glBegin(GL_QUADS)
				glVertex3f( -100.0,  -100.0, 0)
				glVertex3f( -100.0,   100.0, 0)
				glVertex3f(  100.0,   100.0, 0)
				glVertex3f(  100.0,  -100.0, 0)
				glEnd()
				glPopMatrix()

	def display(self):
		glClearColor(0.8, 0.8, 0.8, 0.0)
		glClearDepth(1.0)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

		gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0)

		# glPushMatrix()
		# self.render_base()
		# glPopMatrix()

		glPushMatrix()
		if self.flag['replay'] and self.Scenario.record_size != 0:
			self.Scenario.render_record(self.frame)
		else:
			self.Scenario.render(depth = self.flag['depth'], trajectory = self.flag['trajectory'])
		glPopMatrix()

		glutSwapBuffers()

if __name__=="__main__":
	exp = Experiment()

