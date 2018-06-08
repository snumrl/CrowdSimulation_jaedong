import sys
sys.path.append('../base')

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import csim
import numpy as np
from constants import Constants as cst
from ddpg import DDPG
from basic import Basic
from corridor import Corridor

FLAG_USE_RECENT_CKPT = False
FLAG_USE_REPLAY_MEMORY = False
FLAG_WARMUP_FOR_TRAINING = False

class Experiment:
	def __init__(self, WIDTH=1260, HEIGHT=680):
		self.WIDTH = WIDTH
		self.HEIGHT = HEIGHT
		self.initGL()
		self.initFlag()

		#SCENARIO = 'Basic'
		SCENARIO = 'Corridor'
		# SCENARIO = 'Bottleneck'
		# SCENARIO = 'Crossway'

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

		obs = self.Observe()

		if SCENARIO == 'Basic':
			self.Scenario = Basic(obs)
		elif SCENARIO == 'Corridor':
			self.Scenario = Corridor(obs)
		# elif SCENARIO == 'Bottleneck':
		# 	self.Scenario = Bottleneck(obs)
		# elif SCENARIO == 'Crossway':
		# 	self.Scenario = Crossway(obs)

	def setNetwork(self):
		network_dim = []
		network_dim.append(4)
		network_dim.append(60)
		network_dim.append(2)
		self.Algorithm = DDPG(network_dim)

	def Reset(self):
		self.flag['play'] = False
		self.flag['train'] = False
		self.flag['record'] = False
		self.flag['replay'] = False
		self.flag['greedy'] = False

		self.Parser.Reset(-1)
		obs = self.Observe()
		self.Scenario.setObjectData(obs)

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
		for i in range(action_len):
			action[i]['theta'] = float(action[i]['theta'])
			action[i]['velocity'] = float(action[i]['velocity'])

		return action

	def timer_func(self, fps=120):

		if self.flag['replay']:
			fps = 40
			self.frame += 1
		else:
			fps = 120
			self.frame = 0
			if self.flag['train']:
				if self.isTerm:
					print "New Episode"
					self.Algorithm.expl_rate_decay()
					self.isTerm = False

				obs, action, memory = self.Execute(action_type='ACTOR', run_type='TRAIN')
				memory['obs'] = self.convert_to_numpy(memory['obs'])

				self.Scenario.setObjectData(memory['obs'])

				if memory['isTerm']:
					self.isTerm = True
					self.Parser.Reset(-1)

				self.Update('ACTOR', obs, action, memory)

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

	def reshape(self, w, h):
		glViewport(0, 0, w, h)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		glOrtho(-self.WIDTH/2, self.WIDTH/2, -self.HEIGHT/2, self.HEIGHT/2, -30, 30)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

	def display(self):
		glClearColor(0.9, 0.9, 0.9, 0.0)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

		gluLookAt(0, 0, 30, 0, 0, 0, 0, 1, 0)

		glPushMatrix()
		if self.flag['replay'] and self.Scenario.record_size != 0:
			self.Scenario.render_record(self.frame)
		else:
			self.Scenario.render(depth = self.flag['depth'], trajectory = self.flag['trajectory'])
		glPopMatrix()

		glutSwapBuffers()

if __name__=="__main__":
	exp = Experiment()

