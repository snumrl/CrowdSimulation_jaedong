import numpy as np
import random

from constants import Constants as cst

class ReplayMemory:
	def __init__(self, algorithm):
		self.memory = dict()
		self.mSize=cst.MEMORY_SIZE

		if algorithm == 'DDPG':
			self.memory['actor'] = []
			self.memory['critic'] = []

	def addMemory(self, buffername, s, a, ns, r, t):
		newMemory = {}
		# newMemory['state']=s
		# newMemory['action']=a
		# newMemory['next_state']=ns
		newMemory['state']=s['agent'][0]
		newMemory['action']=a[0]
		newMemory['next_state']=ns['agent'][0]
		newMemory['reward']=r
		newMemory['term'] = t
		self.memory[buffername].append(newMemory)
		if len(self.memory[buffername]) > self.mSize:
			self.memory[buffername].pop(0)

	def getRandomMemories(self, buffername, mbsize=cst.BATCH_SIZE):
		if mbsize>len(self.memory[buffername]):
			mbsize=len(self.memory[buffername])

		mb = []
		for i in range(mbsize):
			mb.append(random.choice(self.memory[buffername]))

		return mb

	def getMemorySize(self):
		return len(self.memory['actor'])
