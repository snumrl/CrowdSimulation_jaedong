import tensorflow as tf
import numpy as np
from time import localtime, strftime
from abc import abstractmethod

def weight_variable(name, shape):
	xavier = tf.contrib.layers.xavier_initializer()
	return tf.get_variable(name, shape=shape, initializer = xavier)

def bias_variable(name, shape):
	xavier = tf.contrib.layers.xavier_initializer()
	return tf.get_variable(name, shape=shape, initializer = xavier)

def variable_summaries(var, name):
	#mean = tf.reduce_mean(var)
	#stddev = tf.sqrt(tf.reduce_sum(tf.square(var - mean)))

	# tf.summary.scalar('mean/' + name, mean)
	# tf.summary.scalar('sttdev/', stddev)
	# tf.summary.scalar('max/', tf.reduce_max(var))
	# tf.summary.scalar('min/', tf.reduce_min(var))
	tf.summary.histogram(name, var)

class Layer:
	def __init__(self,
		name,
		is_copy=False):
		self.name = name
		self.is_copy = is_copy

		self.init_done = False
		self.assigner_done = False

	@abstractmethod
	def initialize(self, tensor_in):
		raise NotImplementedError

	@abstractmethod
	def copy(self):
		raise NotImplementedError

	def create_assigner(self, layer):
		if not self.is_copy:
			raise Exception("This layer isn't a copied one!")
		if not self.init_done:
			raise Exception("Must Initialize First!")

		self.tau = tf.placeholder(tf.float32)

		self.W_assign = self.W.assign(
			tf.add(
				tf.multiply(tf.subtract(1.0, self.tau), self.W),
				tf.multiply(self.tau, layer.W)
				)
			)
		self.b_assign = self.b.assign(
			tf.add(
				tf.multiply(tf.subtract(1.0, self.tau), self.b),
				tf.multiply(self.tau, layer.b)
				)
			)
		self.assigner_done = True

	def assign(self, sess, tau):
		if not self.assigner_done :
			raise Exception('create assigner function must be called first')
		sess.run(self.W_assign, feed_dict={
			self.tau : tau
			})

		sess.run(self.b_assign, feed_dict={
			self.tau : tau
			})

class FullyConnLayer(Layer):
	def __init__(self,
		name,
		tensor_in,
		dim_in=0,
		dim_out=0,
		act_fn = tf.nn.relu,
		is_copy=False):
		Layer.__init__(self,name, is_copy)

		self.dim_in = dim_in
		self.dim_out = dim_out
		self.act_fn = act_fn

		self.initialize(tensor_in)

	def initialize(self, tensor_in):
		self.W = weight_variable(
			name=self.name+'/weights',
			shape=[self.dim_in, self.dim_out])
		variable_summaries(self.W, self.name+'/weights')

		self.b = bias_variable(
			name=self.name+"/biases",
			shape=[self.dim_out])
		variable_summaries(self.b, self.name+"/biases")

		self.z = tf.matmul(tensor_in, self.W) + self.b
		tf.summary.histogram(self.name+'/pre_activations', self.z)

		if self.act_fn is None:
			self.h = self.z
		else:
			self.h = self.act_fn(self.z)

		tf.summary.histogram(self.name+'/activations', self.h)

		self.init_done=True

	def copy(self, tensor_in):
		new_layer = FullyConnLayer(self.name+"_copy",
			tensor_in,
			self.dim_in,
			self.dim_out,
			self.act_fn,
			True)

		new_layer.create_assigner(self)

		return new_layer

class ConvolutionLayer(Layer):
	def __init__(self,
		name,
		tensor_in,
		filter_size=0,
		dim_in=0,
		dim_out=0,
		stride=1,
		padding='SAME',
		is_copy=False):

		Layer.__init__(self,name, is_copy)

		self.filter_size = filter_size
		self.dim_in = dim_in
		self.dim_out = dim_out
		self.stride = stride
		self.padding = padding

		self.initialize(tensor_in)

	def initialize(self, tensor_in):
		self.W = weight_variable(
			name=self.name+"_W",
			shape=[self.filter_size, self.dim_in, self.dim_out])

		self.b = bias_variable(
			name=self.name+"_b",
			shape=[self.dim_out])

		self.z = tf.nn.conv1d(
			tensor_in,
			self.W,
			stride=self.stride,
			padding=self.padding) + self.b

		self.h = tf.nn.relu(self.z)

		self.init_done=True

	def copy(self, tensor_in):
		new_layer = ConvolutionLayer(self.name+"_copy",
			tensor_in,
			self.filter_size,
			self.dim_in,
			self.dim_out,
			self.stride,
			self.padding,
			True)

		new_layer.create_assigner(self)

		return new_layer

class MaxPoolLayer(Layer):
	def __init__(self,
		name,
		tensor_in=None,
		pool_size=0,
		stride=0,
		padding = 'SAME',
		is_copy=False,
		):

		Layer.__init__(self,name, is_copy)

		self.name = name
		self.pool_size = pool_size
		self.stride = stride
		self.padding = padding

		self.initialize(tensor_in)

	def initialize(self, tensor_in):
		self.h = tf.layers.max_pooling1d(
			inputs=tensor_in,
			pool_size = self.pool_size,
			strides=self.stride,
			padding=self.padding)

		self.init_done=True

	def copy(self, tensor_in):
		return MaxPoolLayer(self.name,
			tensor_in,
			self.pool_size,
			self.stride,
			self.padding,
			True)

	def create_assigner(self, layer):
		raise Exception("No parameters inside MaxPoolLayer")

	def assign(self, sess, tau):
		#raise Exception("No parameters inside MaxPoolLayer")
		return


class NeuralNetwork:
	def __init__(self, name, device='/cpu:0'):
		self.name = name
		self.graph= tf.Graph()
		self.saver = None

	@abstractmethod
	def initialize(self):
		raise NotImplementedError("Must Override function initialize")

	@abstractmethod
	def train(self, state, output):
		raise NotImplementedError

	@abstractmethod
	def loss(self, state):
		raise NotImplementedError

	@abstractmethod
	def predict(self, state):
		raise NotImplementedError

	def save(self, ckpt_dir, ckpt_time):
		ckptname = ckpt_dir+self.name+"_"+ckpt_time
		print ckptname
		self.saver.save(self.sess, ckptname)

	def load(self, ckpt_dir):
		ckptname = tf.train.latest_checkpoint(ckpt_dir)
		print ckptname
		self.saver.restore(self.sess, ckptname)

	def close(self):
		self.sess.close()
