import tensorflow as tf
import numpy as np
import tflearn

class ActorNetwork():
	"""
	Input to the network is the state, output is the action
	under a deterministic policy.
	The output layer activation is a tanh to keep the action
	"""
	def __init__(self, sess, body_dim, sensor_dim, action_dim, learning_rate, tau, batch_size):
		self.sess = sess
		self.body_dim = body_dim
		self.sensor_dim = sensor_dim
		self.s_dim = body_dim + sensor_dim
		self.a_dim = action_dim
		self.learning_rate = learning_rate
		self.tau = tau
		self.batch_size = batch_size

		# Actor Network
		self.input_body, self.input_sensor, self.out = self.create_actor_network()
		self.network_params = tf.trainable_variables()

		# Target Network
		self.target_input_body, self.target_input_sensor, self.target_out = self.create_actor_network()
		self.target_network_params = tf.trainable_variables()[
			len(self.network_params):]

		# Op for periodically updating target network with online network
		# weights
		self.update_target_network_params = \
			[self.target_network_params[i].assign(tf.multiply(self.network_params[i], self.tau) +
												  tf.multiply(self.target_network_params[i], 1. - self.tau))
				for i in range(len(self.target_network_params))]

		# This gradient will be provided by the critic network
		self.action_gradient = tf.placeholder(tf.float32, [None, self.a_dim])

		# Combine the gradients here
		self.unnormalized_actor_gradients = tf.gradients(
			self.out, self.network_params, -self.action_gradient)
		self.actor_gradients = list([x if x is None else tf.div(x, self.batch_size) for x in self.unnormalized_actor_gradients])

		# Optimization Op
		self.optimize = tf.train.AdamOptimizer(self.learning_rate).\
			apply_gradients(list(zip(self.actor_gradients, self.network_params)))

		self.num_trainable_vars = len(
			self.network_params) + len(self.target_network_params)

		self.saver = tf.train.Saver(max_to_keep=None)

	def create_actor_network(self):
		input_body = tflearn.input_data(shape=[None, 5])
		# input_sensor = tflearn.input_data(shape=[None, 3, 41, 1])
		input_sensor = tflearn.input_data(shape=[None, 45])

		body_net = tflearn.fully_connected(input_body, 64)
		body_net = tflearn.layers.normalization.batch_normalization(body_net)
		body_net = tflearn.activations.elu(body_net)

		sensor_net = tflearn.fully_connected(input_sensor, 64)
		sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		sensor_net = tflearn.activations.elu(sensor_net)
		sensor_net = tflearn.fully_connected(input_sensor, 32)
		sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		sensor_net = tflearn.activations.elu(sensor_net)

		# sensor_net = tf.transpose(input_sensor, [0,1,2,3])
		# sensor_net = tflearn.conv_2d(sensor_net, 32, [3,5], strides=1, activation='elu')
		# sensor_net = tflearn.fully_connected(sensor_net, 32)
		# sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		# sensor_net = tflearn.activations.elu(sensor_net)

		t1 = tflearn.fully_connected(body_net, 32)
		t2 = tflearn.fully_connected(sensor_net, 32)

		net = tflearn.activation(
			tf.matmul(body_net, t1.W) + tf.matmul(sensor_net, t2.W) + t2.b, activation='elu')

		w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
		out = tflearn.fully_connected(
			net, self.a_dim, activation='tanh', weights_init=w_init)

		# input_body = tflearn.input_data(shape=[None, self.body_dim])
		# input_sensor = tflearn.input_data(shape=[None, self.sensor_dim])

		# body_net = tflearn.fully_connected(input_body, 32)
		# body_net = tflearn.layers.normalization.batch_normalization(body_net)
		# body_net = tflearn.activations.elu(body_net)
		# body_net = tflearn.fully_connected(body_net, 16)
		# body_net = tflearn.layers.normalization.batch_normalization(body_net)
		# body_net = tflearn.activations.elu(body_net)

		# sensor_net = tflearn.fully_connected(input_sensor, 64)
		# sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		# sensor_net = tflearn.activations.elu(sensor_net)
		# sensor_net = tflearn.fully_connected(sensor_net, 32)
		# sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		# sensor_net = tflearn.activations.elu(sensor_net)

		# t1 = tflearn.fully_connected(body_net, 32)
		# t2 = tflearn.fully_connected(sensor_net, 32)

		# net = tflearn.activation(
		# 	tf.matmul(body_net, t1.W) + tf.matmul(sensor_net, t2.W) + t2.b, activation='elu')

		# # w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
		# w_init = tflearn.initializations.xavier(uniform=True, seed=None, dtype=tf.float32)
		# out = tflearn.fully_connected(
		# 	net, self.a_dim, activation='tanh', weights_init=w_init)

		return input_body, input_sensor, out

	def train(self, input_body, input_sensor, a_gradient):
		self.sess.run(self.optimize, feed_dict={
			self.input_body: input_body,
			self.input_sensor: input_sensor,
			self.action_gradient: a_gradient
		})

	def predict(self, input_body, input_sensor):
		return self.sess.run(self.out, feed_dict={
			self.input_body: input_body,
			self.input_sensor: input_sensor
		})

	def predict_target(self, input_body, input_sensor):
		return self.sess.run(self.target_out, feed_dict={
			self.target_input_body: input_body,
			self.target_input_sensor: input_sensor
		})

	def update_target_network(self):
		self.sess.run(self.update_target_network_params)

	def get_num_trainable_vars(self):
		return self.num_trainable_vars

	def save(self, ckpt_dir, ckpt_time):
		ckptname = ckpt_dir+"Actor Network_"+ckpt_time
		print(ckptname)
		self.saver.save(self.sess, ckptname)

	def load(self, ckpt_dir):
		ckptname = tf.train.latest_checkpoint(ckpt_dir)
		print(ckptname)
		self.saver.restore(self.sess, ckptname)


class CriticNetwork():
	"""
	Input to the network is the state and action, output is Q(s,a).
	The action must be obtained from the output of the Actor network.
	"""
	def __init__(self, sess, body_dim, sensor_dim, action_dim, learning_rate, tau, gamma, num_actor_vars):
		self.sess = sess
		self.body_dim = body_dim
		self.sensor_dim = sensor_dim
		self.s_dim = body_dim + sensor_dim
		self.a_dim = action_dim
		self.learning_rate = learning_rate
		self.tau = tau
		self.gamma = gamma

		# Create the critic network
		self.input_body, self.input_sensor, self.action, self.out = self.create_critic_network()
		self.network_params = tf.trainable_variables()[num_actor_vars:]

		# Target Network
		self.target_input_body, self.target_input_sensor, self.target_action, self.target_out = self.create_critic_network()
		self.target_network_params = tf.trainable_variables()[(len(self.network_params) + num_actor_vars):]

		# Op for periodically updating target network with online network
		# weights with regularization
		self.update_target_network_params = \
			[self.target_network_params[i].assign(tf.multiply(self.network_params[i], self.tau) \
			+ tf.multiply(self.target_network_params[i], 1. - self.tau))
				for i in range(len(self.target_network_params))]

		# Network target (y_i)
		self.predicted_q_value = tf.placeholder(tf.float32, [None, 1])

		# Define loss and optimization Op
		self.loss = tflearn.mean_square(self.predicted_q_value, self.out)
		self.optimize = tf.train.AdamOptimizer(
			self.learning_rate).minimize(self.loss)

		# Get the gradient of the net w.r.t. the action.
		# For each action in the minibatch (i.e., for each x in xs),
		# this will sum up the gradients of each critic output in the minibatch
		# w.r.t. that action. Each output is independent of all
		# actions except for one.
		self.action_grads = tf.gradients(self.out, self.action)
		self.saver = tf.train.Saver(max_to_keep=None)

	def create_critic_network(self):
		input_body = tflearn.input_data(shape=[None, 5])
		# input_sensor = tflearn.input_data(shape=[None, 3, 41, 1])
		input_sensor = tflearn.input_data(shape=[None, 45])

		action = tflearn.input_data(shape=[None, self.a_dim])

		body_net = tflearn.fully_connected(input_body, 64)
		body_net = tflearn.layers.normalization.batch_normalization(body_net)
		body_net = tflearn.activations.elu(body_net)

		sensor_net = tflearn.fully_connected(input_sensor, 64)
		sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		sensor_net = tflearn.activations.elu(sensor_net)
		sensor_net = tflearn.fully_connected(input_sensor, 32)
		sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		sensor_net = tflearn.activations.elu(sensor_net)

		# sensor_net = tf.transpose(input_sensor, [0, 1, 2, 3])
		# sensor_net = tflearn.conv_2d(sensor_net, 32, [3,5], strides=1, activation='elu')
		# sensor_net = tflearn.fully_connected(sensor_net, 32)
		# sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		# sensor_net = tflearn.activations.elu(sensor_net)

		t1 = tflearn.fully_connected(body_net, 32)
		t2 = tflearn.fully_connected(sensor_net, 32)
		t3 = tflearn.fully_connected(action, 32)

		net = tflearn.activation(
			tf.matmul(body_net, t1.W) + tf.matmul(sensor_net, t2.W) + tf.matmul(action, t3.W) + t3.b, activation='elu')

		w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
		out = tflearn.fully_connected(net, 1, weights_init=w_init)
		# input_body = tflearn.input_data(shape=[None, self.body_dim])
		# input_sensor = tflearn.input_data(shape=[None, self.sensor_dim])
		# action = tflearn.input_data(shape=[None, self.a_dim])

		# body_net = tflearn.fully_connected(input_body, 32)
		# body_net = tflearn.layers.normalization.batch_normalization(body_net)
		# body_net = tflearn.activations.elu(body_net)
		# body_net = tflearn.fully_connected(body_net, 16)
		# body_net = tflearn.layers.normalization.batch_normalization(body_net)
		# body_net = tflearn.activations.elu(body_net)

		# sensor_net = tflearn.fully_connected(input_sensor, 64)
		# sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		# sensor_net = tflearn.activations.elu(sensor_net)
		# sensor_net = tflearn.fully_connected(sensor_net, 32)
		# sensor_net = tflearn.layers.normalization.batch_normalization(sensor_net)
		# sensor_net = tflearn.activations.elu(sensor_net)

		# t1 = tflearn.fully_connected(body_net, 32)
		# t2 = tflearn.fully_connected(sensor_net, 32)
		# t3 = tflearn.fully_connected(action, 32)

		# net = tflearn.activation(
		# 	tf.matmul(body_net, t1.W) + tf.matmul(sensor_net, t2.W) + tf.matmul(action, t3.W) + t3.b, activation='elu')

		# # w_init = tflearn.initializations.uniform(minval=-0.003, maxval=0.003)
		# w_init = tflearn.initializations.xavier(uniform=True, seed=None, dtype=tf.float32)
		# out = tflearn.fully_connected(net, 1, weights_init=w_init)

		return input_body, input_sensor, action, out

	def train(self, input_body, input_sensor, action, predicted_q_value):
		return self.sess.run([self.out, self.optimize], feed_dict={
			self.input_body: input_body,
			self.input_sensor: input_sensor,
			self.action: action,
			self.predicted_q_value: predicted_q_value
		})

	def predict(self, input_body, input_sensor, action):
		return self.sess.run(self.out, feed_dict={
			self.input_body: input_body,
			self.input_sensor: input_sensor,
			self.action: action
		})

	def predict_target(self, input_body, input_sensor, action):
		return self.sess.run(self.target_out, feed_dict={
			self.target_input_body: input_body,
			self.target_input_sensor: input_sensor,
			self.target_action: action
		})

	def action_gradients(self, input_body, input_sensor, actions):
		return self.sess.run(self.action_grads, feed_dict={
			self.input_body: input_body,
			self.input_sensor: input_sensor,
			self.action: actions
		})

	def update_target_network(self):
		self.sess.run(self.update_target_network_params)

	def save(self, ckpt_dir, ckpt_time):
		ckptname = ckpt_dir+"Critic Network_"+ckpt_time
		print(ckptname)
		self.saver.save(self.sess, ckptname)

	def load(self, ckpt_dir):
		ckptname = tf.train.latest_checkpoint(ckpt_dir)
		print(ckptname)
		self.saver.restore(self.sess, ckptname)
