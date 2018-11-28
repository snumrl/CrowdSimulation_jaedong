import os
try:
	from mpi4py import MPI
except ImportError:
	MPI = None

import sys
sys.path.append('baselines')

from baselines.common import tf_util as U
from baselines import logger
from baselines.bench import Monitor
from baselines.common import set_global_seeds

from time import localtime, strftime
from env import Env

def make_env(seed=None):
	reward_scale = 1.0

	rank = MPI.COMM_WORLD.Get_rank()
	myseed = seed + 1000 * rank if seed is not None else None
	set_global_seeds(myseed)
	env = Env()

	logger_path = None if logger.get_dir() is None else os.path.join(logger.get_dir(), str(rank))
	env = Monitor(env, logger_path, allow_early_resets=True)
	env.seed(seed)
	if reward_scale != 1.0:
		from baselines.common.retro_wrappers import RewardScaler
		env = RewardScaler(env, reward_scale)

	return env


def train(num_timesteps, path=None):
	from baselines.ppo1 import mlp_policy, pposgd_simple
	U.make_session(num_cpu=1).__enter__()
	def policy_fn(name, ob_space, ac_space):
		return mlp_policy.MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
			hid_size=64, num_hid_layers=3)

	env = make_env()
	print("prev path : ", path)
	pi = pposgd_simple.learn(env, policy_fn,
			max_timesteps=num_timesteps,
			timesteps_per_actorbatch=512,
			clip_param=0.2, entcoeff=0.0,
			optim_epochs=10,
			optim_stepsize=1e-4,
			optim_batchsize=64,
			gamma=0.95,
			lam=0.95,
			schedule='linear',
			model_path=path
		)

	env.env.plotSave()

	# U.save_state("../data/ckpt/network/test001")

	return pi

def main():
	logger.configure()
	# path_ = None
	path_ = "../data/ckpt/network/1127a/15.0"
	train(num_timesteps=1e6, path=path_)

if __name__ == '__main__':
	main()
