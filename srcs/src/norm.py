def clamp(n, minn, maxn):
	return max(min(maxn, n), minn)

class Normalizer:
	def __init__(self,
				 real_val_max, real_val_min,
				 norm_val_max, norm_val_min,
				 apply_clamp=True):
		self.set_real_range(real_val_max, real_val_min)
		self.set_norm_range(norm_val_max, norm_val_min)
		self.apply_clamp = apply_clamp
		self.dim = len(real_val_max)

	def set_real_range(self, real_val_max, real_val_min):
		self.real_val_max = real_val_max
		self.real_val_min = real_val_min
		self.real_val_diff = real_val_max - real_val_min
		self.real_val_diff_inv = 1.0 / self.real_val_diff
		#
		# Check if wrong values exist in the setting
		# e.g. min <= max or abs(max-min) is too small
		#
		for v in self.real_val_diff:
			if v <= 0.0 or abs(v) < 1.0e-08:
				raise Exception('Normalizer', 'wrong values')

	def set_norm_range(self, norm_val_max, norm_val_min):
		self.norm_val_max = norm_val_max
		self.norm_val_min = norm_val_min
		self.norm_val_diff = norm_val_max - norm_val_min
		self.norm_val_diff_inv = 1.0 / self.norm_val_diff
		#
		# Check if wrong values exist in the setting
		# e.g. min <= max or abs(max-min) is too small
		#
		for v in self.norm_val_diff:
			if v <= 0.0 or abs(v) < 1.0e-08:
				raise Exception('Normalizer', 'wrong values')

	def real_to_norm(self, val):
		val_0_1 = (val - self.real_val_min) * self.real_val_diff_inv
		if self.apply_clamp:
			self._clamp(val_0_1)
		return self.norm_val_min + self.norm_val_diff * val_0_1

	def norm_to_real(self, val):
		val_0_1 = (val - self.norm_val_min) * self.norm_val_diff_inv
		if self.apply_clamp:
			self._clamp(val_0_1)
		return self.real_val_min + self.real_val_diff * val_0_1

	def _clamp(self, val):
		for i in range(len(val)):
			val[i] = clamp(val[i], 0.0, 1.0)
