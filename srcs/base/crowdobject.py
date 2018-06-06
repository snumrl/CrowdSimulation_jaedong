class CrowdObject:
	def __init__(self):
		return

	def reset(self, state):
		raise NotImplementedError("Must Override function initialize")

	def action(self, action):
		raise NotImplementedError("Must Override function initialize")

	def render(self):
		raise NotImplementedError("Must Override function initialize")