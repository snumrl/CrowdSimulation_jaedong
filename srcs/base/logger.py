import inspect
import os

from constants import Constants as cst

def log(msg):
	if cst.DEBUG_MODE is False:
		return

	filename = inspect.getfile(inspect.currentframe().f_back)
	filename = os.path.basename(filename)
	lineno = str(inspect.currentframe().f_back.f_lineno)
	print filename+"("+lineno+") :",
	print msg