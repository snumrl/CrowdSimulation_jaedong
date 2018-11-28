#ifndef PARSER_H
#define PARSER_H

#include <boost/python.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/numpy.hpp>
#include <string>

#include "Env.h"

class Parser{
	protected:
		Env* _env;

		int agent_num;
		int obstacle_num;
		int wall_num;

	public:
		Parser(std::string scenario, int a, int o);
		~Parser();

		void Reset(int idx, int a, int o);
		boost::python::dict Step(boost::python::numpy::ndarray& action_, bool isTest);
		boost::python::dict Observe();
};


#endif
