#ifndef PARSER_H
#define PARSER_H

#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/dict.hpp>

#include "Env.h"

using namespace boost::python;
using namespace std;

class Parser{
	protected:
		Env* _env;

	public:
		Parser(string scenario);
		~Parser();

		void Reset(int idx);
		dict Step(list action, bool isTest);
		dict Observe();
		list dmap_to_list(double* d);
};


#endif
