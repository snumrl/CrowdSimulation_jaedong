#include <iostream>
#include <vector>

#include "parser.h"
#include "scenario/Basic.h"
#include "scenario/Corridor.h"
#include "scenario/Crossway.h"
#include "scenario/Circle.h"
#include "scenario/Bottleneck.h"

using namespace boost::python;
using namespace std;

#define AGENT_NUM 20
#define OBSTACLE_NUM 0

static double time_step = 1.0;

Parser::Parser(string Scenario)
{
	cout << "Scenario : " << Scenario << endl;

	if(Scenario.compare("Basic") == 0){
		_env = new Basic(AGENT_NUM, OBSTACLE_NUM);
	}
	else if(Scenario.compare("Corridor") == 0){
		_env = new Corridor(AGENT_NUM, OBSTACLE_NUM);
	}
	else if(Scenario.compare("Crossway") == 0){
		_env = new Crossway(AGENT_NUM, OBSTACLE_NUM);
	}
	else if(Scenario.compare("Circle") == 0){
		_env = new Circle(AGENT_NUM, OBSTACLE_NUM);
	}
	else if(Scenario.compare("Bottleneck") == 0){
		_env = new Bottleneck(AGENT_NUM, OBSTACLE_NUM);
	}
}

Parser::~Parser()
{
	cout << "~Parser" << endl;

	delete _env;
}

void Parser::Reset(int idx)
{
	_env -> Reset(idx);
}

dict Parser::Step(list action, bool isTest)
{
	for(int i=0; i<AGENT_NUM; i++)
	{
		double theta = extract<double>(action[i]["theta"]);
		double velocity = extract<double>(action[i]["velocity"]);
		bool stop = extract<bool>(action[i]["stop"]);
		_env->setAction(i, theta, velocity, stop);
	}

	_env->Update();

	dict memory;
	memory["obs"] = Observe();
	memory["isTerm"] = _env->isTerm(isTest);
	memory["reward"] = _env->getReward();

	return memory;
}

dict Parser::Observe()
{
	dict obs;
	list agent_state;
	list obstacle_state;

	vector<Agent*> agents = _env -> Observe();
	vector<Obstacle*> obstacles = _env-> getObstacles();

	Agent* cur_agent;
	for(int i=0; i<AGENT_NUM; i++)
	{
		cur_agent = agents.at(i);

		dict cur_agent_state;

		double* p = cur_agent->getP();
		list p_list;
		p_list.append(p[0]);
		p_list.append(p[1]);
		cur_agent_state["p"] = p_list;

		double* q = cur_agent->getQ();
		list q_list;
		q_list.append(q[0]);
		q_list.append(q[1]);
		cur_agent_state["q"] = q_list;

		double* d = cur_agent->getD();
		list d_list;
		d_list.append(d[0]);
		d_list.append(d[1]);
		cur_agent_state["d"] = d_list;

		list v_list;
		v_list.append(cur_agent->getV());
		cur_agent_state["v"] = v_list;

		list f_list;
		f_list.append(cur_agent->getFront());
		cur_agent_state["front"] = f_list;

		double* c = cur_agent->getColor();
		list c_list;
		c_list.append(c[0]);
		c_list.append(c[1]);
		c_list.append(c[2]);
		cur_agent_state["color"] = c_list;

		list dmap_list;
		double* cur_dmap = cur_agent->getDmap();
		for(int i=0; i<20; i++)
		{
			dmap_list.append(cur_dmap[i]);
		}
		cur_agent_state["d_map"] = dmap_list;

		list vmap_list;
		double* cur_vmap = cur_agent->getVmap();
		for(int i=0; i<40; i++)
		{
			vmap_list.append(cur_vmap[i]);
		}
		cur_agent_state["v_map"] = vmap_list;

		agent_state.append(cur_agent_state);
	}

	Obstacle* cur_obstacle;
	for(int i=0; i<OBSTACLE_NUM; i++)
	{
		cur_obstacle = obstacles.at(i);

		dict cur_obstacle_state;

		double* p = cur_obstacle->getP();
		list p_list;
		p_list.append(p[0]);
		p_list.append(p[1]);
		cur_obstacle_state["p"] = p_list;

		obstacle_state.append(cur_obstacle_state);
	}

	obs["agent"] = agent_state;
	obs["obstacle"] = obstacle_state;

	return obs;
}

list Parser::dmap_to_list(double* d)
{
	list d_;
	for(int i=0; i<20; i++)
	{
		d_.append(d[i]);
	}

	return d_;
}

BOOST_PYTHON_MODULE(csim)
{
	class_<Parser>("Parser", init<std::string>())
		.def("Reset", &Parser::Reset)
		.def("Step", &Parser::Step)
		.def("Observe", &Parser::Observe);
}


