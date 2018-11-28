#include <iostream>
#include <vector>
#include <ctime>
#include "parser.h"
#include "scenarios/Basic.h"
#include "scenarios/Passing.h"
#include "scenarios/Dot.h"
#include "scenarios/Hallway.h"

using namespace std;

namespace p = boost::python;
namespace np = boost::python::numpy;

Parser::Parser(string Scenario, int a, int o)
{
	np::initialize();
	Py_Initialize();

	cout << "Scenario : " << Scenario << endl;

	agent_num = a;
	obstacle_num = o;

	if(Scenario.compare("Basic") == 0)
		_env = new Basic(agent_num, obstacle_num);
	else if(Scenario.compare("Passing") == 0)
		_env = new Passing(agent_num, obstacle_num);
	else if(Scenario.compare("Dot") == 0)
		_env = new Dot(agent_num, obstacle_num);
	else if(Scenario.compare("Hallway") == 0)
		_env = new Hallway(agent_num, obstacle_num);
}

Parser::~Parser()
{
	cout << "~Parser" << endl;
	delete _env;
}

void Parser::Reset(int idx,  int a, int o)
{
	_env -> Reset(idx);
}

p::dict Parser::Step(np::ndarray& action_, bool isTest)
{
	float* a = reinterpret_cast<float*>(action_.get_data());

	double w, v_x, v_y;
	for(int i=0; i<action_.shape(0); i++)
	{
		int idx_offset = action_.shape(1) * i;

		w = a[idx_offset];
		v_x = a[idx_offset+1];
		v_y = a[idx_offset+2];

		_env->setAction(i, w, v_x, v_y);
	}

	clock_t begin = clock();
	_env->Update();
	clock_t end = clock();
	double elapsed_secs = double(end-begin)/CLOCKS_PER_SEC;

	p::dict memory;
	clock_t begin2 = clock();
	memory["obs"] = Observe();
	clock_t end2 = clock();
	double elapsed_secs2 = double(end2-begin2)/CLOCKS_PER_SEC;
	if(elapsed_secs+elapsed_secs2 > 0.08){
		std::cout << "update time : " << elapsed_secs << std::endl;
		std::cout << "observe time : " << elapsed_secs2 << std::endl;
	}

	memory["isCol"] = _env->isCol();
	memory["isTerm"] = _env->isTerm(isTest);
	memory["reward"] = _env->getReward();
	double* reward_sep = _env->getRewardSep();
	p::list reward_sep_list;
	for(int i=0; i<4; i++)
		reward_sep_list.append(reward_sep[i]);
	memory["reward_sep"] = reward_sep_list;

	return memory;
}

p::dict Parser::Observe()
{
	p::dict obs;
	p::list agent_state;
	p::list obstacle_state;
	p::list wall_state;

	vector<Agent*> agents = _env -> Observe();
	vector<Obstacle*> obstacles = _env-> getObstacles();
	vector<Wall*> walls = _env-> getWalls();

	Agent* cur_agent;
	for(int i=0; i<agent_num; i++)
	{
		cur_agent = agents.at(i);
		p::dict cur_agent_data;

		double r_data[10]; // r, p, d, front, color
		cur_agent->getRenderData(r_data);
		p::list render_list;
		for(int j=0; j<10; j++)
			render_list.append(r_data[j]);

		double body_data[14]; // body
		cur_agent->getBodyState(body_data);
		p::list body_list;
		for(int j=0; j<14; j++)
			body_list.append(body_data[j]);

		double* sensor_data; // sensor
		sensor_data = cur_agent->getVision();
		p::list sensor_list;
		double vision_depth = cur_agent->getVisionDepth();
		for(int j=0; j<36; j++)
			sensor_list.append(sensor_data[j]/vision_depth);

		double* velocity_data; // sensor
		velocity_data = cur_agent->getVisionVel();
		p::list velocity_list;
		for(int j=0; j<36; j++)
			velocity_list.append(velocity_data[j]);

		double* sensor_offset_data;
		sensor_offset_data = cur_agent->getVisionOffset();
		p::list offset_list;
		for(int j=0; j<36; j++)
			offset_list.append(sensor_offset_data[j]);

		cur_agent_data["render_data"] = render_list;
		cur_agent_data["body_state"] =  body_list;
		cur_agent_data["sensor_state"] =  sensor_list;
		cur_agent_data["velocity_state"] =  velocity_list;
		cur_agent_data["offset_data"] = offset_list;

		agent_state.append(cur_agent_data);
	}

	Obstacle* cur_obstacle;
	for(int i=0; i<obstacle_num; i++)
	{
		cur_obstacle = obstacles.at(i);

		p::dict cur_obstacle_state;

		double* p = cur_obstacle->getP();
		p::list p_list;
		p_list.append(p[0]);
		p_list.append(p[1]);
		cur_obstacle_state["p"] = p_list;

		double* r = cur_obstacle->getR();
		p::list r_list;
		r_list.append(r[0]);
		r_list.append(r[1]);
		cur_obstacle_state["r"] = r_list;

		double f = cur_obstacle->getFront();
		p::list f_list;
		f_list.append(f);
		cur_obstacle_state["front"] = f_list;

		obstacle_state.append(cur_obstacle_state);
	}

	Wall* cur_wall;
	for(int i=0; i<walls.size(); i++)
	{
		cur_wall = walls.at(i);

		p::dict cur_wall_state;

		double* p = cur_wall->getP();
		p::list p_list;
		p_list.append(p[0]);
		p_list.append(p[1]);
		cur_wall_state["p"] = p_list;

		double w = cur_wall->getW();
		p::list w_list;
		w_list.append(w);
		cur_wall_state["w"] = w_list;

		double h = cur_wall->getH();
		p::list h_list;
		h_list.append(h);
		cur_wall_state["h"] = h_list;

		wall_state.append(cur_wall_state);
	}

	obs["agent"] = agent_state;
	obs["obstacle"] = obstacle_state;
	obs["wall"] = wall_state;

	return obs;
}

BOOST_PYTHON_MODULE(csim)
{
	p::class_<Parser>("Parser", p::init<std::string, int, int>())
		.def("Reset", &Parser::Reset)
		.def("Step", &Parser::Step)
		.def("Observe", &Parser::Observe);
}


