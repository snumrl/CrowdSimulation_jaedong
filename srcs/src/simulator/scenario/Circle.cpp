#include <iostream>
#include <ctime>
#include <cmath>
#include "../mMath.h"
#include "Circle.h"

using namespace std;

#define PI 3.14159265
#define Radius 300.0

Circle::Circle(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();

	Reset(-1);
}

Circle::~Circle()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
	{
		delete (*it);
	}
	_agents.clear();
}

void Circle::initEvaluation()
{
	int eval_set_num = 4;

	srand((unsigned int)time(0));

	double angle = 360.0/(double)agent_num;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num; j++)
		{
			eval_agent_p_x.push_back(Radius*cos( (i+j) * angle * PI / 180.0));
			eval_agent_p_y.push_back(Radius*sin( (i+j) * angle * PI / 180.0));
			eval_agent_d_x.push_back(Radius*cos( ((i+j) * angle+180.0) * PI / 180.0));
			eval_agent_d_y.push_back(Radius*sin( ((i+j) * angle+180.0) * PI / 180.0));
		}
	}
}

void Circle::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Circle::ResetEval(int idx)
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
	{
		delete (*it);
	}
	_agents.clear();

	for(int i=0; i<agent_num; i++)
	{
		Agent* agent = new Agent(); // p q d
		agent->setP(eval_agent_p_x.at(idx*agent_num + i), eval_agent_p_y.at(idx*agent_num + i));
		agent->setPprev(eval_agent_p_x.at(idx*agent_num + i), eval_agent_p_y.at(idx*agent_num + i));
		agent->setD(eval_agent_d_x.at(idx*agent_num + i), eval_agent_d_y.at(idx*agent_num + i));

		double dir[2];
		vec_sub_vec(agent->getD(), agent->getP(), dir);
		double dir_len;
		vec_norm(dir);
		vec_divide_scalar(dir, dir_len, dir);

		agent->setQ(dir[0], dir[1]);
		agent->setFront(CoorToAngle(dir));

		double* dmap = new double[20];
		for(int j=0; j<20; j++){
			dmap[j] = _vision_depth;
		}

		double* vmap = new double[40];
		for(int j=0; j<40; j++){
			vmap[j] = 0.0;
		}

		agent->setDmap(dmap);
		agent->setVmap(vmap);

		addAgent(agent);
	}

	_cur_step = 0;
}

void Circle::ResetEnv()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);

	_agents.clear();

	srand((unsigned int)time(0));

	double angle = 360.0/(double)agent_num;
	int rand_idx = rand() % agent_num;
	Agent* agent;
	for(int i=rand_idx; i<rand_idx + agent_num; i++)
	{
		agent = new Agent(); // p q d
		agent->setP(Radius*cos( i * angle * PI / 180.0), Radius*sin( i * angle * PI / 180.0));
		agent->setPprev(Radius*cos( i * angle * PI / 180.0), Radius*sin( i * angle * PI / 180.0));
		agent->setD(Radius*cos((i * angle+180.0) * PI / 180.0), Radius*sin((i * angle+180.0) * PI / 180.0));

		double dir[2];
		vec_sub_vec(agent->getD(), agent->getP(), dir);
		double dir_len = vec_norm(dir);
		double dir_[2];
		vec_divide_scalar(dir, dir_len, dir);

		agent->setQ(dir[0], dir[1]);
		agent->setFront(CoorToAngle(dir));
		agent->setColor(0.8, 0.2, 0.2);

		double* dmap = new double[20];
		for(int j=0; j<20; j++){
			dmap[j] = _vision_depth;
		}

		double* vmap = new double[40];
		for(int j=0; j<40; j++){
			vmap[j] = 0.0;
		}

		agent->setDmap(dmap);
		agent->setVmap(vmap);

		addAgent(agent);
	}

	_cur_step = 0;
}

void Circle::Render()
{
	for(int i=0; i<agent_num; i++)
	{
		getAgent(i)->Render();
	}
}



