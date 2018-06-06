#include <iostream>
#include <ctime>
#include "mMath.h"
#include "Basic.h"

using namespace std;

Basic::Basic(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();

	Reset(-1);
}

Basic::~Basic()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
	{
		delete (*it);
	}
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
	{
		delete (*it);
	}
	_obstacles.clear();
}

void Basic::initEvaluation()
{
	int eval_set_num = 6;

	srand((unsigned int)time(0));

	int rand_x;
	int rand_y;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num/2; j++)
		{
			rand_x = rand()%100;
			rand_y = rand()%200;

			eval_agent_p_x.push_back(-300 + rand_x);
			eval_agent_p_y.push_back(-100 + rand_y);
			eval_agent_d_x.push_back(250);
			eval_agent_d_y.push_back(-100 + rand_y);
		}

		for(int j=agent_num/2; j<agent_num; j++)
		{
			rand_x = rand()%100;
			rand_y = rand()%200;

			eval_agent_p_x.push_back(200 + rand_x);
			eval_agent_p_y.push_back(-100 + rand_y);
			eval_agent_d_x.push_back(-250);
			eval_agent_d_y.push_back(-100 + rand_y);
		}

		for(int j=0; j<obstacle_num; j++)
		{
			rand_x = rand()%300;
			rand_y = rand()%300;

			eval_obs_p_x.push_back(-150 + rand_x);
			eval_obs_p_y.push_back(-150 + rand_y);
		}
	}
}

void Basic::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Basic::ResetEval(int idx)
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
	{
		delete (*it);
	}
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
	{
		delete (*it);
	}
	_obstacles.clear();

	for(int i=0; i<agent_num; i++)
	{
		Agent* agent = new Agent(); // p q d
		agent->setP(eval_agent_p_x.at(idx*agent_num + i), eval_agent_p_y.at(idx*agent_num + i));
		agent->setPprev(eval_agent_p_x.at(idx*agent_num + i), eval_agent_p_y.at(idx*agent_num + i));
		agent->setD(eval_agent_d_x.at(idx*agent_num + i), eval_agent_d_y.at(idx*agent_num + i));

		if(i < agent_num/2)
		{
			agent->setQ(1.0, 0.0);
			agent->setFront(0.0);
		}
		else
		{
			agent->setQ(-1.0, 0.0);
			agent->setFront(180.0);
		}

		double* dmap = new double[20];
		for(int j=0; j<20; j++){
			dmap[j] = _vision_depth;
		}

		double* delta = new double[20];
		for(int j=0; j<20; j++){
			delta[j] = 0.0;
		}

		agent->setDmap(dmap);
		agent->setDelta(delta);

		addAgent(agent);
	}

	for(int i=0; i<obstacle_num; i++)
	{
		Obstacle* obs = new Obstacle(); // p q d
		obs->setP(eval_obs_p_x.at(idx*obstacle_num + i), eval_obs_p_y.at(idx*obstacle_num + i));

		addObstacle(obs);
	}

	_cur_step = 0;
}

void Basic::ResetEnv()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);

	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);

	_obstacles.clear();

	srand((unsigned int)time(0));

	int rand_x;
	int rand_y;
	Agent* agent;
	for(int i=0; i<agent_num; i++)
	{
		rand_x = rand()%100;
		rand_y = rand()%200;

		if(i < agent_num/2)
		{
			agent = new Agent(); // p q d
			agent->setP(-300.0 + rand_x, -100.0 + rand_y);
			agent->setPprev(-300.0 + rand_x, -100.0 + rand_y);
			agent->setD(250.0, -200.0 + rand_y);
			agent->setQ(1.0, 0.0);
			agent->setFront(0.0);
			agent->setColor(0.8, 0.2, 0.2);
		}
		else
		{
			agent = new Agent(); // p q d
			agent->setP(200.0 + rand_x, -100.0 + rand_y);
			agent->setPprev(200.0 + rand_x, -100.0 + rand_y);
			agent->setD(-250.0, -200.0 + rand_y);
			agent->setQ(-1.0, 0.0);
			agent->setFront(180.0);
			agent->setColor(0.2, 0.8, 0.2);
		}


		double* dmap = new double[20];
		for(int j=0; j<20; j++){
			dmap[j] = _vision_depth;
		}

		double* delta = new double[20];
		for(int j=0; j<20; j++){
			delta[j] = 0.0;
		}

		agent->setDmap(dmap);
		agent->setDelta(delta);

		addAgent(agent);
	}

	for(int i=0; i<obstacle_num; i++)
	{
		int rand_x;
		int rand_y;
		rand_x = rand()%300;
		rand_y = rand()%300;
		Obstacle* obs = new Obstacle(); // p q d
		obs->setP(-150.0 + rand_x, -150.0 + rand_y);

		addObstacle(obs);
	}

	_cur_step = 0;
}

void Basic::Render()
{
	for(int i=0; i<agent_num; i++)
	{
		getAgent(i)->Render();
	}

	for(int i=0; i<obstacle_num; i++)
	{
		getObstacle(i)->Render();
	}
}



