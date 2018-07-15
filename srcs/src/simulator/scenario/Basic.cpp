#include <iostream>
#include <ctime>
#include "../mMath.h"
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
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();
}

void Basic::initEvaluation()
{
	srand((unsigned int)time(0));

	int eval_set_num = 4;
	int rand_x, rand_y;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num; j++)
		{
			rand_x = rand()%15;
			rand_y = rand()%40;

			if(j<agent_num/2){
				eval_agent_p_x.push_back(-30 + rand_x);
				eval_agent_p_y.push_back(-20 + rand_y);
				eval_agent_d_x.push_back( 0 + rand()%15);
				eval_agent_d_y.push_back(-20 + rand()%40);
			}
			else{
				eval_agent_p_x.push_back(15 + rand_x);
				eval_agent_p_y.push_back(-20 + rand_y);
				eval_agent_d_x.push_back(-15 + rand()%15);
				eval_agent_d_y.push_back(-20 + rand()%40);
			}
		}

		for(int j=0; j<obstacle_num; j++)
		{
			rand_x = rand()%30;
			rand_y = rand()%30;

			eval_obs_p_x.push_back(-15 + rand_x);
			eval_obs_p_y.push_back(-15 + rand_y);
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
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
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
			agent->setFront(3.14);
		}

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

	int rand_x, rand_y;
	double r_j;
	double pos[2];
	bool col = false;
	Agent* agent;
	for(int i=0; i<agent_num; i++)
	{
		while(true)
		{
			rand_x = rand()%60;
			rand_y = rand()%40;

			pos[0] = -30 + rand_x;
			pos[1] = -20 + rand_y;

			col = false;
			int start_idx = 0;
			if(i > agent_num/2)
				start_idx = agent_num/2;

			for(int j=start_idx; j<i; j++)
			{
				r_j = getAgent(j)->getR();
				if(Dist(pos, getAgent(j)->getP()) < r_j * 2 + 2.0)
				{
					col = true;
					break;
				}
			}

			if(col == false)
				break;
		}

		agent = new Agent();
		agent->setP(pos[0], pos[1]);
		agent->setPprev(pos[0], pos[1]);
		agent->setD( -30 + rand()%60, -20 + rand()%40);
		agent->setQ(1.0, 0.0);
		agent->setFront(0.0);
		agent->setColor(0.9, 0.1, 0.1);

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

	for(int i=0; i<obstacle_num; i++)
	{
		int rand_x;
		int rand_y;
		rand_x = rand()%30;
		rand_y = rand()%30;
		Obstacle* obs = new Obstacle(); // p q d
		obs->setP(-15.0 + rand_x, -15.0 + rand_y);

		addObstacle(obs);
	}

	_cur_step = 0;
}

void Basic::Render()
{
	for(int i=0; i<agent_num; i++)
		getAgent(i)->Render();

	for(int i=0; i<obstacle_num; i++)
		getObstacle(i)->Render();
}



