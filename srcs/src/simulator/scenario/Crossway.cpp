#include <iostream>
#include <ctime>
#include "../mMath.h"
#include "Crossway.h"

using namespace std;

Crossway::Crossway(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();

	double st1[2] = {-600, 250};
	double p1[2] = {1.0, 0.0};
	double l1 = 1200;
	Wall* w1 = new Wall(st1, p1, l1);

	double st2[2] = {-600, -250};
	double p2[2] = {1.0, 0.0};
	double l2 = 1200;
	Wall* w2 = new Wall(st2, p2, l2);

	addWall(w1);
	addWall(w2);

	Reset(-1);
}

Crossway::~Crossway()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
	{
		delete (*it);
	}
	_agents.clear();
}

void Crossway::initEvaluation()
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
	}
}

void Crossway::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Crossway::ResetEval(int idx)
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
	{
		delete (*it);
	}
	_agents.clear();

	for(vector< Wall* >::iterator it = _walls.begin() ; it != _walls.end(); it++)
	{
		delete (*it);
	}
	_walls.clear();

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

void Crossway::ResetEnv()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);

	_agents.clear();

	srand((unsigned int)time(0));

	int rand_x;
	int rand_y;
	double pos[2];
	bool col = false;
	Agent* agent;
	for(int i=0; i<agent_num; i++)
	{
		while(true)
		{
			rand_x = rand()%100;
			rand_y = rand()%200;

			if(i < agent_num/2)
			{
				pos[0] = -300 + rand_x;
				pos[1] = -100 + rand_y;
			}
			else
			{
				pos[0] = 200 + rand_x;
				pos[1] = -100 + rand_y;
			}

			col = false;
			int start_idx = 0;
			if(i > agent_num/2)
				start_idx = agent_num/2;

			for(int j=start_idx; j<i; j++)
			{
				if(Dist(pos, getAgent(j)->getP()) < 20)
				{
					col = true;
					break;
				}
			}

			if(col == false)
				break;
		}

		agent = new Agent(); // p q d
		agent->setP(pos[0], pos[1]);
		agent->setPprev(pos[0], pos[1]);
		if(i < agent_num/2)
		{
			agent->setD(300.0, -100 + rand()%200);
			agent->setQ(1.0, 0.0);
			agent->setFront(0.0);
			agent->setColor(0.8, 0.2, 0.2);
		}
		else
		{
			agent->setD(-300.0, -100 + rand()%200);
			agent->setQ(-1.0, 0.0);
			agent->setFront(180.0);
			agent->setColor(0.2, 0.8, 0.2);
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

	_cur_step = 0;
}

void Crossway::Render()
{
	for(int i=0; i<agent_num; i++)
	{
		getAgent(i)->Render();
	}
}



