#include <iostream>
#include <ctime>
#include "../mMath.h"
#include "Bottleneck.h"

using namespace std;

Bottleneck::Bottleneck(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();

	double st1[2] = {-600, 250};
	double p1[2] = {1.0, 0.0};
	double l1 = 600;
	Wall* w1 = new Wall(st1, p1, l1);

	double st2[2] = {0, 250};
	double p2[2] = {1.0, 0.0};
	double l2 = 600;
	Wall* w2 = new Wall(st2, p2, l2);

	double st3[2] = {-600, -250};
	double p3[2] = {1.0, 0.0};
	double l3 = 600;
	Wall* w3 = new Wall(st3, p3, l3);

	double st4[2] = {0, -250};
	double p4[2] = {1.0, 0.0};
	double l4 = 600;
	Wall* w4 = new Wall(st4, p4, l4);

	addWall(w1);
	addWall(w2);
	addWall(w3);
	addWall(w4);

	Reset(-1);
}

Bottleneck::~Bottleneck()
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
}

void Bottleneck::initEvaluation()
{
	int eval_set_num = 6;

	srand((unsigned int)time(0));

	int rand_x;
	int rand_y;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num; j++)
		{
			rand_x = rand()%100;
			rand_y = rand()%200;

			eval_agent_p_x.push_back(-300 + rand_x);
			eval_agent_p_y.push_back(-100 + rand_y);
			eval_agent_d_x.push_back(250);
			eval_agent_d_y.push_back(-100 + rand_y);
		}
	}
}

void Bottleneck::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Bottleneck::ResetEval(int idx)
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
		agent->setQ(1.0, 0.0);
		agent->setFront(0.0);

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

void Bottleneck::ResetEnv()
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

			pos[0] = -300 + rand_x;
			pos[1] = -100 + rand_y;

			col = false;
			int start_idx = 0;
			for(int j=0; j<i; j++)
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
		agent->setD(300.0, -100 + rand()%200);
		agent->setQ(1.0, 0.0);
		agent->setFront(0.0);
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

void Bottleneck::Render()
{
	for(int i=0; i<agent_num; i++)
	{
		getAgent(i)->Render();
	}
}



