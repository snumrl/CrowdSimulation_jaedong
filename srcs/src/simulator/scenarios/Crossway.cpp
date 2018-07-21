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
	initWalls();

	Reset(-1);
}

Crossway::~Crossway()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();
}

void Crossway::initWalls()
{
	double st1[2] = {-35, 15};
	double p1[2] = {1.0, 0.0};
	double n1[2] = {0.0, -1.0};
	double l1 = 20;
	addWall(new Wall(st1, p1, l1, n1));

	double st2[2] = {-35, -15};
	double p2[2] = {1.0, 0.0};
	double n2[2] = {0.0, 1.0};
	double l2 = 20;
	addWall(new Wall(st2, p2, l2, n2));

	double st3[2] = {15, 15};
	double p3[2] = {1.0, 0.0};
	double n3[2] = {0.0, -1.0};
	double l3 = 20;
	addWall(new Wall(st3, p3, l3, n3));

	double st4[2] = {15, -15};
	double p4[2] = {1.0, 0.0};
	double n4[2] = {0.0, 1.0};
	double l4 = 20;
	addWall(new Wall(st4, p4, l4, n4));

	double st5[2] = {-15, 15};
	double p5[2] = {0.0, 1.0};
	double n5[2] = {1.0, 0.0};
	double l5 = 20;
	addWall(new Wall(st5, p5, l5, n5));

	double st6[2] = {15, 15};
	double p6[2] = {0.0, 1.0};
	double n6[2] = {-1.0, 0.0};
	double l6 = 20;
	addWall(new Wall(st6, p6, l6, n6));

	double st7[2] = {-15, -15};
	double p7[2] = {0.0, -1.0};
	double n7[2] = {1.0, 0.0};
	double l7 = 20;
	addWall(new Wall(st7, p7, l7, n7));

	double st8[2] = {15, -15};
	double p8[2] = {0.0, -1.0};
	double n8[2] = {-1.0, 0.0};
	double l8 = 20;
	addWall(new Wall(st8, p8, l8, n8));

	wall_num = _walls.size();
}

void Crossway::initEvaluation()
{
	srand((unsigned int)time(0));

	int rand_x, rand_y;
	double pos[2];
	bool col = false;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num; j++)
		{
			while(true)
			{
				if(j < agent_num/2)
				{
					rand_x = rand()%5;
					rand_y = rand()%28;
					pos[0] = -30 + rand_x;
					pos[1] = -14 + rand_y;
				}
				else
				{
					rand_x = rand()%28;
					rand_y = rand()%5;
					pos[0] = -14 + rand_x;
					pos[1] = -30 + rand_y;
				}

				col = false;
				int start_idx = 0;
				if(i > agent_num/2)
					start_idx = agent_num/2;

				for(int k=start_idx; k<j; k++)
				{
					double p[2];
					p[0] = eval_agent_p_x.at(k);
					p[1] = eval_agent_p_y.at(k);
					if(Dist(pos, p) < 1.0)
					{
						col = true;
						break;
					}
				}

				if(col == false)
					break;
			}

			eval_agent_p_x.push_back(pos[0]);
			eval_agent_p_y.push_back(pos[1]);
			if(j < agent_num/2)
			{
				eval_agent_d_x.push_back(25);
				eval_agent_d_y.push_back(pos[1]);
			}
			else
			{
				eval_agent_d_x.push_back(pos[0]);
				eval_agent_d_y.push_back(25);
			}
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
		delete (*it);
	_agents.clear();

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
			agent->setQ(0.0, 1.0);
			agent->setFront(3.141592/2);
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

	int rand_x, rand_y;
	double r_j;
	double pos[2];
	bool col = false;
	Agent* agent;
	for(int i=0; i<agent_num; i++)
	{
		while(true)
		{
			rand_x = rand()%5;
			rand_y = rand()%28;

			if(i < agent_num/2)
			{
				pos[0] = -30 + rand_x;
				pos[1] = -14 + rand_y;
			}
			else
			{
				pos[0] = -14 + rand_y;
				pos[1] = -30 + rand_x;
			}

			col = false;
			int start_idx = 0;
			if(i > agent_num/2)
				start_idx = agent_num/2;

			for(int j=start_idx; j<i; j++)
			{
				r_j = getAgent(j)->getR();
				if(Dist(pos, getAgent(j)->getP()) < r_j * 2 + 0.8)
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
			if(i==0)
				agent->setD(23, pos[1]);
			else
				agent->setD(25.0, pos[1]);
			agent->setQ(1.0, 0.0);
			agent->setFront(0.0);
			agent->setColor(0.9, 0.1, 0.1);
		}
		else
		{
			agent->setD(pos[0], 25);
			agent->setQ(0.0, 1.0);
			agent->setFront(3.141592/2);
			agent->setColor(0.1, 0.9, 0.1);
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

