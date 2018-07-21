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
	initWalls();

	Reset(-1);
}

void Bottleneck::initWalls()
{
	double st1[2] = {-600, 200};
	double p1[2] = {1.0, 0.0};
	double n1[2] = {0.0, -1.0};
	double l1 = 600;
	Wall* w1 = new Wall(st1, p1, l1, n1);

	double st2[2] = {0, 80};
	double p2[2] = {0.0, 1.0};
	double n2[2] = {-1.0, 0.0};
	double l2 = 120;
	addWall(new Wall(st2, p2, l2, n2));

	double st3[2] = {-600, -200};
	double p3[2] = {1.0, 0.0};
	double n3[2] = {0.0, 1.0};
	double l3 = 600;
	addWall(new Wall(st3, p3, l3, n3));

	double st4[2] = {0, -80};
	double p4[2] = {0.0, -1.0};
	double n4[2] = {-1.0, 0.0};
	double l4 = 120;
	addWall(new Wall(st4, p4, l4, n4));

	double st5[2] = {0, 80};
	double p5[2] = {1.0, 0.0};
	double n5[2] = {0.0, -1.0};
	double l5 = 600;
	addWall(new Wall(st5, p5, l5, n5));

	double st6[2] = {0, -80};
	double p6[2] = {1.0, 0.0};
	double n6[2] = {0.0, 1.0};
	double l6 = 600;
	addWall(new Wall(st6, p6, l6, n6));

	wall_num = _walls.size();

}

Bottleneck::~Bottleneck()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Wall* >::iterator it = _walls.begin() ; it != _walls.end(); it++)
		delete (*it);
	_walls.clear();
}

void Bottleneck::initEvaluation()
{
	srand((unsigned int)time(0));

	int rand_x, rand_y;
	double pos[2];
	bool col =false;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num; j++)
		{
			while(true)
			{
				rand_x = rand()%60;
				rand_y = rand()%240;

				pos[0] = -300 + rand_x;
				pos[1] = -120 + rand_y;

				col = false;
				int start_idx = 0;
				for(int k=0; k<j; k++)
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
			eval_agent_d_x.push_back(250);
			eval_agent_d_y.push_back(-10 + rand()%20);
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
		delete (*it);
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

	int rand_x, rand_y;
	double r_j;
	double pos[2];
	bool col = false;
	Agent* agent;
	for(int i=0; i<agent_num; i++)
	{
		while(true)
		{
			col = false;

			rand_x = rand()%150;
			rand_y = rand()%340;

			pos[0] = -300 + rand_x;
			pos[1] = -170 + rand_y;

			if(i==0 && pos[1] > -80 && pos[1] < 80)
				col = true;

			int start_idx = 0;
			for(int j=0; j<i; j++)
			{
				r_j = getAgent(j)->getR();
				if(Dist(pos, getAgent(j)->getP()) < r_j * 2)
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
		if(i==0)
			agent->setD(300.0, -10 + rand()%20);
		else
			agent->setD(600.0, -10 + rand()%20);
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


