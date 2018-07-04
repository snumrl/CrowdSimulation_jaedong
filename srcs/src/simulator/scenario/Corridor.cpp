#include <iostream>
#include <ctime>
#include "../mMath.h"
#include "Corridor.h"

using namespace std;

Corridor::Corridor(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();

	// double st1[2] = {-600, 65};
	// double p1[2] = {1.0, 0.0};
	// double n1[2] = {0.0, -1.0};
	// double l1 = 1200;
	// Wall* w1 = new Wall(st1, p1, l1, n1);

	// double st2[2] = {-600, -65};
	// double p2[2] = {1.0, 0.0};
	// double n2[2] = {0.0, 1.0};
	// double l2 = 1200;
	// Wall* w2 = new Wall(st2, p2, l2, n2);

	double st1[2] = {-30, 8};
	double p1[2] = {1.0, 0.0};
	double n1[2] = {0.0, -1.0};
	double l1 = 60;
	Wall* w1 = new Wall(st1, p1, l1, n1);

	double st2[2] = {-30, -8};
	double p2[2] = {1.0, 0.0};
	double n2[2] = {0.0, 1.0};
	double l2 = 60;
	Wall* w2 = new Wall(st2, p2, l2, n2);

	addWall(w1);
	addWall(w2);

	Reset(-1);
}

Corridor::~Corridor()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();
}

void Corridor::initEvaluation()
{
	srand((unsigned int)time(0));

	int eval_set_num = 4;
	int rand_x, rand_y;
	double pos[2];
	bool col = false;
	for(int i=0; i<eval_set_num; i++)
	{
		for(int j=0; j<agent_num; j++)
		{
			while(true)
			{
				rand_x = rand()%3;
				rand_y = rand()%14;

				if(i < agent_num/2)
				{
					pos[0] = -15 + rand_x;
					pos[1] = -7 + rand_y;
				}
				else
				{
					pos[0] = 12 + rand_x;
					pos[1] = -7 + rand_y;
				}


				col = false;
				int start_idx = 0;
				if(j > agent_num/2)
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
				eval_agent_d_x.push_back(15);
				eval_agent_d_y.push_back(pos[1]);
			}
			else
			{
				eval_agent_d_x.push_back(-15);
				eval_agent_d_y.push_back(pos[1]);
			}
		}

		for(int j=0; j<obstacle_num; j++)
		{
			rand_x = rand()%40;
			rand_y = rand()%20;

			eval_obs_p_x.push_back(-20 + rand_x);
			eval_obs_p_y.push_back(-10 + rand_y);
		}
	}
}

void Corridor::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Corridor::ResetEval(int idx)
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

		// if(i < agent_num)
		if(i < agent_num/2)
		{
			agent->setQ(1.0, 0.0);
			agent->setFront(0.0);
		}
		else
		{
			agent->setQ(-1.0, 0.0);
			// agent->setFront(180.0);
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

void Corridor::ResetEnv()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);

	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);

	_obstacles.clear();

	srand((unsigned int)time(0));

	for(int i=0; i<obstacle_num; i++)
	{
		int rand_x, rand_y;
		rand_x = rand()%40;
		rand_y = rand()%14;
		Obstacle* obs = new Obstacle(); // p q d
		obs->setP(-20.0 + rand_x, -7.0 + rand_y);

		addObstacle(obs);
	}

	int rand_x, rand_y;
	double r_j;
	int rand_idx  = rand() % 5;
	double pos[2];
	bool col = false;
	Agent* agent;
	for(int i=0; i<agent_num; i++)
	{
		while(true)
		{
			rand_x = rand()%3;
			rand_y = rand()%14;

			if(i < agent_num/2)
			{
				pos[0] = -15 + rand_x;
				pos[1] = -7 + rand_y;
			}
			else
			{
				pos[0] = 12 + rand_x;
				pos[1] = -7 + rand_y;
			}

			col = false;
			int start_idx = 0;
			if(i > agent_num/2)
				start_idx = agent_num/2;

			for(int j=start_idx; j<i; j++)
			{
				r_j = getAgent(j)->getR();
				if(Dist(pos, getAgent(j)->getP()) < r_j * 2 + 0.1)
				{
					col = true;
					break;
				}
			}

			// for(int j=0; j<obstacle_num; j++)
			// {
			// 	if(Dist(pos, getObstacle(j)->getP()) < r_j * 2 + 0.1)
			// 	{
			// 		col = true;
			// 		break;
			// 	}
			// }

			if(col == false)
				break;
		}

		agent = new Agent(); // p q d
		agent->setP(pos[0], pos[1]);
		agent->setPprev(pos[0], pos[1]);
		if(i < agent_num/2)
		{
			agent->setD(25.0, -7 + rand()%14);
			// agent->setD(400.0, pos[1]);
			agent->setQ(1.0, 0.0);
			agent->setFront(0.0);
			agent->setColor(0.8, 0.2, 0.2);
		}
		else
		{
			agent->setD(-25.0, -7 + rand()%14);
			// agent->setD(-400.0, pos[1]);
			agent->setQ(-1.0, 0.0);
			// agent->setFront(180.0);
			agent->setFront(3.14);
			agent->setColor(0.2, 0.8, 0.2);
		}

		// if(i < agent_num/2)
		// {
		// 	pos[0] = -100;
		// 	pos[1] = 50 - 25*((i+rand_idx)%5);
		// 	agent = new Agent(); // p q d
		// 	agent->setP(pos[0], pos[1]);
		// 	agent->setPprev(pos[0], pos[1]);
		// 	agent->setD(400.0 , pos[1]);
		// 	agent->setQ(1.0, 0.0);
		// 	agent->setFront(0.0);
		// 	agent->setColor(0.8, 0.2, 0.2);
		// 	if(i==0)
		// 		agent->setD(250.0 , pos[1]);

		// }
		// else
		// {
		// 	pos[0] = 100;
		// 	pos[1] = 50 - 25*(i-5);
		// 	agent = new Agent(); // p q d
		// 	agent->setP(pos[0], pos[1]);
		// 	agent->setPprev(pos[0], pos[1]);
		// 	agent->setD(-400.0, pos[1]);
		// 	agent->setQ(-1.0, 0.0);
		// 	agent->setFront(180.0);
		// 	agent->setColor(0.2, 0.8, 0.2);
		// }

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

void Corridor::Render()
{
	for(int i=0; i<agent_num; i++)
		getAgent(i)->Render();

	for(int i=0; i<obstacle_num; i++)
		getObstacle(i)->Render();
}



