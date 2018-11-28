#include <iostream>
#include <ctime>
#include <omp.h>
#include "../mMath.h"
#include "Hallway.h"

using namespace std;

Hallway::Hallway(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();
	initWalls();

	Reset(-1);
}

void Hallway::initWalls()
{
	wall_num = 2;

	double p1[2] = {0.0, 11.0};
	double w1 = 48.0;
	double h1 = 10.0;
	addWall(new Wall(p1, w1, h1));

	double p2[2] = {0.0, -11.0};
	double w2 = 48.0;
	double h2= 10.0;
	addWall(new Wall(p2, w2, h2));

	wall_num = _walls.size();
}

Hallway::~Hallway()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();
}

void Hallway::initEvaluation()
{
	srand((unsigned int)time(0));
}

void Hallway::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Hallway::ResetEval(int idx)
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();
}

void Hallway::ResetEnv()
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
		double obs_pos[2];
		obs_pos[0] = -10.0 + rand()%20;
		obs_pos[1] = -5.0 + rand()%10;

		double obs_r[2];
		obs_r[0] = (10 + rand()%30)/20.0;
		obs_r[1] = (10 + rand()%30)/20.0;

		double tmp;
		if(obs_r[0] < obs_r[1]){
			tmp = obs_r[0];
			obs_r[0] = obs_r[1];
			obs_r[1] = tmp;
		}

		Obstacle* obs = new Obstacle(obs_r, obs_pos); // p q d
		addObstacle(obs);
	}

	for(int i=0; i<agent_num; i++)
	{
		double agent_r[2];
		agent_r[0] = (3 + rand()%7)/10.0;
		agent_r[1] = (3 + rand()%7)/10.0;

		double tmp;
		if(agent_r[0] < agent_r[1]){
			tmp = agent_r[0];
			agent_r[0] = agent_r[1];
			agent_r[1] = tmp;
		}

		Agent* agent = new Agent(agent_r);
		agent->setId(i);

		bool col = false;
		double agent_pos[2];
		while(true)
		{
			if(i%2==0){
				agent_pos[0] = 6.0 + rand()%8;
				agent_pos[1] = -4.0 + rand()%8;
			}
			else{
				agent_pos[0] = -14.0 + rand()%8;
				agent_pos[1] = -4.0 + rand()%8;
			}

			col = false;
			int start_idx = 0;
			for(int j=start_idx; j<i; j++)
			{
				double boundary = (getAgent(j)->getR())[0] + agent_r[0] + 0.1;
				if(Dist(agent_pos, getAgent(j)->getP()) < boundary)
				{
					col = true;
					break;
				}
			}

			if(col == false)
			{
				for(int j=0; j<obstacle_num; j++)
				{
					double boundary = getObstacle(j)->getR()[0] + agent_r[0];
					if(Dist(agent_pos, getObstacle(j)->getP()) < boundary)
					{
						col = true;
						break;
					}
				}
			}

			if(col == false)
				break;
		}

		agent->setP(agent_pos[0], agent_pos[1]);
		agent->setPprev(agent_pos[0], agent_pos[1]);

		bool d_col = false;
		double d_pos[2];
		while(true){
			d_col = false;
			if(i%2==0){
				d_pos[0] = -14 + rand()%4;
				d_pos[1] = -4 + rand()%8;
			}
			else{
				d_pos[0] = 10 + rand()%4;
				d_pos[1] = -4 + rand()%8;
			}

			for(int j=0; j<obstacle_num; j++)
			{
				double boundary = getObstacle(j)->getR()[0] + 0.5;
				if(Dist(d_pos, getObstacle(j)->getP()) < boundary)
				{
					d_col = true;
					break;
				}
			}

			if(Dist(d_pos, agent->getP()) < 3.0)
				d_col = true;

			if(!d_col)
				break;
		}

		agent->setD( d_pos[0], d_pos[1]);
		if(i%2==0){
			agent->setColor(0.1, 0.9, 0.1);
			agent->setQy(-1.0, 0.0);
			agent->setQx(0.0, 11.0);
			agent->setFront(3.141592);
		}
		else{
			agent->setColor(0.9, 0.1, 0.1);
			agent->setQy(1.0, 0.0);
			agent->setQx(0.0, -1.0);
			agent->setFront(0.0);
		}
		if(i==0)
			agent->setColor(0.9, 0.5, 0.1);

		addAgent(agent);
	}
	_cur_step = 0;
}




