#include <iostream>
#include <ctime>
#include <omp.h>
#include "../mMath.h"
#include "Basic.h"

using namespace std;

Basic::Basic(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();
	initWalls();

	Reset(-1);
}

void Basic::initWalls()
{
	wall_num = _walls.size();
}

Basic::~Basic()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();

	for(vector< Wall* >::iterator it = _walls.begin() ; it != _walls.end(); it++)
		delete (*it);
	_walls.clear();
}

void Basic::initEvaluation()
{
	srand((unsigned int)time(0));


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

	// #pragma omp parallel for
	for(int i=0; i<obstacle_num; i++)
	{
		double obs_pos[2];
		obs_pos[0] = -10.0 + rand()%20;
		obs_pos[1] = -10.0 + rand()%20;

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
		agent_r[1] = (3 + rand()%2)/10.0;

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
			agent_pos[0] = -12.0 + rand()%24;
			agent_pos[1] = -12.0 + rand()%24;

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
			d_pos[0] = -10 + rand()%20;
			d_pos[1] = -10 + rand()%20;

			for(int j=0; j<obstacle_num; j++)
			{
				double boundary = getObstacle(j)->getR()[0] + 0.5;
				if(Dist(d_pos, getObstacle(j)->getP()) < boundary)
				{
					d_col = true;
					break;
				}
			}

			if(Dist(d_pos, agent->getP()) < 8.0)
				d_col = true;

			if(!d_col)
				break;
		}

		double cur_front = ((rand()%628)/100.0)-3.14;
		double y_coord[2];
		double x_coord[2];
		RadianToCoor(cur_front, y_coord);
		RadianToCoor(cur_front-0.5*3.141592, x_coord);

		agent->setFront(cur_front);
		agent->setQy(y_coord[0], y_coord[1]);
		agent->setQx(x_coord[0], x_coord[1]);
		agent->setD( d_pos[0], d_pos[1]);
		agent->setColor(0.1, 0.9, 0.1);

		// agent->setD( d_pos[0], d_pos[1]);
		// double dir[2] = { d_pos[0]-agent_pos[0], d_pos[1]-agent_pos[1]};
		// double cur_rad = CoorToRadian(dir);
		// agent->setQy(dir[0], dir[1]);
		// agent->setQx(0.0, -1.0);
		// agent->setFront(cur_rad);
		// agent->setColor(0.9, 0.1, 0.1);
		// if(i==0)
		// 	agent->setColor(0.1, 0.9, 0.1);

		addAgent(agent);
	}
	_cur_step = 0;
}




