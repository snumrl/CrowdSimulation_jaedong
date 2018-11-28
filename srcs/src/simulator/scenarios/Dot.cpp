#include <iostream>
#include <ctime>
#include <omp.h>
#include "../mMath.h"
#include "Dot.h"

using namespace std;

Dot::Dot(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();
	initWalls();

	Reset(-1);
}

Dot::~Dot()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();
}

void Dot::initWalls()
{
	wall_num = _walls.size();
}

void Dot::initEvaluation()
{
}

void Dot::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
}

void Dot::ResetEval(int idx)
{

}

void Dot::ResetEnv()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();

	srand((unsigned int)time(0));

	for(int i=0; i<9; i++){
		double obs_pos[2];
		obs_pos[0] = -8 + 8*(i%3);
		obs_pos[1] = 8 - 8*(i/3);

		double obs_r[2];
		obs_r[0] = (40 + rand()%30)/20.0;
		obs_r[1] = (40 + rand()%30)/20.0;

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

		int rand_p = rand()%16;
		double agent_p[2];
		agent_p[0] = -12 + 8*(rand_p%4);
		agent_p[1] =  12 - 8*(rand_p/4);

		Agent* agent = new Agent(agent_r, agent_p);
		agent->setId(i);
		agent->setPprev(agent_p[0], agent_p[1]);

		int rand_d;
		double agent_d[2];
		while(true){
			rand_d = rand()%16;
			if(rand_d != rand_p)
				break;
		}
		agent_d[0] = -12 + 8*(rand_d%4);
		agent_d[1] =  12 - 8*(rand_d/4);
		agent->setD( agent_d[0], agent_d[1]);

		double rand_front = (rand()%20 - 10)/10.0 * 3.141592;
		double qy[2];
		double qx[2];
		RadianToCoor(rand_front, qy);
		RadianToCoor(rand_front-3.141592/2.0, qx);
		agent->setQy(qy[0], qy[1]);
		agent->setQx(qx[0], qx[1]);
		agent->setFront(rand_front);
		agent->setColor(0.1, 0.9, 0.1);

		addAgent(agent);
	}
	_cur_step = 0;
}




