#include <iostream>
#include <math.h>
#include <omp.h>
#include "mMath.h"
#include "Env.h"

using namespace std;

Env::Env()
{
	cout << "Env()" << endl;
}

Env::~Env()
{
	cout << "~Env()" << endl;
}

const vector<Agent*> & Env::Observe()
{
	#pragma omp parallel for
	for(int i=0; i<agent_num; i++)
	{
		double* d_map;
		double new_d_map[20];
		double* delta;
		Agent* cur_agent;

		cur_agent = _agents.at(i);

		double angle_v[2];
		for(int j=0; j<20; j++)
		{
			new_d_map[j] = _vision_depth;
			AngleToCoor(cur_agent->getFront() + cur_agent->getFov()/2 - j*cur_agent->getInterval(), angle_v);
			new_d_map[j] = depth_by_agents(angle_v, cur_agent, new_d_map[j], i);
			new_d_map[j] = depth_by_obstacles(angle_v, cur_agent, new_d_map[j]);
			// d_map[j] = depth_by_walls(angle, cur_agent, d_map[j]);
		}

		d_map = cur_agent->getDmap();
		delta = cur_agent->getDelta();
		for(int j=0; j<20; j++)
		{
			delta[j] = new_d_map[j] - d_map[j];
			d_map[j] = new_d_map[j];
		}
	}

	return _agents;
}

double Env::depth_by_obstacles(double* angle, Agent* agent, double cur_d)
{
	Obstacle* cur_obs;
	for(int i=0; i<obstacle_num; i++)
	{
		cur_obs = _obstacles.at(i);
		double d = Dist(agent->getP(), cur_obs->getP());
		double r = agent->getR() + cur_obs->getR() + 1.0;
		if(d - r > agent->getDepth())
			continue;

		double cur_dist = RayToSphereDistance(agent->getP(), cur_obs->getP(), angle, r);

		if(cur_dist > 0 && cur_dist < cur_d)
		{
			cur_d = cur_dist;
		}

	}

	return cur_d;
}

double Env::depth_by_agents(double* angle, Agent* agent, double cur_d, int idx)
{
	Agent* cur_agent;
	for(int i=0; i<agent_num; i++)
	{
		if(i == idx)
			continue;

		cur_agent = _agents.at(i);
		double d = Dist(agent->getP(), cur_agent->getP());
		double r = agent->getR() + cur_agent->getR() + 1.0;
		if(d - r > agent->getDepth())
			continue;

		double cur_dist = RayToSphereDistance(agent->getP(), cur_agent->getP(), angle, r);

		if(cur_dist >= 0 && cur_dist < cur_d)
		{
			cur_d = cur_dist;
		}

	}

	return cur_d;
}

void Env::Update()
{
	// collision check
	double prev_score = getScore();
	for(int i=0; i<agent_num; i++)
		_agents.at(i)->Action();

	_agents.at(0)->setCol(false);
	for(int i=0; i<agent_num; i++)
	{
		Agent* cur_agent;
		cur_agent = _agents.at(i);

		bool isCol = false;
		#pragma omp parallel for
		for(int j=0; j<agent_num; j++)
		{
			if(i!=j)
			{
				Agent* other_agent;
				other_agent = _agents.at(j);

				double d = Dist(cur_agent->getP(), other_agent->getP());
				double r = cur_agent->getR() + other_agent->getR() + 1.0;
				if(d < r)
				{
					isCol = true;
					cur_agent->Revert(other_agent->getP(), true);
					other_agent->Revert(cur_agent->getP(), true);
				}
			}
		}

		#pragma omp parallel for
		for(int j=0; j<obstacle_num; j++)
		{
			Obstacle* cur_obs;
			cur_obs = _obstacles.at(j);

			double d = Dist(cur_agent->getP(), cur_obs->getP());
			double r = cur_agent->getR() + cur_obs->getR() + 1.0;
			if(d < r)
			{
				isCol = true;
				cur_agent->Revert(cur_obs->getP(), true);
			}
		}

		if(isCol)
			cur_agent->setCol(true);
	}

	double next_score = getScore();
	double score = next_score - prev_score;

	if(_agents.at(0)->getCol())
		score = -1.0;

	_reward = score;

	_cur_step += 1;
}

void Env::setAction(int i, double t, double v, bool s)
{
	_agents.at(i)->setAction(t, v, s);
}

bool Env::isTerm(bool isTest)
{
	if(isTest)
		return false;

	if(Dist(_agents.at(0)->getD(), _agents.at(0)->getP()) < 10)
		return true;

	if(_cur_step > _max_step)
		return true;

	return false;
}

double Env::getReward()
{
	return _reward;
}

double Env::getScore()
{
	double d = Dist(_agents.at(0)->getD(), _agents.at(0)->getP());
	double d_square = sqrt(d);

	return -0.1*d;
}

void Env::addAgent(Agent* agent)
{
	_agents.push_back(agent);
}

void Env::addObstacle(Obstacle* obstacle)
{
	_obstacles.push_back(obstacle);
}

const vector<Agent*>& Env::getAgents()
{
	return _agents;
}

const vector<Obstacle*> & Env::getObstacles()
{
	return _obstacles;
}

Agent* Env::getAgent(int id)
{
	return _agents[id];
}

Obstacle* Env::getObstacle(int id)
{
	return _obstacles[id];
}

int Env::getNumAgents() const
{
	return _agents.size();
}

int Env::getNumObstacles() const
{
	return _obstacles.size();
}
