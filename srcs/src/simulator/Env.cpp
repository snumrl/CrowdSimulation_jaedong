#include <iostream>
#include <math.h>
#include <omp.h>
#include <ctime>
#include "mMath.h"
#include "Env.h"

using namespace std;

#define PI 3.141592

#define w_target -20.0
#define w_col -80.0
#define w_smooth_a -0.5
#define w_smooth_w -0.5
#define w_pref_v -2.0

#define w_bubble -0.0

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
		Agent* cur_agent;
		cur_agent = _agents.at(i);
		for(int j=0; j<agent_num; j++)
		{
			if(i==j)
				continue;

			Agent* other_agent;
			other_agent = _agents.at(j);

			double other_data[9];
			other_agent->getData(other_data);
			cur_agent->setVision(other_data);
		}

		for(int j=0; j<obstacle_num; j++)
		{
			Obstacle* obs;
			obs = _obstacles.at(j);

			double obs_data[5];
			obs->getData(obs_data);
			cur_agent->setVision(obs_data);
		}

		// for(int j=0; j<wall_num; j++)
		// {
		// 	Wall* wall;
		// 	wall = _walls.at(j);

		// 	cur_agent->setVisionWall(wall);
		// }
	}

	return _agents;
}

void Env::Update()
{
	srand((unsigned int)time(0));

	double prev_score = getTargetScore();
	double smooth_score = getSmoothScore(_agents.at(0));
	double prefV_score = getPrefVScore(_agents.at(0));
	// double bubble_score = getBubbleScore(_agents.at(0));

	Agent* agent_;
	for(int i=0; i<agent_num; i++){
		agent_ = _agents.at(i);
		agent_->Action();
		agent_->visionReset();
		// if(Dist(agent_->getP(), agent_->getD()) < 4.0)
		// 	agent_->setD( -30 + rand()%60, -20 + rand()%40);
	}

	double next_score = getTargetScore();
	double target_score = next_score - prev_score;

	for(int i=0; i<agent_num; i++)
	{
		Agent* cur_agent;
		cur_agent = _agents.at(i);
		cur_agent->setCol(false);

		bool isCol = false;
		#pragma omp parallel for
		for(int j=i+1; j<agent_num; j++)
		{
			Agent* other_agent;
			other_agent = _agents.at(j);
			double other_data[9];
			other_agent->getData(other_data);

			if(cur_agent->colCheck(other_data)){
				isCol = true;
				cur_agent->Revert(other_agent->getP(), true);
				other_agent->Revert(cur_agent->getP(), true);
			}
		}

		#pragma omp parallel for
		for(int j=0; j<obstacle_num; j++)
		{
			Obstacle* cur_obs;
			cur_obs = _obstacles.at(j);
			double obs_data[9];
			cur_obs->getData(obs_data);

			if(cur_agent->colCheck(obs_data)){
				isCol = true;
				cur_agent->Revert(cur_obs->getP(), true);
			}
		}

		int wall_num = _walls.size();
		#pragma omp parallel for
		for(int j=0; j<wall_num; j++)
		{
			Wall* cur_wall;
			cur_wall = _walls.at(j);

			if(LineSphereIntersection(cur_wall->getU(), cur_wall->getP(), cur_wall->getL(), cur_agent->getP(), (cur_agent->getR())[0]))
			{
				isCol = true;
				cur_agent->Revert(cur_wall->getSt(), false);
			}
		}

		if(isCol)
			cur_agent->setCol(true);
	}

	double col_score = 0;
	if(_agents.at(0)->getCol())
		col_score = w_col;

	_reward_sep[0] = target_score;
	_reward_sep[1] = prefV_score;
	_reward_sep[2] = smooth_score;
	_reward_sep[3] = col_score;

	double score = target_score + prefV_score + smooth_score + col_score;

	_reward = score;
}

void Env::setAction(int i, double w, double a_x, double a_y)
{
	_agents.at(i)->setAction(w, a_x, a_y);
	if(i==0)
		_cur_step += 1;
}

bool Env::isTerm(bool isTest)
{
	if(!isTest && _agents.at(0)->getCol())
		return true;

	if(Dist(_agents.at(0)->getD(), _agents.at(0)->getP()) < 2.0)
		return true;

	if(_cur_step > _max_step)
		return true;

	return false;
}

bool Env::isCol()
{
	if(_agents.at(0)->getCol())
		return true;
}

double Env::getReward()
{
	return _reward;
}

double* Env::getRewardSep()
{
	return _reward_sep;
}

double Env::getTargetScore()
{
	double d = Dist(_agents.at(0)->getD(), _agents.at(0)->getP());
	// double d_square = d*d;

	return w_target * d;
}

double Env::getSmoothScore(Agent* agent_)
{
	double* cur_a = agent_->getA();
	double cur_w = agent_->getW();

	double smooth_a = 0.0;
	double smooth_w = 0.0;

	double prefA[2];
	prefA[0] = 0.0;
	prefA[1] = 0.0;

	double a_dist = Dist(cur_a, prefA);
	double a_boundary = 0.2;
	if(a_dist > a_boundary)
		smooth_a = w_smooth_a * (a_dist - a_boundary) * (a_dist - a_boundary);

	double w_boundary = 0.4;
	if(cur_w > w_boundary) // -0.15 ~ 0.15
		smooth_w = w_smooth_w*(cur_w - w_boundary)*(cur_w - w_boundary);
	if(cur_w < -w_boundary)
		smooth_w = w_smooth_w*(cur_w + w_boundary)*(cur_w + w_boundary);

	return smooth_a + smooth_w;
}

double Env::getPrefVScore(Agent* agent_)
{
	double* cur_v = agent_->getV();
	double pref_v = 0.0;

	double prefV1[2];
	prefV1[0] = 0.0;
	prefV1[1] = 1.0;

	double prefV2[2];
	prefV2[0] = -0.5;
	prefV2[1] = 0.0;

	double prefV3[2];
	prefV3[0] = 0.5;
	prefV3[1] = 0.0;

	double prefV4[2];
	prefV4[0] = 0.0;
	prefV4[1] = 0.0;

	double dist1 = Dist(cur_v, prefV1);
	double dist2 = Dist(cur_v, prefV2);
	double dist3 = Dist(cur_v, prefV3);
	double dist4 = Dist(cur_v, prefV4);

	double minDist = dist1;
	if(dist2 < minDist)
		minDist = dist2;
	if(dist3 < minDist)
		minDist = dist3;
	if(dist4 < minDist)
		minDist = dist4;

	// std::cout << "v : " << cur_v[0] << ", " << cur_v[1] << std::endl;

	pref_v = w_pref_v * (minDist) * (minDist);

	return pref_v;
}

double Env::getBubbleScore(Agent* agent_)
{
	double* cur_vision = agent_->getVision();
	double vision_depth = agent_->getVisionDepth();
	int vision_ray = agent_->getVisionRayNum();
	double mean = 0.0;
	int idx = 0;

	double min_ray = vision_depth;
	for(int i=0; i<vision_ray; i++){
		if(cur_vision[i] < min_ray)
			min_ray = cur_vision[i];
	}

	double bScore = (double)(vision_depth - min_ray)/vision_depth;

	// double cov_ = Cov(cur_vision, 45);
	// double dist = (double)(min_ray)/(double)vision_depth;
	// double d_of_cov = dist/cov_;

	return w_bubble*bScore*bScore;
}

void Env::addAgent(Agent* agent)
{
	_agents.push_back(agent);
}

void Env::addObstacle(Obstacle* obstacle)
{
	_obstacles.push_back(obstacle);
}

void Env::addWall(Wall* wall)
{
	_walls.push_back(wall);
}

const vector<Agent*>& Env::getAgents()
{
	return _agents;
}

const vector<Obstacle*> & Env::getObstacles()
{
	return _obstacles;
}

const vector<Wall*> & Env::getWalls()
{
	return _walls;
}

Agent* Env::getAgent(int id)
{
	return _agents[id];
}

Obstacle* Env::getObstacle(int id)
{
	return _obstacles[id];
}

Wall* Env::getWall(int id)
{
	return _walls[id];
}

int Env::getNumAgents() const
{
	return _agents.size();
}

int Env::getNumObstacles() const
{
	return _obstacles.size();
}

int Env::getNumWalls() const
{
	return _walls.size();
}
