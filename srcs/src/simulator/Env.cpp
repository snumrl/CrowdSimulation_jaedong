#include <iostream>
#include <math.h>
#include <omp.h>
#include <ctime>
#include "mMath.h"
#include "Env.h"

using namespace std;

#define PI 3.141592

#define w_target 4.0
#define w_col -4
#define w_smooth_v -0.01
#define w_smooth_w -0.01
#define w_pref_v 0.0
#define w_pref_w -0.0
#define w_pref_dir 0.01
#define w_bubble 0.0

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
		#pragma omp parallel for
		for(int j=0; j<agent_num; j++)
		{
			if(i==j)
				continue;

			Agent* other_agent;
			other_agent = _agents.at(j);

			double other_data[7];
			other_agent->getData(other_data);
			cur_agent->setVision(other_data, false);
		}

		#pragma omp parallel for
		for(int j=0; j<obstacle_num; j++)
		{
			Obstacle* obs;
			obs = _obstacles.at(j);

			double obs_data[7];
			obs->getData(obs_data);
			cur_agent->setVision(obs_data, false);
		}

		#pragma omp parallel for
		for(int j=0; j<wall_num; j++)
		{
			Wall* wall;
			wall = _walls.at(j);

			vector<Edge*> edges = wall->getEdges();
			for(int k=0; k<edges.size(); k++){
				double edge_data[4];
				edge_data[0] = edges.at(k)->getSt()[0];
				edge_data[1] = edges.at(k)->getSt()[1];
				edge_data[2] = edges.at(k)->getEd()[0];
				edge_data[3] = edges.at(k)->getEd()[1];

				cur_agent->setVision(edge_data, true);
			}
		}
	}

	return _agents;
}

void Env::Update()
{
	srand((unsigned int)time(0));

	double score = 0.0;

	double prev_dist = getTargetDist();


	Agent* agent_;
	for(int i=0; i<agent_num; i++){
		agent_ = _agents.at(i);
		agent_->Action();
	}

	double cur_dist = getTargetDist();
	double diff_dist = prev_dist - cur_dist;

	double target_score = getTargetScore(diff_dist);
	double prefV_score = getPrefScore(_agents.at(0));
	double dir_score = getDirScore(_agents.at(0));
	double smooth_score = getSmoothScore(_agents.at(0));
	double bubble_score = getBubbleScore(_agents.at(0));

	for(int i=0; i<agent_num; i++){
		agent_ = _agents.at(i);
		agent_->setVisionVel();
		agent_->visionReset();
		// if(Dist(agent_->getP(), agent_->getD()) < 4.0)
		// 	agent_->setD( -24 + rand()%48, -16 + rand()%32);
	}

	#pragma omp parallel for
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
			double other_data[7];
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
			double obs_data[7];
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

			double tmp[2];
			if(cur_agent->colCheckWall(cur_wall)){
				isCol = true;
				cur_agent->Revert(tmp, false);
			}
		}

		if(isCol){
			cur_agent->setCol(true);
			// std::cout << i << " : col" << std::endl;
		}
	}

	double col_score = 0.0;
	if(_agents.at(0)->getCol())
		col_score = w_col;

	score += target_score;
	score += prefV_score;
	score += smooth_score;
	score += bubble_score;
	score += col_score;
	score += dir_score;

	_reward = score;

	_reward_sep[0] = target_score;
	_reward_sep[1] = prefV_score;
	_reward_sep[2] = smooth_score;
	_reward_sep[3] = col_score;
	_reward_sep[4] = bubble_score;
	_reward_sep[5] = dir_score;
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

double Env::getTargetDist()
{
	Agent* agent_ = _agents.at(0);
	double dist = Dist(agent_->getP(), agent_->getD());
}

double Env::getTargetScore(double dist)
{
	// double dist_square = dist*dist;

	double score = 0.0;
	if(dist > 0.0){
		score = w_target * dist;
	}
	else{
		score = w_target * dist;
	}

	return score;
}

double Env::getSmoothScore(Agent* agent_)
{
	double* cur_v = agent_->getV();
	double* prev_v = agent_->getVprev();
	double cur_w = agent_->getW();
	double prev_w = agent_->getWprev();

	double cur_v_ = sqrt(cur_v[0]*cur_v[0] + cur_v[1]*cur_v[1]);
	// std::cout << "v : " << cur_v[0] << ", " << cur_v[1] << std::endl;
	// std::cout << "v : " << cur_v_ << std::endl;

	double smooth_v = 0.0;
	double smooth_w = 0.0;

	double v_dist = Dist(cur_v, prev_v);
	smooth_v = w_smooth_v*v_dist;

	double w_dist = fabs(cur_w - prev_w);
	smooth_w = w_smooth_w*w_dist;

	return smooth_v + smooth_w;
}

double Env::getPrefScore(Agent* agent_)
{
	double* cur_v = agent_->getV();
	double cur_w = agent_->getW();

	double pref_v = 0.0;
	double pref_w = 0.0;

	// double prefV1[2];
	// prefV1[0] = 0.0;
	// prefV1[1] = 1.0;

	// double prefV2[2];
	// prefV2[0] = -1.0;
	// prefV2[1] = 0.0;

	// double prefV3[2];
	// prefV3[0] = 1.0;
	// prefV3[1] = 0.0;

	// double prefV4[2];
	// prefV4[0] = 0.0;
	// prefV4[1] = 0.0;

	// double dist1 = Dist(cur_v, prefV1);
	// double dist2 = Dist(cur_v, prefV2);
	// double dist3 = Dist(cur_v, prefV3);
	// double dist4 = Dist(cur_v, prefV4);
	// double dist5 = fabs(cur_v[0]);
	// double dist6 = fabs(cur_v[1]);

	// if(cur_v[1] >= prefV1[1])
	// 	dist5 = dist1;
	// if(cur_v[1] < 0.0)
	// 	dist5 = 10.0;
	// if(cur_v[0] <= prefV2[0] && cur_v[1] < prefV1[1])
	// 	dist6 = dist2;
	// if(cur_v[0] >= prefV3[0] && cur_v[1] < prefV1[1])
	// 	dist6 = dist3;

	// double minDist = dist1;
	// if(dist2 < minDist)
	// 	minDist = dist2;
	// if(dist3 < minDist)
	// 	minDist = dist3;
	// if(dist4 < minDist)
	// 	minDist = dist4;
	// if(dist5 < minDist)
	// 	minDist = dist5;
	// if(dist6 < minDist)
	// 	minDist = dist6;

	// if(minDist > 0.3)
	// 	pref_v = w_pref_v*(minDist- 0.3);

	double origin[2] = {0.0, 0.0};
	double tmp_v_dist = Dist(origin, cur_v);
	if(tmp_v_dist > 1.0){
		pref_v = w_pref_v*(tmp_v_dist - 1.0);
	}

	double origin_ = 0.0;
	double tmp_w_dist = fabs(origin_ - cur_w);
	if(tmp_w_dist > 1.5){
		pref_w = w_pref_w*(tmp_w_dist - 1.5);
	}

	return pref_v + pref_w;
}

double Env::getDirScore(Agent* agent_)
{
	// double pref_dir = 0.0;

	double* cur_v = agent_->getV();

	double cur_v_len = sqrt(cur_v[0]*cur_v[0]+cur_v[1]*cur_v[1]);
	double cur_v_norm[2] = {cur_v[0]/cur_v_len, cur_v[1]/cur_v_len};
	double dir[2] = {0.0, 1.0};
	// pref_dir = w_pref_dir*(-1.0 * tanh(10.0*acos(Dot(cur_v_norm, dir))));

	// return pref_dir;
	double pref_dir = 0.0;

	double* cur_d = agent_->getD();
	double* cur_p = agent_->getP();
	double vec[2];

	vec_sub_vec(cur_d, cur_p, vec);
	double vec_len = sqrt(vec[0]*vec[0]+vec[1]*vec[1]);
	double vec_norm[2] = {vec[0]/vec_len, vec[1]/vec_len};

	double* cur_q = agent_->getQy();

	pref_dir = w_pref_dir*(-1.0 * acos(Dot(cur_q, vec_norm)));
	pref_dir += w_pref_dir*(-1.0 * acos(Dot(cur_v_norm, dir)));

	// std::cout << "q : " << acos(Dot(cur_q, vec_norm)) << std::endl;
	// std::cout << "v : " << acos(Dot(cur_v_norm, dir)) << std::endl;
	// std::cout << "dir : " << pref_dir << std::endl;
	// std::cout << std::endl;

	// std::cout << "dir : " << acos(Dot(cur_q, vec_norm)) << std::endl;
	// std::cout << "dir : " << pref_dir << std::endl;

	return pref_dir;
}

double Env::getBubbleScore(Agent* agent_)
{
	double bubble = 0.0;

	double* cur_vision = agent_->getVision();
	double vision_depth = agent_->getVisionDepth();
	int vision_ray = agent_->getVisionRayNum();
	double mean = 0.0;
	int idx = 0;

	double min_ray = vision_depth;
	for(int i=0; i<vision_ray; i++){
		if(cur_vision[i] < min_ray)
			min_ray = cur_vision[i];
		mean += cur_vision[i];
	}
	double depth_ = mean/(double)vision_ray;
	depth_ = vision_depth;
	bubble = w_bubble * (depth_ - min_ray)/depth_;

	// double cov_ = Cov(cur_vision, 45);
	// double dist = (double)(min_ray)/(double)vision_depth;
	// double d_of_cov = dist/cov_;

	// std::cout << "bubble : " << bubble <<  std::endl;

	return bubble;
}

double Env::getReward()
{
	return _reward;
}

double* Env::getRewardSep()
{
	return _reward_sep;
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
