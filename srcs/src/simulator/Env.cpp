#include <iostream>
#include <math.h>
#include <omp.h>
#include "mMath.h"
#include "Env.h"

using namespace std;

#define PI 3.141592

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
		double* d_map = cur_agent->getDmap();
		double* v_map = cur_agent->getVmap();

		#pragma omp parallel for
		for(int j=0; j<20; j++)
		{
			double cur_radian;
			double angle_v[2];
			double tmp_map[3];

			tmp_map[0] = _vision_depth;
			tmp_map[1] = 0.0;
			tmp_map[2] = 0.0;
			// AngleToCoor(cur_agent->getFront() + cur_agent->getFov()/2 - j*cur_agent->getInterval(), angle_v);
			cur_radian = cur_agent->getFront() + (cur_agent->getFov()/2 - j*cur_agent->getInterval())/180.0*PI;
			RadianToCoor(cur_radian, angle_v);
			depth_by_agents(angle_v, cur_agent, tmp_map, i);
			depth_by_obstacles(angle_v, cur_agent, tmp_map);
			depth_by_walls(angle_v, cur_agent, tmp_map, i);

			d_map[j] = tmp_map[0];
			v_map[2*j] = tmp_map[1];
			v_map[2*j+1] = tmp_map[2];
		}
	}

	return _agents;
}

void Env::depth_by_obstacles(double* angle, Agent* agent, double* _map)
{
	double cur_d = _map[0];
	double cur_v[2];
	cur_v[0] = _map[1];
	cur_v[1] = _map[2];

	Obstacle* cur_obs;
	for(int i=0; i<obstacle_num; i++)
	{
		cur_obs = _obstacles.at(i);
		double d = Dist(agent->getP(), cur_obs->getP());
		double r = agent->getR() + cur_obs->getR();
		if(d - r > agent->getDepth())
			continue;

		double cur_dist = RayToSphereDistance(agent->getP(), cur_obs->getP(), angle, r);

		if(cur_dist > 0 && cur_dist < cur_d)
		{
			cur_d = cur_dist;
			double* _q = agent->getQ();
			cur_v[0] = -1*agent->getV()*_q[0];
			cur_v[1] = -1*agent->getV()*_q[1];
			Rotate2d(_q[0], _q[1], cur_v);
		}
	}

	_map[0] = cur_d;
	_map[1] = cur_v[0];
	_map[2] = cur_v[1];
}

void Env::depth_by_agents(double* angle, Agent* agent,  double* _map, int idx)
{
	double cur_d = _map[0];
	double cur_v[2];
	cur_v[0] = _map[1];
	cur_v[1] = _map[2];

	Agent* cur_agent;
	for(int i=0; i<agent_num; i++)
	{
		if(i == idx)
			continue;

		cur_agent = _agents.at(i);
		double d = Dist(agent->getP(), cur_agent->getP());
		double r = agent->getR() + cur_agent->getR();
		if(d - r > agent->getDepth())
			continue;

		double cur_dist = RayToSphereDistance(agent->getP(), cur_agent->getP(), angle, r);

		if(cur_dist >= 0 && cur_dist < cur_d)
		{
			cur_d = cur_dist;
			double* _q = agent->getQ();
			double* _q_cur = cur_agent->getQ();
			cur_v[0] = cur_agent->getV()*_q_cur[0] - agent->getV()*_q[0];
			cur_v[1] = cur_agent->getV()*_q_cur[1] - agent->getV()*_q[1];
			Rotate2d(_q[0], _q[1], cur_v);
		}
	}

	_map[0] = cur_d;
	_map[1] = cur_v[0];
	_map[2] = cur_v[1];
}

void Env::depth_by_walls(double* angle, Agent* agent, double* _map, int idx)
{
	bool isNew = false;

	double cur_d = _map[0];
	double cur_v[2] = {_map[1], _map[2]};
	double vision_depth = agent->getDepth();

	Wall* cur_wall;
	double offset[2];
	double* agent_pos = agent->getP();
	int wall_num = _walls.size();
	for(int i=0; i<wall_num; i++)
	{
		cur_wall = _walls.at(i);

		MinWallOffset(cur_wall, agent, offset);

		double wall_st[2];
		double wall_ed[2];
		vec_add_vec(cur_wall->getSt(), offset, wall_st);
		vec_add_vec(cur_wall->getEd(), offset, wall_ed);

		double L1[3];
		Line(wall_st, wall_ed, L1);

		double L2[3];
		double agent_ray[2];
		vec_add_scalar_vec(agent_pos, vision_depth, angle, agent_ray);
		Line(agent_pos, agent_ray, L2);

		double R[3];
		LineIntersenction(L1, L2, R);
		if(R[2])
		{
			double tmp_R[2] = {R[0], R[1]};

			double tmp_st_R[2];
			vec_sub_vec(wall_st, tmp_R, tmp_st_R);
			double l0_st = vec_norm(tmp_st_R);

			vec_sub_vec(wall_ed, tmp_R, tmp_st_R);
			double l0_ed = vec_norm(tmp_st_R);

			double tmp_p_R[2];
			vec_sub_vec(agent_pos, tmp_R, tmp_p_R);
			double l1_st = vec_norm(tmp_p_R);

			double tmp_ray[2];
			vec_add_scalar_vec(agent_pos, vision_depth, angle, tmp_ray);
			double tmp_ray_R[2];
			vec_sub_vec(tmp_ray, tmp_R, tmp_ray_R);
			double l1_ed = vec_norm(tmp_ray_R);

			double wall_l = cur_wall->getL();
			if(l0_st <= wall_l && l0_ed <= wall_l && l1_st <= vision_depth && l1_ed <= vision_depth)
			{
				double tmp_depth_vec[2];
				vec_sub_vec(agent_pos, tmp_R, tmp_depth_vec);
				double tmp_depth = vec_norm(tmp_depth_vec);
				if(tmp_depth < cur_d)
				{
					cur_d = tmp_depth;
					isNew = true;
				}
			}
		}

		double boundary = agent->getR();
		double cur_dist = RayToSphereDistance(agent_pos, cur_wall->getSt(), angle, boundary);

		if(cur_dist >= 0 && cur_dist < cur_d)
		{
			cur_d = cur_dist;
			isNew = true;
		}

		cur_dist = RayToSphereDistance(agent_pos, cur_wall->getEd(), angle, boundary);

		if(cur_dist >= 0 && cur_dist < cur_d)
		{
			cur_d = cur_dist;
			isNew = true;
		}
	}

	if(isNew)
	{
		double* _q = agent->getQ();
		cur_v[0] = -1 * agent->getV() * _q[0];
		cur_v[1] = -1 * agent->getV() * _q[1];
		Rotate2d(_q[0], _q[1], cur_v);
	}

	_map[0] = cur_d;
	_map[1] = cur_v[0];
	_map[2] = cur_v[1];
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
				double r = cur_agent->getR() + other_agent->getR() + 0.1;
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
			double r = cur_agent->getR() + cur_obs->getR() + 0.1;
			if(d < r)
			{
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

			if(LineSphereIntersection(cur_wall->getU(), cur_wall->getP(), cur_wall->getL(), cur_agent->getP(), cur_agent->getR()))
			{
				isCol = true;
				cur_agent->Revert(cur_wall->getSt(), false);
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

	if(_agents.at(0)->getCol())
		return true;

	if(Dist(_agents.at(0)->getD(), _agents.at(0)->getP()) < 1.5)
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
	double d_square = pow(d,2);

	// return -0.001*d_square;
	return -10.0*d;
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
