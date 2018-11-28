#include <iostream>
#include <ctime>
#include <omp.h>
#include "../mMath.h"
#include "Passing.h"

using namespace std;

Passing::Passing(int agent_n, int obs_n)
{
	agent_num = agent_n;
	obstacle_num = obs_n;

	initEvaluation();
	initWalls();

	Reset(-1);
}

void Passing::initWalls()
{
	wall_num = _walls.size();
}

Passing::~Passing()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();
}

void Passing::initEvaluation()
{
	srand((unsigned int)time(0));

	for(int k=0; k<eval_set_num; k++)
	{
		vector<double> cur_obs_p;
		vector<double> cur_obs_r;
		for(int i=0; i<obstacle_num; i++)
		{
			double obs_pos[2];
			obs_pos[0] = -18.0 + 10*(i%4);
			obs_pos[1] = -18.0 + 9*(i/4);

			double obs_r[2];
			obs_r[0] = (10 + rand()%60)/20.0;
			obs_r[1] = (10 + rand()%60)/20.0;

			double tmp;
			if(obs_r[0] < obs_r[1]){
				tmp = obs_r[0];
				obs_r[0] = obs_r[1];
				obs_r[1] = tmp;
			}

			cur_obs_p.push_back(obs_pos[0]);
			cur_obs_p.push_back(obs_pos[1]);
			cur_obs_r.push_back(obs_r[0]);
			cur_obs_r.push_back(obs_r[1]);
		}
		eval_obs_p.push_back(cur_obs_p);
		eval_obs_r.push_back(cur_obs_r);

		vector<double> cur_agent_p;
		vector<double> cur_agent_r;
		vector<double> cur_agent_d;
		for(int i=0; i<agent_num; i++)
		{
			double agent_r[2];
			agent_r[0] = (10 + rand()%20)/20.0;
			agent_r[1] = (10 + rand()%20)/20.0;

			double tmp;
			if(agent_r[0] < agent_r[1]){
				tmp = agent_r[0];
				agent_r[0] = agent_r[1];
				agent_r[1] = tmp;
			}

			// agent_r[1] = agent_r[0];

			bool col = false;
			double agent_pos[2];
			while(true)
			{
				agent_pos[0] = 20.0 + rand()%4;
				agent_pos[1] = -12.0 + rand()%24;

				col = false;
				int start_idx = 0;
				for(int j=start_idx; j<i; j++)
				{
					double boundary = cur_agent_r.at(j*2) + agent_r[0] + 0.1;
					double tmp_p[2];
					tmp_p[0] = cur_agent_p.at(j*2);
					tmp_p[1] = cur_agent_p.at(j*2+1);
					if(Dist(agent_pos, tmp_p) < boundary)
					{
						col = true;
						break;
					}
				}

				if(col == false)
				{
					for(int j=0; j<obstacle_num; j++)
					{
						double boundary = cur_obs_r.at(j*2) + agent_r[0];
						double tmp_p[2];
						tmp_p[0] = cur_obs_p.at(j*2);
						tmp_p[1] = cur_obs_p.at(j*2+1);
						if(Dist(agent_pos, tmp_p) < boundary)
						{
							col = true;
							break;
						}
					}
				}

				if(col == false)
					break;
			}

			bool d_col = false;
			double agent_d[2];
			while(true){
				d_col = false;
				agent_d[0] = -15 + rand()%30;
				agent_d[1] = -15 + rand()%30;

				for(int j=0; j<obstacle_num; j++)
				{
					double boundary = cur_obs_r.at(j*2) + 0.5;
					double tmp_p[2];
					tmp_p[0] = cur_obs_p.at(j*2);
					tmp_p[1] = cur_obs_p.at(j*2+1);
					if(Dist(agent_d, tmp_p) < boundary)
					{
						d_col = true;
						break;
					}
				}
				if(!d_col)
					break;
			}

			cur_agent_p.push_back(agent_pos[0]);
			cur_agent_p.push_back(agent_pos[1]);
			cur_agent_r.push_back(agent_r[0]);
			cur_agent_r.push_back(agent_r[1]);
			cur_agent_d.push_back(agent_d[0]);
			cur_agent_d.push_back(agent_d[0]);
		}
		eval_agent_p.push_back(cur_agent_p);
		eval_agent_r.push_back(cur_agent_r);
		eval_agent_d.push_back(cur_agent_d);
	}
}

void Passing::Reset(int idx)
{
	if(idx == -1)
		ResetEnv();
	else
		ResetEval(idx);
}

void Passing::ResetEval(int idx)
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();

	for(int i=0; i<agent_num; i++)
	{
		double agent_r[2];
		agent_r[0] = eval_agent_r.at(idx).at(i*2);
		agent_r[1] = eval_agent_r.at(idx).at(i*2+1);

		Agent* agent = new Agent(agent_r);
		agent->setId(i);
		agent->setP(eval_agent_p.at(idx).at(i*2), eval_agent_p.at(idx).at(i*2+1));
		agent->setPprev(eval_agent_p.at(idx).at(i*2), eval_agent_p.at(idx).at(i*2+1));
		agent->setD(eval_agent_d.at(idx).at(i*2), eval_agent_d.at(idx).at(i*2+1));
		agent->setQy(1.0, 0.0);
		agent->setQx(0.0, -1.0);
		agent->setFront(0.0);
		agent->setColor(0.9, 0.1, 0.1);
		addAgent(agent);
	}

	for(int i=0; i<obstacle_num; i++)
	{
		double obstacle_r[2];
		obstacle_r[0] = eval_obs_r.at(idx).at(i*2);
		obstacle_r[1] = eval_obs_r.at(idx).at(i*2+1);

		double obstacle_p[2];
		obstacle_p[0] = eval_obs_p.at(idx).at(i*2);
		obstacle_p[1] = eval_obs_p.at(idx).at(i*2+1);

		Obstacle* obstacle = new Obstacle(obstacle_p, obstacle_r);
		addObstacle(obstacle);
	}

	_cur_step = 0;
}

void Passing::ResetEnv()
{
	for(vector< Agent* >::iterator it = _agents.begin() ; it != _agents.end(); it++)
		delete (*it);
	_agents.clear();

	for(vector< Obstacle* >::iterator it = _obstacles.begin() ; it != _obstacles.end(); it++)
		delete (*it);
	_obstacles.clear();

	srand((unsigned int)time(0));

	// for(int i=0; i<8; i++)
	// {
	// 	double obs_pos[2];
	// 	obs_pos[0] = -18.0 + 10*(i%4);
	// 	obs_pos[1] = -18.0 + 9*(i/4);

	// 	double obs_r[2];
	// 	obs_r[0] = (10 + rand()%60)/20.0;
	// 	obs_r[1] = (10 + rand()%60)/20.0;

	// 	double tmp;
	// 	if(obs_r[0] < obs_r[1]){
	// 		tmp = obs_r[0];
	// 		obs_r[0] = obs_r[1];
	// 		obs_r[1] = tmp;
	// 	}

	// 	Obstacle* obs = new Obstacle(obs_pos, obs_r); // p q d
	// 	addObstacle(obs);
	// }

	// #pragma omp parallel for
	for(int i=0; i<10; i++)
	{
		double obs_pos[2];
		obs_pos[0] = -16.0 + 3*i;
		obs_pos[1] = 4.0;
		if(i>2 && i<6)
			obs_pos[1] = 6.4;

		double obs_r[2];
		obs_r[0] = 3.0;
		obs_r[1] = 3.0;

		Obstacle* obs = new Obstacle(obs_pos, obs_r); // p q d
		addObstacle(obs);

		obs_pos[0] = -16.0 + 3*i;
		obs_pos[1] = -4.0;
		if(i>2 && i<6)
			obs_pos[1] = -6.4;

		obs_r[0] = 3.0;
		obs_r[1] = 3.0;

		obs = new Obstacle(obs_pos, obs_r); // p q d
		addObstacle(obs);
	}

	for(int i=0; i<2; i++)
	{
		double obs_pos[2];
		obs_pos[0] = 17.0;
		obs_pos[1] = 7.0;
		if(i==1)
			obs_pos[1] = -7.0;

		double obs_r[2];
		obs_r[0] = 4.0;
		obs_r[1] = 4.0;

		Obstacle* obs = new Obstacle(obs_pos, obs_r); // p q d
		addObstacle(obs);
	}

	double obs_pos[2];
	obs_pos[0] = 23.0;
	obs_pos[1] = 0.0;

	double obs_r[2];
	obs_r[0] = 5.0;
	obs_r[1] = 5.0;

	Obstacle* obs = new Obstacle(obs_pos, obs_r); // p q d
	addObstacle(obs);

	for(int i=0; i<agent_num; i++)
	{
		double agent_r[2];
		// agent_r[0] = (10 + rand()%20)/20.0;
		// agent_r[1] = (10 + rand()%20)/20.0;

		agent_r[0] = (10 + rand()%25)/20.0;
		agent_r[1] = 0.5;

		double tmp;
		if(agent_r[0] < agent_r[1]){
			tmp = agent_r[0];
			agent_r[0] = agent_r[1];
			agent_r[1] = tmp;
		}

		// agent_r[1] = agent_r[0];

		Agent* agent = new Agent(agent_r);
		agent->setId(i);

		bool col = false;
		double agent_pos[2];
		while(true)
		{
			agent_pos[0] = 16.0 + rand()%2;
			agent_pos[1] = -1.0 + rand()%2;

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
			// d_pos[0] = -24 + rand()%6;
			// d_pos[1] = -12 + rand()%24;
			d_pos[0] = -23 + rand()%3;
			d_pos[1] = 0;

			for(int j=0; j<obstacle_num; j++)
			{
				double boundary = getObstacle(j)->getR()[0] + 0.5;
				if(Dist(d_pos, getObstacle(j)->getP()) < boundary)
				{
					d_col = true;
					break;
				}
			}
			if(!d_col)
				break;
		}

		agent->setD( d_pos[0], d_pos[1]);
		double front_rnd = (rand()%20 - 10)*0.1*3.141592;
		double qy[2];
		RadianToCoor(front_rnd, qy);
		double qx[2];
		RadianToCoor(front_rnd-3.141592/2.0, qx);
		agent->setQy(qy[0], qy[1]);
		agent->setQx(qx[0], qx[1]);
		agent->setFront(front_rnd);
		agent->setColor(0.9, 0.1, 0.1);
		if(i==0)
			agent->setColor(0.1, 0.9, 0.1);

		// double* dmap = new double[20];
		// for(int j=0; j<20; j++){
		// 	dmap[j] = _vision_depth;
		// }

		// double* vmap = new double[40];
		// for(int j=0; j<40; j++){
		// 	vmap[j] = 0.0;
		// }

		// agent->setDmap(dmap);
		// agent->setVmap(vmap);

		addAgent(agent);
	}
	_cur_step = 0;
}




