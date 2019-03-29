
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

	bool isRand = false;
	if(rand()%2 == 0){
		isRand = true;
	}

	double obs_x = -4 + rand()%16;
	if(isRand){
		for(int i=0; i<obstacle_num; i++)
		{
			double obs_pos[2];
			obs_pos[0] = -12.0 + rand()%20;
			// obs_pos[1] = -4.0 + rand()%8;
			obs_pos[1] = 24;

			double obs_r[2];
			obs_r[0] = (10 + rand()%10)/20.0;
			obs_r[1] = (10 + rand()%10)/20.0;

			Obstacle* obs = new Obstacle(obs_r, obs_pos); // p q d
			addObstacle(obs);
		}
	}
	else{
		double obs_h = -3.0 + rand()%6;
		for(int i=0; i<obstacle_num; i++)
		{
			double obs_pos[2];
			double obs_r[2];

			if(i==0){
				obs_pos[0] = obs_x;
				obs_pos[1] = 3.6 + obs_h;

				obs_r[0] = 2.8 + (rand()%3)*0.1;
				obs_r[1] = 2.8 + (rand()%3)*0.1;
			}
			else if(i==1){
				obs_pos[0] = obs_x;
				obs_pos[1] = -3.6 + obs_h;

				obs_r[0] = 2.8 + (rand()%3)*0.1;
				obs_r[1] = 2.8 + (rand()%3)*0.1;
			}
			else{
				while(true){
					obs_pos[0] = -10.0 + rand()%18;
					if(obs_pos[0] < obs_x - 5.0 || obs_pos[0] > obs_x + 5.0)
						break;
				}

				obs_pos[1] = -4.0 + rand()%8;

				obs_r[0] = (10 + rand()%10)/20.0;
				obs_r[1] = (10 + rand()%10)/20.0;
			}

			double tmp;
			if(obs_r[0] < obs_r[1]){
				tmp = obs_r[0];
				obs_r[0] = obs_r[1];
				obs_r[1] = tmp;
			}

			Obstacle* obs = new Obstacle(obs_r, obs_pos); // p q d
			addObstacle(obs);
		}
	}

	for(int i=0; i<agent_num; i++)
	{
		double agent_r[2];
		agent_r[0] = (3 + rand()%5)/10.0;
		agent_r[1] = (3 + rand()%2)/10.0;
		// agent_r[0] = (3 + rand()%2)/10.0;
		// agent_r[1] = agent_r[0];
		// agent_r[0] = 1.0;
		// agent_r[1] = 0.3;

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
				agent_pos[0] = obs_x + 4.0 + (rand()%80)*0.1;
				// agent_pos[0] = 0.0;
				agent_pos[1] = -2.5 + (rand()%50)*0.1;
			}
			else{
				agent_pos[0] = 12.0 + (rand()%160)*0.1;
				agent_pos[1] = -2.5 + (rand()%50)*0.1;
			}

			col = false;
			int start_idx = 0;
			for(int j=start_idx; j<i; j++)
			{
				double boundary = (getAgent(j)->getR())[0] + agent_r[0] + 0.5;
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
					double boundary = getObstacle(j)->getR()[0] + agent_r[0] + 1.0;
					if(Dist(agent_pos, getObstacle(j)->getP()) < boundary)
					{
						col = true;
						break;
					}
				}
			}

			if(col == false)
				break;

			std::cout << "agent " << i << " : " << agent_pos[0] << "," << agent_pos[1] << std::endl;
		}

		agent->setP(agent_pos[0], agent_pos[1]);
		agent->setPprev(agent_pos[0], agent_pos[1]);

		bool d_col = false;
		double d_pos[2];
		while(true){
			if(i%2==0){
				d_pos[0] = -24 + rand()%4;
				d_pos[1] = -5 + rand()%10;
			}
			else{
				d_pos[0] = 20 + rand()%4;
				d_pos[1] = agent_pos[1];
			}
			if(i==0){
				d_pos[0] = -20 + rand()%4;
			}

			d_col = false;
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

			std::cout << "goal " << i << " : " << agent_pos[0] << "," << agent_pos[1] << std::endl;

		}

		// double cur_front = (rand()%314)/100.+3.14*0.5;
		// double y_coord[2];
		// double x_coord[2];
		// RadianToCoor(cur_front, y_coord);
		// RadianToCoor(cur_front-0.5*3.141592, x_coord);

		// agent->setFront(cur_front);
		// agent->setQy(y_coord[0], y_coord[1]);
		// agent->setQx(x_coord[0], x_coord[1]);
		// agent->setD( d_pos[0], d_pos[1]);
		// agent->setColor(0.9, 0.5, 0.1);

		if(i%2==0){
			// double cur_front = ((rand()%628)/100.0)-3.14;
			// double cur_front = 0.5*3.14 + ((rand()%314)/100.0);
			double cur_front = 3.14;
			double y_coord[2];
			double x_coord[2];
			RadianToCoor(cur_front, y_coord);
			RadianToCoor(cur_front-0.5*3.141592, x_coord);

			agent->setFront(cur_front);
			agent->setQy(y_coord[0], y_coord[1]);
			agent->setQx(x_coord[0], x_coord[1]);
			agent->setD( d_pos[0], d_pos[1]);
			agent->setColor(0.1, 0.9, 0.1);
		}
		else{
			double cur_front = (rand()%314)/100.- 3.14*0.5;
			double y_coord[2];
			double x_coord[2];
			RadianToCoor(cur_front, y_coord);
			RadianToCoor(cur_front-0.5*3.141592, x_coord);

			agent->setFront(cur_front);
			agent->setQy(y_coord[0], y_coord[1]);
			agent->setQx(x_coord[0], x_coord[1]);
			agent->setD( d_pos[0], d_pos[1]);
			agent->setColor(0.9, 0.1, 0.1);
		}
		if(i==0)
			agent->setColor(0.9, 0.5, 0.1);

		addAgent(agent);
	}
	_cur_step = 0;
}




