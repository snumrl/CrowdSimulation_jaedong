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
	wall_num = 5;

	double p1[2] = {0.0, 10.0};
	double w1 = 48.0;
	double h1 = 12.0;
	addWall(new Wall(p1, w1, h1));

	double p2[2] = {0.0, -10.0};
	double w2 = 48.0;
	double h2= 12.0;
	addWall(new Wall(p2, w2, h2));

	double p3[2] = {-6.0, 2.2};
	double w3 = 4.0;
	double h3 = 1.0;
	addWall(new Wall(p3, w3, h3));

	double p4[2] = {-6.0, 0.0};
	double w4 = 4.0;
	double h4= 1.0;
	addWall(new Wall(p4, w4, h4));

	double p5[2] = {-6.0, -2.2};
	double w5 = 4.0;
	double h5= 1.0;
	addWall(new Wall(p5, w5, h5));

	double p6[2] = {2.0, 2.2};
	double w6 = 4.0;
	double h6 = 1.0;
	addWall(new Wall(p6, w6, h6));

	double p7[2] = {2.0, 0.0};
	double w7 = 4.0;
	double h7= 1.0;
	addWall(new Wall(p7, w7, h7));

	double p8[2] = {2.0, -2.2};
	double w8 = 4.0;
	double h8= 1.0;
	addWall(new Wall(p8, w8, h8));

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
}

void Passing::ResetEnv()
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

	srand((unsigned int)time(0));

	double p1[2] = {0.0, 10.0};
	double w1 = 48.0;
	double h1 = 12.0;
	addWall(new Wall(p1, w1, h1));

	double p2[2] = {0.0, -10.0};
	double w2 = 48.0;
	double h2= 12.0;
	addWall(new Wall(p2, w2, h2));

	for(int i=0; i<3; i++)
	{
		double wall_pos1[2];
		wall_pos1[0] = -6 + rand()%4;
		wall_pos1[1] = 2.2 - i*2.2;

		double w1 = 4.0;
		double h1 = 0.8;

		double wall_pos2[2];
		wall_pos2[0] = 2.0 + rand()%4;
		wall_pos2[1] = 2.2 - i*2.2;

		double w2 = 4.0;
		double h2 = 0.8;

		addWall(new Wall(wall_pos1, w1, h1));
		addWall(new Wall(wall_pos2, w2, h2));
	}

	for(int i=0; i<0; i++)
	{
		double obs_pos[2];
		obs_pos[0] = -12.0 + rand()%20;
		obs_pos[1] = -4.0 + rand()%8;

		double obs_r[2];
		obs_r[0] = (10 + rand()%10)/20.0;
		obs_r[1] = (10 + rand()%10)/20.0;

		Obstacle* obs = new Obstacle(obs_r, obs_pos); // p q d
		addObstacle(obs);
	}

	for(int i=0; i<agent_num; i++)
	{
		double agent_r[2];
		agent_r[0] = (3 + rand()%7)/10.0;
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
				agent_pos[0] = 15.0 + (rand()%30)*0.1;
				// agent_pos[0] = 0.0;
				agent_pos[1] = -3.0 + (rand()%60)*0.1;
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
				d_pos[1] = agent_pos[1];
			}
			else{
				d_pos[0] = 20 + rand()%4;
				d_pos[1] = agent_pos[1];
			}
			if(i==0){
				d_pos[0] = -16 + rand()%2;
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
			double cur_front = ((rand()%628)/100.0)-3.14;
			// double cur_front = 3.14;
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




