#ifndef ENV_H
#define ENV_H

#include <vector>
#include "Agent.h"
#include "Obstacle.h"
#include "Wall.h"

using namespace std;

class Env{
	protected:
		vector<Agent* >  _agents;
		vector<Obstacle* >  _obstacles;
		vector<Wall* >  _walls;

		int agent_num;
		int obstacle_num;
		int wall_num;

		int _cur_step = 0;
		int _max_step = 400;
		double _reward;
		double _reward_sep[4];

		int eval_set_num = 6;
		vector<vector<double>> eval_agent_p;
		vector<vector<double>> eval_agent_r;
		vector<vector<double>> eval_agent_d;
		vector<vector<double>> eval_obs_p;
		vector<vector<double>> eval_obs_r;

	public:
		Env();
		~Env();

		virtual void Reset(int idx) = 0;
		virtual void initWalls() = 0;
		virtual void initEvaluation() = 0;
		virtual void ResetEval(int idx) = 0;
		virtual void ResetEnv() = 0;

		const vector<Agent*> & Observe();
		void setAction(int i, double w, double a_x, double a_y);
		void Update();
		double getReward();
		double* getRewardSep();
		double getTargetDist();
		double getTargetScore(double dist);
		double getSmoothScore(Agent* agent_);
		double getPrefVScore(Agent* agent_);
		double getBubbleScore(Agent* agent_);
		bool isTerm(bool isTest);
		bool isCol();

		// Object
		void addAgent(Agent* agent);
		void addObstacle(Obstacle* obstacle);
		void addWall(Wall* wall);

		const vector<Agent*> & getAgents();
		const vector<Obstacle*> & getObstacles();
		const vector<Wall*> & getWalls();

		Agent* getAgent(int id);
		Obstacle* getObstacle(int id);
		Wall* getWall(int id);

		int getNumAgents() const;
		int getNumObstacles() const;
		int getNumWalls() const;
};

#endif
