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
		int _max_step = 300;
		int _vision_depth = 30;

		double _reward;

		int eval_set_num = 10;

		vector<int> eval_agent_p_x;
		vector<int> eval_agent_p_y;
		vector<int> eval_agent_d_x;
		vector<int> eval_agent_d_y;
		vector<int> eval_obs_p_x;
		vector<int> eval_obs_p_y;

	public:
		Env();
		~Env();

		virtual void Reset(int idx) = 0;
		virtual void initWalls() = 0;
		virtual void initEvaluation() = 0;
		virtual void ResetEval(int idx) = 0;
		virtual void ResetEnv() = 0;

		const vector<Agent*> & Observe();
		void setAction(int i, double t, double v, bool s);
		void Update();
		double getReward();
		double getScore();
		bool isTerm(bool isTest);
		bool isCol();

		void depth_by_obstacles(double* angle, Agent* agent, double* _map);
		void depth_by_agents(double* angle, Agent* agent,  double* _map, int idx);
		void depth_by_walls(double* angle, Agent* agent,  double* _map, int idx);

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
