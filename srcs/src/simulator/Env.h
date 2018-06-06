#ifndef ENV_H
#define ENV_H

#include <vector>
#include "Agent.h"
#include "Obstacle.h"

using namespace std;

class Env{
	protected:
		vector<Agent* >  _agents;
		vector<Obstacle* >  _obstacles;

		int agent_num;
		int obstacle_num;

		int _cur_step = 0;
		int _max_step = 400;
		int _vision_depth = 300;

		double _reward;

	public:
		Env();
		~Env();

		virtual void Reset(int idx) = 0;
		virtual void Render() = 0;

		const vector<Agent*> & Observe();
		void setAction(int i, double t, double v, bool s);
		void Update();
		double getReward();
		double getScore();
		bool isTerm(bool isTest);

		double depth_by_obstacles(double* angle, Agent* agent, double cur_d);
		double depth_by_agents(double* angle, Agent* agent, double cur_d, int idx);

		// Object

		void addAgent(Agent* agent);
		void addObstacle(Obstacle* obstacle);

		const vector<Agent*> & getAgents();
		const vector<Obstacle*> & getObstacles();

		Agent* getAgent(int id);
		Obstacle* getObstacle(int id);

		int getNumAgents() const;
		int getNumObstacles() const;

};

#endif
