#ifndef CROSSWAY_H
#define CROSSWAY_H

#include <vector>

#include "../Env.h"

using namespace std;

class Crossway : public Env
{
	protected:

	public:
		Crossway(int agent_n, int obs_n);
		~Crossway();

		virtual void Reset(int idx) override;
		virtual void Render() override;

		void initEvaluation();

		void ResetEval(int idx);
		void ResetEnv();

		vector<int> eval_agent_p_x;
		vector<int> eval_agent_p_y;
		vector<int> eval_agent_d_x;
		vector<int> eval_agent_d_y;
		vector<int> eval_obs_p_x;
		vector<int> eval_obs_p_y;
};

#endif





