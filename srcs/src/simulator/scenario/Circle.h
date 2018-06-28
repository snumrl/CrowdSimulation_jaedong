#ifndef CIRCLE_H
#define CIRCLE_H

#include <vector>

#include "../Env.h"

using namespace std;

class Circle : public Env
{
	protected:

	public:
		Circle(int agent_n, int obs_n);
		~Circle();

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





