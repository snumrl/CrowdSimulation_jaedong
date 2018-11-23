#ifndef PASSING_H
#define PASSING_H

#include <vector>

#include "../Env.h"

using namespace std;

class Passing : public Env
{
	protected:

	public:
		Passing(int agent_n, int obs_n);
		~Passing();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





