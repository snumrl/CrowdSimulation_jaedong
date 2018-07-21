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
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





