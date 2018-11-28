#ifndef HALLWAY_H
#define HALLWAY_H

#include <vector>

#include "../Env.h"

using namespace std;

class Hallway : public Env
{
	protected:

	public:
		Hallway(int agent_n, int obs_n);
		~Hallway();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





