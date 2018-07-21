#ifndef BOTTLENECK_H
#define BOTTLENECK_H

#include <vector>

#include "../Env.h"

using namespace std;

class Bottleneck : public Env
{
	protected:

	public:
		Bottleneck(int agent_n, int obs_n);
		~Bottleneck();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





