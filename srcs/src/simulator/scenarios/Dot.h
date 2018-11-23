#ifndef DOT_H
#define DOT_H

#include <vector>

#include "../Env.h"

using namespace std;

class Dot : public Env
{
	protected:

	public:
		Dot(int agent_n, int obs_n);
		~Dot();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





