#ifndef BASIC_H
#define BASIC_H

#include <vector>

#include "../Env.h"

using namespace std;

class Basic : public Env
{
	protected:

	public:
		Basic(int agent_n, int obs_n);
		~Basic();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





