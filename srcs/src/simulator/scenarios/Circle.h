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
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





