#ifndef MIx_H
#define MIx_H

#include <vector>

#include "../Env.h"

using namespace std;

class Mix : public Env
{
	protected:

	public:
		Mix(int agent_n, int obs_n);
		~Mix();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





