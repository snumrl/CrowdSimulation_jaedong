#ifndef CORRIDOR_H
#define CORRIDOR_H

#include <vector>

#include "../Env.h"

using namespace std;

class Corridor : public Env
{
	protected:

	public:
		Corridor(int agent_n, int obs_n);
		~Corridor();

		virtual void Reset(int idx) override;
		virtual void initWalls() override;
		virtual void initEvaluation() override;
		virtual void ResetEval(int idx) override;
		virtual void ResetEnv() override;
};

#endif





