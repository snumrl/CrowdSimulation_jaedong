#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>

#include "Agent.h"
#include "mMath.h"

using namespace std;

Agent::Agent()
{

}

Agent::~Agent()
{
	delete _dmap;
	delete _delta;
}

void Agent::Update(double* p)
{
	_p[0] = p[0];
	_p[1] = p[1];
}

void Agent::Render()
{
	glColor3f(0.8, 0.2, 0.2);

	glPushMatrix();
	glTranslatef(_p[0], _p[1], 0.0);
	GLUquadric *sphere;
	sphere = gluNewQuadric();
	gluSphere(sphere, _r, 50, 10);
	glPopMatrix();
}

void Agent::setAction(double delta_t, double delta_v, bool isStop)
{
	if(isStop)
	{
		_stop = true;
		return ;
	}

	_front += delta_t*10;
	if(_front > 180)
		_front -= 360;

	if(_front < -180)
		_front += 360;

	AngleToCoor(_front, _q);

	_v += delta_v;
	if(_v > 3.0)
		_v = 3.0;
	if(_v < -0.3)
		_v = -0.3;
}

void Agent::Action()
{
	if(_stop)
		return;

	_p_prev[0] = _p[0];
	_p_prev[1] = _p[1];

	_p[0] += _v * _q[0];
	_p[1] += _v * _q[1];
}

void Agent::Revert(double* p, bool col)
{
	if(col)
	{
		double vec[2];
		vec[0] = _p[0] - p[0];
		vec[1] = _p[1] - p[1];

		_p[0] += 0.1*vec[0];
		_p[1] += 0.1*vec[1];
	}
	else
	{
		double vec[2];
		vec[0] = _p_prev[0] - _p[0];
		vec[1] = _p_prev[1] - _p[1];

		_p[0] = _p_prev[0] + 0.1*vec[0];
		_p[1] = _p_prev[1] + 0.1*vec[1];
	}
}



