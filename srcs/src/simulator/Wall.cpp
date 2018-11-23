#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <iostream>
#include <math.h>

#include "Wall.h"


using namespace std;

Wall::Wall()
{
}

Wall::~Wall()
{
}

Wall::Wall(double* u, double* p, double l, double* normal)
{
	_st[0] = u[0];
	_st[1] = u[1];

	_n[0] = normal[0];
	_n[1] = normal[1];

	double p_len = sqrt(pow(p[0], 2) + pow(p[1], 2));
	_p[0] = p[0]/p_len;
	_p[1] = p[1]/p_len;

	_l = l;

	_ed[0] = _st[0] + _l * _p[0];
	_ed[1] = _st[1] + _l * _p[1];

	_u[0] = u[0];
	_u[1] = u[1];

	_q[0] = -1*_p[1];
	_q[1] = _p[0];
}



void Wall::getData(double* _data)
{

}
