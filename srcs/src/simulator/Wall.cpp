#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <iostream>
#include <math.h>

#include "Wall.h"

using namespace std;

Edge::Edge()
{
}

Edge::~Edge()
{
}

Edge::Edge(double* u, double* p, double l, double* normal)
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

Wall::Wall()
{
}

Wall::Wall(double* p, double w, double h)
{
	_p[0] = p[0];
	_p[1] = p[1];

	 _w = w;
	 _h = h;

	double st1[2];
	st1[0] = p[0] - 0.5*w;
	st1[1] = p[1] + 0.5*h;

	double st2[2];
	st2[0] = p[0] + 0.5*w;
	st2[1] = p[1] + 0.5*h;

	double st3[2];
	st3[0] = p[0] + 0.5*w;
	st3[1] = p[1] - 0.5*h;

	double st4[2];
	st4[0] = p[0] - 0.5*w;
	st4[1] = p[1] - 0.5*h;

	double normal1[2];
	normal1[0] = 0.0;
	normal1[1] = 1.0;

	double normal2[2];
	normal2[0] = 1.0;
	normal2[1] = 0.0;

	double normal3[2];
	normal3[0] = 0.0;
	normal3[1] = -1.0;

	double normal4[2];
	normal4[0] = -1.0;
	normal4[1] = 0.0;

	double dir1[2];
	dir1[0] = 1.0;
	dir1[1] = 0.0;

	double dir2[2];
	dir2[0] = 0.0;
	dir2[1] = -1.0;

	double dir3[2];
	dir3[0] = -1.0;
	dir3[1] = 0.0;

	double dir4[2];
	dir4[0] = 0.0;
	dir4[1] = 1.0;

	double len1 = w;
	double len2 = h;
	double len3 = w;
	double len4 = h;

	Edge* edge1 = new Edge(st1, dir1, len1, normal1);
	Edge* edge2 = new Edge(st2, dir2, len2, normal2);
	Edge* edge3 = new Edge(st3, dir3, len3, normal3);
	Edge* edge4 = new Edge(st4, dir4, len4, normal4);

	edges.push_back(edge1);
	edges.push_back(edge2);
	edges.push_back(edge3);
	edges.push_back(edge4);
}

Wall::~Wall()
{
}
