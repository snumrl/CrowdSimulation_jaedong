#include <math.h>

#include "mMath.h"

#define PI 3.14159265

void AngleToCoor(double angle, double* coor)
{
	coor[0] = cos( angle * PI / 180.0);
	coor[1] = sin( angle * PI / 180.0);
}

double Dist(double* p1, double* p2)
{
	return sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2));
}

double Dot(double* d1, double* d2)
{
	return d1[0]*d2[0] + d1[1]*d2[1];
}

void Rotate2d(double cos, double sin, double* p)
{
	double p0 = cos * p[0] + sin * p[1];
	double p1 = -1 * sin * p[0] + cos * p[1];
	p[0] = p0;
	p[1] = p1;
}

void MinWallOffset(Wall* w, Agent* agent, double* offset)
{
	double* p = agent->getP();
	double r = agent->getR();

	double* st = w->getSt();
	double* ed = w->getEd();

	if(st[0] == ed[0])
	{
		offset[1] = 0.0;
		if(st[0] > p[0])
			offset[0] = -r;
		else
			offset[0] = r;

		return ;
	}

	if(st[1] == ed[1])
	{
		offset[0] = 0.0;
		if(st[1] > p[1])
			offset[1] = -r;
		else
			offset[1] = r;

		return ;
	}

	offset[0] = 0;
	offset[1] = 0;
}

double RayToSphereDistance(double* p1, double* p2, double* angle, double r)
{
	double p12[2];
	p12[0] = p2[0] - p1[0];
	p12[1] = p2[1] - p1[1];

	double d[2];
	double dot_value = Dot(p12, angle);
	d[0] = p12[0] - dot_value*angle[0];
	d[1] = p12[1] - dot_value*angle[1];

	double d_len = sqrt(Dot(d, d));

	if(r < d_len || dot_value  < 0)
		return -1;

	double dist;
	if(dot_value < 0)
		dist = -1*dot_value - sqrt(pow(r,2) - pow(d_len, 2));
	else
		dist = dot_value - sqrt(pow(r,2) - pow(d_len, 2));

	if(dist < 0)
		dist = 0;

	return dist;
}

void Line(double* p1, double* p2, double* L)
{
	L[0] = p1[1] - p2[1];
	L[1] = p2[0] - p1[0];
	L[2] = p2[0]*p1[1] - p1[0]*p2[1];
}

void LineIntersenction(double* L1, double* L2, double* R)
{
	double D  = L1[0] * L2[1] - L1[1] * L2[0];
	double Dx = L1[2] * L2[1] - L1[1] * L2[2];
	double Dy = L1[0] * L2[2] - L1[2] * L2[0];

	R[0] = Dx/D;
	R[1] = Dy/D;
	if(D != 0)
	{
		R[2] = 1;
	}
	else
		R[2] = 0;
}

bool LineSphereIntersection(double* u, double* p, double l, double* q, double r)
{
	double q_u[2];
	vec_sub_vec(q, u, q_u);
	double x = vec_norm(q_u);

	double y = Dot(p, q_u);

	if (y +r < 0)
		return false;

	if (y - r > l)
		return false;

	if (x*x - y*y < r*r)
		return true;

	return false;
}

void ves_assign(double* result, double* v)
{
	result[0] = v[0];
	result[1] = v[1];
}

void vec_sub_vec(double* v1, double* v2, double* result)
{
	result[0] = v1[0] - v2[0];
	result[1] = v1[1] - v2[1];
}

void vec_add_vec(double* v1, double* v2, double* result)
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
}

void vec_add_scalar_vec(double* v1, double s, double* v2, double* result)
{
	result[0] = v1[0] + s*v2[0];
	result[1] = v1[1] + s*v2[1];
}

void vec_mul_scalar(double* v, double s, double* result)
{
	result[0] = s * v[0];
	result[1] = s * v[1];
}

double vec_norm(double* v)
{
	return sqrt(pow(v[0], 2) + pow(v[1], 2));
}
