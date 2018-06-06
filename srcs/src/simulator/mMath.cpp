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

	if(r < d_len || Dot(p12, angle)  < 0)
		return -1;

	double dist = Dot(p12, angle) - sqrt(pow(r,2) + pow(d_len, 2));
	if(dist < 0)
		dist = 0;

	return dist;
}
