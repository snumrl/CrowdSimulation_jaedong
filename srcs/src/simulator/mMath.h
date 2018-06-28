#ifndef MMATH_H
#define MMATH_H

#include "Agent.h"
#include "Wall.h"

void AngleToCoor(double angle, double* coor);
void RadianToCoor(double rad, double* coor);
double CoorToAngle(double* coor);
double CoorToRadian(double* coor);
double Dist(double* p1, double* p2);
void Rotate2d(double cos, double sin, double* p);
double RayToSphereDistance(double* p1, double* p2, double* angle, double r);
void MinWallOffset(Wall* w, Agent* agent, double* offset);
void Line(double* p1, double* p2, double* L);
void LineIntersenction(double* L1, double* L2, double* R);
bool LineSphereIntersection(double* u, double* p, double l, double* q, double r);
void vec_assign(double* result, double* v);
void vec_sub_vec(double* v1, double* v2, double* result);
void vec_add_vec(double* v1, double* v2, double* result);
void vec_add_scalar_vec(double* v1, double s, double* v2, double* result);
void vec_mul_scalar(double* v, double s, double* result);
void vec_divide_scalar(double* v, double s, double* result);
double vec_norm(double* v);
#endif
