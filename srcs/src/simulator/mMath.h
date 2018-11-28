#ifndef MMATH_H
#define MMATH_H

#include "Agent.h"
#include "Wall.h"

void AngleToCoor(double angle, double* coor);
double AngleToRadian(double angle);
void RadianToCoor(double rad, double* coor);
double RadianToAngle(double rad);
double CoorToAngle(double* coor);
double CoorToRadian(double* coor);
double Dist(double* p1, double* p2);
double Dot(double* d1, double* d2);
double CrossProduct2d(double* v1, double* v2);
double InnerProduct2d(double* v1, double* v2);
void Rotate2d(double cos, double sin, double* p);
double RayToSphereDistance(double* p1, double* p2, double* angle, double r);
// void MinWallOffset(Wall* w, Agent* agent, double* offset);
void Line(double* p1, double* p2, double* L);
void LineIntersection(double* L1, double* L2, double* R);
void LineIntersection(double* st1, double* ed1, double* st2, double* ed2, double* R);
bool LineSphereIntersection(double* u, double* p, double l, double* q, double r);
bool LineEllipseIntersection(double* p_st, double* p_ed, double* ellipse, double* result);
void vec_assign(double* result, double* v);
void vec_sub_vec(double* v1, double* v2, double* result);
void vec_add_vec(double* v1, double* v2, double* result);
void vec_add_scalar_vec(double* v1, double s, double* v2, double* result);
void vec_mul_scalar(double* v, double s, double* result);
void vec_divide_scalar(double* v, double s, double* result);
double vec_norm(double* v);
double mClip(double lower, double upper, double value);
double Cov(double* elems, int size);

void bivariateForm(double* e, double* e_bv);
void getQuartic(double* bv1, double* bv2, double* quartric);
bool hasAzero(double* z);
bool isInEllipse(double*e, double* p);
bool colEllipsetoEllipse(double* e1, double* e2);
#endif
