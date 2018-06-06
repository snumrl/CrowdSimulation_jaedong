#ifndef MMATH_H
#define MMATH_H

void AngleToCoor(double angle, double* coor);
double Dist(double* p1, double* p2);
double RayToSphereDistance(double* p1, double* p2, double* angle, double r);
#endif
