#include <math.h>
#include <vector>
#include <iostream>

#include "mMath.h"

#define PI 3.14159265

void AngleToCoor(double angle, double* coor)
{
	coor[0] = cos( angle * PI / 180.0);
	coor[1] = sin( angle * PI / 180.0);
}

double AngleToRadian(double angle)
{
	return angle / 180.0 * PI;
}

void RadianToCoor(double rad, double* coor)
{
	coor[0] = cos( rad );
	coor[1] = sin( rad );
}

double RadianToAngle(double rad)
{
	return rad / PI * 180.0;
}

double CoorToAngle(double* coor)
{
	return atan2(coor[1], coor[0]) * 180.0 / (double)PI;
}

double CoorToRadian(double* coor)
{
	return atan2(coor[1], coor[0]);
}

double Dist(double* p1, double* p2)
{
	return sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2));
}

double CrossProduct2d(double* v1, double* v2)
{
	return v1[0]*v2[1] - v1[1]*v2[0];
}

double InnerProduct2d(double* v1, double* v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1];
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

// void MinWallOffset(Wall* w, Agent* agent, double* offset)
// {
// 	double* p = agent->getP();
// 	double* r = agent->getR();

// 	double* st = w->getSt();
// 	double* ed = w->getEd();
// 	double* n = w->getNormal();

// 	offset[0] = r[0] * n[0];
// 	offset[1] = r[0] * n[1];
// }

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

void LineIntersection(double* L1, double* L2, double* R)
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

void LineIntersection(double* st1, double* ed1, double* st2, double* ed2, double* R)
{
	double L1[3];
	Line(st1, ed1, L1);

	double L2[3];
	Line(st2, ed2, L2);

	double D  = L1[0] * L2[1] - L1[1] * L2[0];
	double Dx = L1[2] * L2[1] - L1[1] * L2[2];
	double Dy = L1[0] * L2[2] - L1[2] * L2[0];

	R[0] = Dx/D;
	R[1] = Dy/D;
	if(D != 0)
	{
		double t1_x, t1_y, t2_x, t2_y;
		t1_x = (R[0] - st1[0]) / (ed1[0] - st1[0]);
		t1_y = (R[1] - st1[1]) / (ed1[1] - st1[1]);
		t2_x = (R[0] - st2[0]) / (ed2[0] - st2[0]);
		t2_y = (R[0] - st2[0]) / (ed2[0] - st2[0]);
		if(t1_x >= 0.0 && t1_x <= 1.0 && t1_y >= 0.0 && t1_y <= 1.0 && t2_x >= 0.0 && t2_x <= 1.0 && t2_y >= 0.0 && t2_y <= 1.0)
		{
			R[2] = 1;
		}
		else{
			R[2] = 0;
		}
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

bool LineEllipseIntersection(double* p_st_, double* p_ed_, double* ellipse, double* result)
{
	//ellipse : p[2], r[2], front, qx[2], qy[2]
	double p_st[2];
	p_st[0] = p_st_[0];
	p_st[1] = p_st_[1];

	double p_ed[2];
	p_ed[0] = p_ed_[0];
	p_ed[1] = p_ed_[1];

	p_st[0] -= ellipse[0];
	p_st[1] -= ellipse[1];

	p_ed[0] -= ellipse[0];
	p_ed[1] -= ellipse[1];

	double a = ellipse[2];
	double b = ellipse[3];
	double front = ellipse[4];

	Rotate2d(cos(PI/2.0-front), -sin(PI/2.0-front), p_st);
	Rotate2d(cos(PI/2.0-front), -sin(PI/2.0-front), p_ed);

	double diff_x = p_ed[0] - p_st[0];
	double diff_y = p_ed[1] - p_st[1];

	double A = (diff_x*diff_x)/(a*a) + (diff_y*diff_y)/(b*b);
	double B = 2*p_st[0]*(diff_x)/(a*a) + 2*p_st[1]*(diff_y)/(b*b);
	double C = (p_st[0]*p_st[0])/(a*a) + (p_st[1]*p_st[1])/(b*b) - 1;

	std::vector<double> t_values;

	double D = B*B - 4*A*C;
	double t_;
	double t__;
	if(D == 0){
		t_ = -1*B/(2*A);
		if(t_ >= 0 && t_ <= 1)
			t_values.push_back(t_);
	}
	else if(D > 0){
		t_  = (-1*B+sqrt(D))/(2*A);
		t__ = (-1*B-sqrt(D))/(2*A);
		if(t_ >= 0 && t_ <= 1)
			t_values.push_back(t_);

		if(t__ >= 0 && t__ <= 1)
			t_values.push_back(t__);
	}

	int pts_num = t_values.size();
	double cur_t = -1;
	double t1, t2;
	if(pts_num == 1){
		cur_t = t_values.at(0);
	}
	else if(pts_num == 2){
		t1 = t_values.at(0);
		t2 = t_values.at(1);
		cur_t = t1 < t2 ? t1 : t2;
	}

	if(cur_t != -1){
		double cur_pts[2];
		cur_pts[0] = p_st[0] + (diff_x)*cur_t;
		cur_pts[1] = p_st[1] + (diff_y)*cur_t;
		Rotate2d(cos(front - PI/2.0), -sin(front - PI/2.0), cur_pts);
		cur_pts[0] += ellipse[0];
		cur_pts[1] += ellipse[1];

		result[0] = cur_pts[0];
		result[1] = cur_pts[1];
		return true;
	}

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

void vec_divide_scalar(double* v, double s, double* result)
{
	result[0] = v[0] / s;
	result[1] = v[1] / s;
}

double vec_norm(double* v)
{
	return sqrt(pow(v[0], 2) + pow(v[1], 2));
}

double mClip(double lower, double upper, double value)
{
	if(value < lower)
		return lower;
	if(value > upper)
		return upper;
	return value;
}

double Cov(double* elems, int size)
{
	double mean = 0.0;
	double cov = 0.0;

	for(int i=0; i<size; i++)
		mean += elems[i];
	mean /= double(size);

	// std::cout << "size : " << size << std::endl;

	// std::cout << "mean : " << mean << std::endl;

	for(int i=0; i<size; i++)
		cov += (elems[i]-mean)*(elems[i]-mean);
	cov /= double(size);

	// std::cout << "cov : " << cov << std::endl;

	 return cov;
}

void bivariateForm(double* e, double* e_bv)
{
	double A = cos(e[4]);
	double B = sin(e[4]);
	double a =  A * e[0] + B * e[1];
	double c = -B * e[0] + A * e[1];

	A = cos(-e[4]);
	B = sin(-e[4]);
	double b = e[2]*e[2];
	double d = e[3]*e[3];

	double coeff_xx = (A * A / b) + (B * B / d);
	double coeff_xy = (-2 * A * B / b) + (2 * A * B / d);
	double coeff_yy = (B * B / b) + (A * A / d);
	double coeff_x = (-2 * a * A / b) - (2 * c * B / d);
	double coeff_y = (2 * a * B / b) - (2 * c * A / d);
	double coeff_const = (a * a / b) + (c * c / d) - 1;

	e_bv[0] = coeff_xx;
	e_bv[1] = coeff_xy;
	e_bv[2] = coeff_yy;
	e_bv[3] = coeff_x;
	e_bv[4] = coeff_y;
	e_bv[5] = coeff_const;
}

void getQuartic(double* bv1, double* bv2, double* quartric)
{
	double a = bv1[0];
	double b = bv1[1];
	double c = bv1[2];
	double d = bv1[3];
	double e = bv1[4];
	double f = bv1[5];

	double a1 = bv2[0];
	double b1 = bv2[1];
	double c1 = bv2[2];
	double d1 = bv2[3];
	double e1 = bv2[4];
	double fq = bv2[5];

	double z0 = f*a*d1*d1+a*a*fq*fq-d*a*d1*fq+a1*a1*f*f-2*a*fq*a1*f-d*d1*a1*f+a1*d*d*fq;
	double z1 = e1*d*d*a1-fq*d1*a*b-2*a*fq*a1*e-f*a1*b1*d+2*d1*b1*a*f+2*e1*fq*a*a+d1*d1*a*e-e1*d1*a*d-2*a*e1*a1*f-f*a1*d1*b+2*f*e*a1*a1-fq*b1*a*d-e*a1*d1*d+2*fq*b*a1*d;
	double z2 = e1*e1*a*a+2*c1*fq*a*a-e*a1*d1*b+fq*a1*b*b-e*a1*b1*d-fq*b1*a*b-2*a*e1*a1*e+2*d1*b1*a*e-c1*d1*a*d-2*a*c1*a1*f+b1*b1*a*f+2*e1*b*a1*d+e*e*a1*a1-c*a1*d1*d-e1*b1*a*d+2*f*c*a1*a1-f*a1*b1*b+c1*d*d*a1+d1*d1*a*c-e1*d1*a*b-2*a*fq*a1*c;
	double z3 = -2*a*a1*c*e1+e1*a1*b*b+2*c1*b*a1*d-c*a1*b1*d+b1*b1*a*e-e1*b1*a*b-2*a*c1*a1*e-e*a1*b1*b-c1*b1*a*d+2*e1*c1*a*a+2*e*c*a1*a1-c*a1*d1*b+2*d1*b1*a*c-c1*d1*a*b;
	double z4 = a*a*c1*c1-2*a*c1*a1*c+a1*a1*c*c-b*a*b1*c1-b*b1*a1*c+b*b*a1*c1+c*a*b1*b1;

	quartric[0] = z0;
	quartric[1] = z1;
	quartric[2] = z2;
	quartric[3] = z3;
	quartric[4] = z4;
}

bool hasAzero(double* z)
{
	if(z[0] == 0)
		return true;

	if(z[4] == 0){
		if(z[3] != 0){
			return true;
		}
		if(z[2] != 0){
			return (z[1]*z[1] - 4*z[2]*z[0]) >= 0;
		}

		return z[1] != 0;
	}

	double a = z[3]/z[4];
	double b = z[2]/z[4];
	double c = z[1]/z[4];
	double d = z[0]/z[4];
	double p = (8*b - 3*a*a) / 8.0;
	double q = (a*a*a - 4*a*b + 8*c) / 8.0;
	double r = (-3*a*a*a*a + 256*d - 64*c*a + 16*a*a*b) / 256.0;

	double descrim = 256*r*r*r - 128*p*p*r*r + 144*p*q*q*r - 27*q*q*q*q + 16*p*p*p*p*r - 4*p*p*p*q*q;
	double P = 8*p;
	double D = 64*r - 16*p*p;

	return (descrim < 0 || (descrim > 0 && P < 0 && D < 0) || (descrim == 0 &&  (D != 0 || P <= 0) ) );
}

bool isInEllipse(double*e, double* p)
{
	double x = p[0] - e[0];
	double y = p[1] - e[1];
	double x_ = x*cos(e[4]) - y*sin(e[4]);
	double y_ = x*sin(e[4]) + y*cos(e[4]);

	double a = (x_*x_) / (e[2]*e[2]);
	double b = (y_*y_) / (e[3]*e[3]);

	if(a+b < 1)
		return true;
	else
		return false;
}

bool colEllipsetoEllipse(double* e1, double* e2) // p[0], p[1], r[0], r[1], front
{
	if(fabs(remainder((e1[4]+e2[4]), (PI/2.0))) < 0.01){
		for(int i=0; i<16; i++){
			double p[2];
			p[0] = e2[2] * cos(2*PI/16.0*i);
			p[1] = e2[3] * sin(2*PI/16.0*i);
			double cos_ = cos(e2[4]);
			double sin_ = sin(e2[4]);
			double p0 = cos_ * p[0] - sin_*p[1];
			double p1 = sin_ * p[0] + cos_*p[1];
			p[0] = p0 + e2[0];
			p[1] = p1 + e2[1];
			if(isInEllipse(e1,p)){
				return true;
			}
		}
	}

	double e1_bv[6];
	double e2_bv[6];

	bivariateForm(e1, e1_bv);
	bivariateForm(e2, e2_bv);

	double quartric[5];
	getQuartic(e1_bv, e2_bv, quartric);

	if(hasAzero(quartric))
		return true;
	else
		return false;
}

