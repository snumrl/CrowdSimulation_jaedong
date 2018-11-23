#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <omp.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string.h>

#include "Agent.h"
#include "mMath.h"

using namespace std;

#define PI 3.14159265

Agent::Agent()
{
	_r[0] = 0.3;
	_r[1] = 0.3;

	_v[0] = 0.0;
	_v[1] = 0.0;

	_v_sim[0] = 0.0;
	_v_sim[1] = 0.0;

	setShape();
	visionReset();
	setVisionOffset();
}

Agent::Agent(double* r)
{
	_r[0] = r[0];
	_r[1] = r[1];

	_v[0] = 0.0;
	_v[1] = 0.0;

	_v_sim[0] = 0.0;
	_v_sim[1] = 0.0;

	setShape();
	visionReset();
	setVisionOffset();
}

Agent::Agent(double* r, double* p)
{
	_r[0] = r[0];
	_r[1] = r[1];

	_p[0] = p[0];
	_p[1] = p[1];

	_v[0] = 0.0;
	_v[1] = 0.0;

	_v_sim[0] = 0.0;
	_v_sim[1] = 0.0;

	setShape();
	visionReset();
	setVisionOffset();
}

Agent::~Agent()
{
}

void Agent::getData(double* _data)
{
	_data[0] = _p[0];
	_data[1] = _p[1];
	_data[2] = _r[0];
	_data[3] = _r[1];
	_data[4] = _front;
	_data[5] = _q_x[0];
	_data[6] = _q_x[1];
	_data[7] = _q_y[0];
	_data[8] = _q_y[1];

	return;
}

void Agent::getRenderData(double* r_data)
{
	// r, p, d, front, color
	r_data[0] = _r[0];
	r_data[1] = _r[1];
	r_data[2] = _p[0];
	r_data[3] = _p[1];
	r_data[4] = _d[0];
	r_data[5] = _d[1];
	r_data[6] = _front;
	r_data[7] = _color[0];
	r_data[8] = _color[1];
	r_data[9] = _color[2];
}

void Agent::getBodyState(double* b_data)
{
	double width = 48.0;
	double height = 32.0;
	double len = sqrt(width*width + height*height)*1.0;

	double scale_p[2];
	scale_p[0] = _p[0] / len;
	scale_p[1] = _p[1] / len;

	double scale_d[2];
	scale_d[0] = _d[0] / len;
	scale_d[1] = _d[1] / len;

	double dist = Dist(scale_p, scale_d);

	double pd[2];
	pd[0] = (scale_d[0] - scale_p[0]) / dist;
	pd[1] = (scale_d[1] - scale_p[1]) / dist;
	double Inner = InnerProduct2d(_q_y, pd);
	double Cross = CrossProduct2d(_q_y, pd);

	//goal
	b_data[0] = Inner;
	b_data[1] = Cross;
	b_data[2] = dist;
	//currunt move
	b_data[3] = _v_sim[0];
	b_data[4] = _v_sim[1];
	b_data[5] = _w_sim;
	//shape
	b_data[6] = (_shape[0] - 0.3)/0.7; //scale to 0~1
	b_data[7] = (_shape[1] - 0.3)/0.7; //scale to 0~1
	b_data[8] = (_shape[2] - 0.3)/0.7; //scale to 0~1
	b_data[9] = (_shape[3] - 0.3)/0.7; //scale to 0~1
	b_data[10] = (_shape[4] - 0.3)/0.7; //scale to 0~1
	b_data[11] = (_shape[5] - 0.3)/0.7; //scale to 0~1
	b_data[12] = (_shape[6] - 0.3)/0.7; //scale to 0~1
	b_data[13] = (_shape[7] - 0.3)/0.7; //scale to 0~1
}

void Agent::visionReset()
{
	for(int i=0; i<_vision_ray_num; i++){
		_vision[i] = _vision_depth;
	}
}

void Agent::setVisionOffset()
{
	double vision_st_ang;
	vision_st_ang = 90.0 - _vision_range/2.0;

	#pragma omp parallel for
	for(int i=0; i<_vision_ray_num; i++){
		double cur_ang = vision_st_ang + (i+0.5)*_vision_interval;
		double cur_rad = AngleToRadian(cur_ang);
		double cur_offset = 1.0 / sqrt( cos(cur_rad)*cos(cur_rad) / (_r[0]*_r[0])  + sin(cur_rad)*sin(cur_rad) / (_r[1]*_r[1]));
		_vision_offset[i] = cur_offset;
	}
}

void Agent::setShape()
{
	#pragma omp parallel for
	for(int i=0; i<8; i++){
		double cur_ang = 45.0*i;
		double cur_rad = AngleToRadian(cur_ang);
		double cur_offset = 1.0 / sqrt( cos(cur_rad)*cos(cur_rad) / (_r[0]*_r[0])  + sin(cur_rad)*sin(cur_rad) / (_r[1]*_r[1]));
		_shape[i] = cur_offset;
	}
}

double* Agent::getVisionOffset()
{
	return _vision_offset;
}

bool Agent::isVisible(double* d)
{
	double other_p[2];
	other_p[0] = d[0];
	other_p[1] = d[1];

	double dist = Dist(_p, other_p);
	double r_max = _r[0] > _r[1] ? _r[0] : _r[1];
	double other_r_max = d[2] > d[3] ? d[2] : d[3];

	if(dist <= _vision_depth + r_max + other_r_max)
		return true;

	return false;
}

void Agent::setVision(double* _data)
{
	if(!isVisible(_data))
		return;

	#pragma omp parallel for
	for(int i=0; i<_vision_ray_num; i++){
		double st_pts[2];
		st_pts[0] = _p[0];
		st_pts[1] = _p[1];
		double cur_ang = 90.0 - _vision_range/2.0 + (i+0.5)*_vision_interval;
		double cur_rad = AngleToRadian(cur_ang)+(_front - PI/2.0);

		double new_depth = _vision_depth;
		if(_data[2] == _data[3]){ // circle
			double angle_coord[2];
			RadianToCoor(cur_rad, angle_coord);
			double other_p[2];
			other_p[0] = _data[0];
			other_p[1] = _data[1];
			double ray = RayToSphereDistance(st_pts, other_p, angle_coord, _data[2]);
			ray -= _vision_offset[i];
			if(ray >= 0.0 && ray <= _vision_depth){
				new_depth = ray;
				if(new_depth < _vision[i]){
					_vision[i] = new_depth;

				}
			}
		}
		else{ // ellipse
			double cur_pts[2];
			double ed_pts[2];
			double cur_len = _vision_depth + _vision_offset[i];
			ed_pts[0] = st_pts[0] + cur_len * cos(cur_rad);
			ed_pts[1] = st_pts[1] + cur_len * sin(cur_rad);
			if(LineEllipseIntersection(st_pts, ed_pts, _data, cur_pts)){
				new_depth = Dist(st_pts, cur_pts) - _vision_offset[i];
				if(new_depth < _vision[i]){
					_vision[i] = new_depth;

				}
			}
		}
	}
}

// void Agent::setVisionWall(double* _data)
// {
// 	if(!isVisibleWall(_data))
// 		return;
// }

bool Agent::isCollidable(double* d)
{
	double other_p[2];
	other_p[0] = d[0];
	other_p[1] = d[1];

	double dist = Dist(_p, other_p);
	double r_max = _r[0] > _r[1] ? _r[0] : _r[1];
	double other_r_max = d[2] > d[3] ? d[2] : d[3];

	if(dist <= r_max + other_r_max + 0.1)
		return true;

	return false;
}

bool Agent::colCheck(double* _data)
{
	if(!isCollidable(_data))
		return false;

	double e1[5];
	e1[0] = _p[0];
	e1[1] = _p[1];
	e1[2] = _r[0];
	e1[3] = _r[1];
	e1[4] = _front-0.5*PI;

	double e2[5];
	e2[0] = _data[0];
	e2[1] = _data[1];
	e2[2] = _data[2];
	e2[3] = _data[3];
	e2[4] = _data[4]-0.5*PI;

	if(colEllipsetoEllipse(e1, e2))
		return true;
	else
		return false;
}

double* Agent::getVision()
{
	return _vision;
}

void Agent::setAction(double w, double a_x, double a_y)
{
	_a[0] = a_x;
	_a[1] = a_y;

	_a_sim[0] = 1.5 * _a[0];
	_a_sim[1] = 1.5 * _a[1];

	_v[0] += _a_sim[0] * _time_step;
	_v[1] += _a_sim[1] * _time_step;

	_v_sim[0] = mClip(-1.0, 1.0, _v[0]);
	_v_sim[1] = mClip(-0.5, 2.0, _v[1]);

	_w = w;
	_w_sim = 1.5*w;
	_w_sim = mClip(-1.5, 1.5, _w_sim);

	_front += _w_sim * _time_step;

	if(_front > PI)
		_front -= PI*2;
	if(_front < -PI)
		_front += PI*2;

	RadianToCoor(_front-0.5*PI, _q_x);
	RadianToCoor(_front, _q_y);
}

void Agent::Action()
{
	if(_stop)
		return;

	_p_prev[0] = _p[0];
	_p_prev[1] = _p[1];

	_p[0] += _v_sim[0] * _q_x[0] * _time_step;
	_p[1] += _v_sim[0] * _q_x[1] * _time_step;

	_p[0] += _v_sim[1] * _q_y[0] * _time_step;
	_p[1] += _v_sim[1] * _q_y[1] * _time_step;

	_v[0] = _v_sim[0];
	_v[1] = _v_sim[1];
	_w = _w_sim;
}

void Agent::Revert(double* p, bool col)
{
	double vec[2];
	if(col){
		vec[0] = _p[0] - p[0];
		vec[1] = _p[1] - p[1];

		_p[0] = _p_prev[0] + 0.05*vec[0];
		_p[1] = _p_prev[1] + 0.05*vec[1];
	}
	else{
		vec[0] = _p_prev[0] - _p[0];
		vec[1] = _p_prev[1] - _p[1];

		_p[0] = _p_prev[0] + 0.05*vec[0];
		_p[1] = _p_prev[1] + 0.05*vec[1];
	}
}



