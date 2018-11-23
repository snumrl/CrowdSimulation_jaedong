#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>
#include <ctime>

#include "Obstacle.h"

using namespace std;

Obstacle::Obstacle()
{
}

Obstacle::Obstacle(double* r, double* p)
{
	_r[0] = r[0];
	_r[1] = r[1];

	_p[0] = p[0];
	_p[1] = p[1];
}

Obstacle::~Obstacle()
{

}

void Obstacle::setFront()
{
	srand((unsigned int)time(0));
	int rand__ = rand()%20;
	double rand_ = rand__ - 10.0;
	rand_ /= 10.0;

	_front = rand_ * 3.14;
}

double Obstacle::getFront()
{
	return _front;
}
void Obstacle::getData(double* _data)
{
	_data[0] = _p[0];
	_data[1] = _p[1];
	_data[2] = _r[0];
	_data[3] = _r[1];
	_data[4] = _front;
}
