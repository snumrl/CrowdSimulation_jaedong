#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>

#include "Obstacle.h"

using namespace std;

Obstacle::Obstacle()
{
}

Obstacle::~Obstacle()
{
}

void Obstacle::Render()
{
	glColor3f(0.8, 0.8, 0.8);

	glPushMatrix();
	glTranslatef(_p[0], _p[1], 0.0);
	GLUquadric *sphere;
	sphere = gluNewQuadric();
	gluSphere(sphere, _r, 50, 10);
	glPopMatrix();
}

