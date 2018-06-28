#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle{
	protected:
		double _p[2];
		double _r = 0.5;

		int _id;

	public:
		Obstacle();
		~Obstacle();

		void Render();

		void setP(double x, double y) {_p[0] = x; _p[1] = y;};
		void setR(double r) {_r = r;};
		void setId (int id) {_id = id;}

		double* getP() {return _p;};
		double getR() {return _r;};
		int getId () const {return _id;}
};

#endif
