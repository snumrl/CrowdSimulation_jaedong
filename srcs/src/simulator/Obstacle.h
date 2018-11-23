#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle{
	protected:
		double _p[2];
		double _r[2];
		double _front = 0.0;

		int _id;

	public:
		Obstacle();
		Obstacle(double* r, double* p);
		~Obstacle();

		void setP(double x, double y) {_p[0] = x; _p[1] = y;};
		void setR(double rx, double ry) {_r[0] = rx; _r[1] = ry;};
		void setFront();
		void setId (int id) {_id = id;}

		double* getP() {return _p;};
		double* getR() {return _r;};
		double getFront();
		int getId () const {return _id;}
		void getData(double* _data);
};

#endif
