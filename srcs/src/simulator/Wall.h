#ifndef WALL_H
#define WALL_H

#include <vector>

class Edge{
	protected:
		double _st[2];
		double _ed[2];
		double _p[2];
		double _u[2];
		double _q[2];
		double _n[2];
		double _l;
	public:
		Edge();
		Edge(double* st, double* p, double l, double* n);
		~Edge();

		void setSt(double* st) {_st[0] = st[0]; _st[1] = st[1];};
		void setEd(double* ed) {_ed[0] = ed[0]; _ed[1] = ed[1];};
		void setU(double* u) {_u[0] = u[0]; _u[1] = u[1];};
		void setP(double* p) {_p[0] = p[0]; _p[1] = p[1];};
		void setL(double l) {_l = l;};
		void setNormal(double* n) {_n[0] = n[0]; _n[1] = n[1];};

		double* getSt() {return _st;};
		double* getEd() {return _ed;};
		double* getU() {return _u;};
		double* getP() {return _p;};
		double getL() {return _l;};
		double* getNormal() {return _n;};
};

class Wall{
	protected:
		std::vector<Edge*> edges;
		double _p[2];
		double _w;
		double _h;

	public:
		Wall();
		Wall(double* p, double w, double h);
		~Wall();

		std::vector<Edge*> getEdges() {return edges;};

		void setP(double p_x, double p_y) {_p[0] = p_x; _p[1] = p_y;};
		void setW(double w) {_w = w;};
		void setH(double h) {_h = h;};

		double* getP() {return _p;};
		double getW() {return _w;};
		double getH() {return _h;};
};


#endif
