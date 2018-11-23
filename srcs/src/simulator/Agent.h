#ifndef AGENT_H
#define AGENT_H

class Agent{
	protected:
		int _id;
		double _r[2];
		double _shape[8];
		double _p[2];
		double _p_prev[2];
		double _d[2];
		double _q_x[2];
		double _q_y[2];
		double _front = 0.0;
		double _color[3];

		double _vision[45];
		double _vision_offset[45];
		double _vision_depth = 10.0;
		double _vision_interval = 8.0;
		double _vision_range = 360.0;
		int _vision_ray_num = 45; // _vision_range / _vision_interval + 1

		double _v[2];
		double _v_sim[2];
		double _a[2];
		double _a_sim[2];
		double _w;
		double _w_sim;
		double _time_step = 0.1;
		bool _col = false;
		bool _stop = false;

	public:
		Agent();
		Agent(double* r);
		Agent(double* r, double* p);
		~Agent();

		void setAction(double w, double v_x, double v_y);
		void Action();
		void Revert(double* p, bool col);

		// void setVisionWall(double* d);
		// bool isVisibleWall(double* d);

		bool isVisible(double* d);
		bool isCollidable(double* d);
		bool colCheck(double* _data);

		void visionReset();
		void setVision(double* d);
		void setVisionOffset();
		void setShape();
		void setVisionDepth(double _vd) {_vision_depth = _vd;};
		void setVisionRayNum(int _r) {_vision_ray_num = _r;};

		double* getVision();
		double* getVisionOffset();
		double getVisionDepth() {return _vision_depth;};
		int getVisionRayNum() {return _vision_ray_num;};

		void getData(double* _data);
		void getRenderData(double* r_data);
		void getBodyState(double* b_data);
		void getSensorState(double* s_data);

		void setP(double x, double y) {_p[0] = x; _p[1] = y;};
		void setPprev(double x, double y) {_p_prev[0] = x; _p_prev[1] = y;};
		void setQx(double x, double y) {_q_x[0] = x; _q_x[1] = y;};
		void setQy(double x, double y) {_q_y[0] = x; _q_y[1] = y;};
		void setD(double x, double y) {_d[0] = x; _d[1] = y;};
		void setR(double _rx, double _ry) {_r[0] = _rx; _r[1] = _ry;};
		void setV(double v_x, double v_y) {_v[0] = v_x; _v[1] = v_y;};
		void setVsim(double v_x, double v_y) {_v_sim[0] = v_x; _v_sim[1] = v_y;};
		void setA(double a_x, double a_y) {_a[0] = a_x; _a[1] = a_y;};
		void setAsim(double a_x, double a_y) {_a_sim[0] = a_x; _a_sim[1] = a_y;};
		void setW(double w) {_w = w;};
		void setWsim(double w) {_w_sim = w;};
		void setFront(double f) {_front = f;};
		void setColor(double r, double g, double b) {_color[0] = r; _color[1] = g; _color[2] = b;};
		void setCol(bool c) {_col = c;};
		void setId (int id) {_id = id;}

		double* getP() {return _p;};
		double* getPprev() {return _p_prev;};
		double* getQx() {return _q_x;};
		double* getQy() {return _q_y;};
		double* getD() {return _d;};
		double* getR() {return _r;};
		double* getV() {return _v;};
		double* getVsim() {return _v_sim;};
		double* getA() {return _a;};
		double* getAsim() {return _a_sim;};
		double getW() {return _w;};
		double getWsim() {return _w_sim;};
		double getFront() {return _front;};
		double* getColor() {return _color;}
		bool getCol() {return _col;};
		int getId () const {return _id;}
};


#endif
