#pragma once
#ifndef MODEL_H
#define MODEL_H
#include <cmath>
#define ROBOTSIZE 30
class Coord			//坐标类
{
public:
	Coord() {}
	~Coord() {}
	Coord(double x, double y) : _x(x), _y(y) { }
	inline double x() const { return _x; }
	inline double y() const { return _y; }
	Coord& operator = (const Coord& ele) {
		if (this != &ele) {
			this->_x = ele.x();
			this->_y = ele.y();
		}
		return *this;
	}
	double dist(Coord ele) { return sqrt((_x - ele.x())*(_x - ele.x()) + (_y - ele.y())*(_y - ele.y())); }
	double getX(void) { return _x; }
	double getY(void) { return _y; }
	void setX(double x) { _x = x; }
	void setY(double y) { _y = y; }
private:
	double _x, _y;
};

class Robot		//机器人类
{
public:
	Robot() {}
	~Robot() {}
	Robot(Coord pos, double velx = 0, double vely = 0, double orientation = 0, double rotationVel = 0, double size = ROBOTSIZE) : 
		_pos(pos), _velx(velx), _vely(vely), _orientation(orientation), _rotationVel(rotationVel), _size(size) { }
	Robot(double x, double y, double velx = 0, double vely = 0, double orientation = 0, double rotationVel = 0, double size = ROBOTSIZE) :
		_pos(Coord(x,y)), _velx(velx), _vely(vely), _orientation(orientation), _rotationVel(rotationVel), _size(size) { }
	inline Coord pos() const { return _pos; }
	inline double size() const { return _size; }
	inline double velx() const { return _velx; }
	inline double vely() const { return _vely; }
	inline double orientation() const { return _orientation; }
	inline double rotationVel() const { return _rotationVel; }
	
	void setPos(Coord pos) { _pos = pos; }
	void setVel(double velx, double vely = 0) { _velx = velx; _vely = vely; }
	void setOrientation(double orientation) { _orientation = orientation; }
	void setRotationVel(double rotationVel) { _rotationVel = rotationVel; }
	void setRobotParam(Coord pos, double velx = 0, double vely = 0, double orientation = 0, double rotationVel = 0, double size = ROBOTSIZE) {
		_pos = pos;
		_velx = velx;
		_vely = vely;
		_orientation = orientation;
		_rotationVel = rotationVel;
		_size = size;
	}
	void setRobotParam(double x, double y, double velx = 0, double vely = 0, double orientation = 0, double rotationVel = 0, double size = ROBOTSIZE) {
		_pos = Coord(x, y);
		_velx = velx;
		_vely = vely;
		_orientation = orientation;
		_rotationVel = rotationVel;
		_size = size;
	}
	bool isCollision(Robot ele) { return _pos.dist(ele.pos()) < (_size + ele.size()); }
	bool isCollision(Coord pa, Coord pb) {
		return abs((pa.y()-pb.y())*_pos.x()-(pa.x()-pb.x())*_pos.y()+pa.x()*pb.y()-pa.y()*pb.x())/pa.dist(pb)<_size;
	}
private:
	Coord _pos;
	double _size;
	double _velx;
	double _vely;
	double _orientation;
	double _rotationVel;
};
#endif // !MODEL_H