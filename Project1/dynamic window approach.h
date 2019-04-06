#pragma once
#ifndef DYNAMIC_WINDOW_APPROACH_H
#define DYNAMIC_WINDOW_APPROACH_H
#include "model.h"
#include <vector>	
//参数待定
#define MAXVEL 300
#define MAXW 200
#define MAXACC 20
#define MAXBETA 20
#define RESOLUTIONVEL 2
#define RESOLUTIONW 2
#define SAMPLETIME 0.2
#define PREDICTTIME 0.5
#define ALPHA 0.2
#define BETA 0.3
#define GAMMA 0.5

typedef std::vector<Coord> CoordVector;
typedef std::vector<Robot> RobotVector;
typedef std::vector<int> VelVector;
typedef std::vector<int> WVector;
typedef std::vector<double> ThetaVector;
typedef std::vector<double> CostVector;
typedef std::vector<CoordVector> TrajVector;

class DWAPlanning
{
public:
	DWAPlanning(Robot robot, RobotVector obsVec, Coord goal, 
		int minVel = -MAXVEL, int maxVel = MAXVEL, int minW = -MAXW, int maxW = MAXW,
		int maxACC = MAXACC, int maxBeta = MAXBETA, int resolutionVel = RESOLUTIONVEL,
		int resolutionW = RESOLUTIONW, double sampleTime = SAMPLETIME, 
		double predictTime = PREDICTTIME) :
		_robot(robot), _obsVec(obsVec), _goal(goal), _minVel(minVel), _maxVel(maxVel), 
		_minW(minW), _maxW(maxW), _maxAcc(maxACC), _maxBeta(maxBeta),
		_resolutionVel(resolutionVel), _resolutionW(resolutionW), _sampleTime(sampleTime),
		_predictTime(predictTime) {}
	~DWAPlanning() {}
	void updateRobotPos(Coord pos) { _robot.setPos(pos); }
	void updateObsVec(RobotVector obsVec) { _obsVec.swap(obsVec); }
	bool generateMotion(int& vx, int& vy, int& w);
private:
	inline double min(double a, double b) { return a < b ? a : b; }
	inline double max(double a, double b) { return a > b ? a : b; }
	void calcDynamicWindow(int& vmin, int& vmax, int& wmin, int& wmax);
	void getPredictTraj(int v, int w, double theta, CoordVector& predictTraj);
	double calcTrajCost(double headingCost, double distCost, double velCost);
	double calHeadingCost(Coord ele);
	double calDistCost(CoordVector traj); 
	double calVelCost(int v, double theta);
	Robot _robot;
	RobotVector _obsVec;
	Coord _goal;
	int _minVel, _maxVel, _minW, _maxW, _maxAcc, _maxBeta;	//速度、角速度、最大加速度、最大角加速度
	int _resolutionVel, _resolutionW;
	double _sampleTime, _predictTime;
};

#endif // !DYNAMIC_WINDOW_APPROACH_H
