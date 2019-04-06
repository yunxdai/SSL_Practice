#include "dynamic window approach.h"
#define PI 3.1415926

void DWAPlanning::calcDynamicWindow(int& vmin, int& vmax, int& wmin, int& wmax) {
	vmin = (int)max(_minVel, _robot.vel() - _maxAcc*_sampleTime);
	vmax = (int)min(_maxVel, _robot.vel() + _maxAcc*_sampleTime);
	wmin = (int)max(_minW, _robot.rotationVel() - _maxBeta*_sampleTime);
	wmax = (int)min(_maxW, _robot.rotationVel() + _maxBeta*_sampleTime);
}

void DWAPlanning::getPredictTraj(int v, int w, double theta, CoordVector& predictTraj) {
	double x = _robot.pos().x();
	double y = _robot.pos().y();
	double alpha = _robot.orientation() + theta;
	double t, dx, dy;
	for (t = _sampleTime; t <= _predictTime; t += _sampleTime) {
		x += v*cos(alpha)*_sampleTime;
		y += v*sin(alpha)*_sampleTime;
		alpha += w*_sampleTime;
		predictTraj.push_back(Coord(x, y, alpha));
	}
}

double DWAPlanning::calHeadingCost(Coord ele) {
	return ele.dist(_goal);
}

double DWAPlanning::calDistCost(CoordVector traj) {
	double dist, minDist = 999999;
	int i, j, trajVecLen = traj.size(), obsVecLen = _obsVec.size();
	for (i = 0; i < trajVecLen; i++)
		for (j = 0; j < obsVecLen; j++) {
			dist = traj[i].dist(_goal);
			if (dist < minDist)	minDist = dist;
		}
	return 1.0 / minDist;
}

double DWAPlanning::calVelCost(int v, double theta) {
	return 1-v*1.0/_maxVel; 
}


double DWAPlanning::calcTrajCost(double headingCost, double distCost, double velCost) {
	double cost;
	cost = ALPHA*headingCost + BETA*distCost + GAMMA*velCost;
	return cost;
}

bool DWAPlanning::generateMotion(int& vx, int& vy, int& w) {
	VelVector vVec;
	WVector wVec;
	ThetaVector thetaVec;
	CostVector headingCostVec, distCostVec, velCostVec;
	int vmin, vmax, wmin, wmax;
	int tempV, tempW, tempTheta, bestV, bestW, bestTheta;
	double tempCost, bestCost = 999999999;
	double theta_min = -PI/6, theta_max = PI/6;
	calcDynamicWindow(vmin, vmax, wmin, wmax);
	for (tempV = vmin; tempV <= vmax; tempV += _resolutionVel)
		for (tempW = wmin; tempW <= wmax; tempW += _resolutionW)
			for (tempTheta = theta_min; tempTheta <= theta_max; tempTheta += PI/36){
				vVec.push_back(tempV);
				wVec.push_back(tempW);
				thetaVec.push_back(tempTheta);
				CoordVector predictTraj;
				getPredictTraj(tempV, tempW, tempTheta, predictTraj);
				headingCostVec.push_back(calHeadingCost(predictTraj[predictTraj.size() - 1]));
				distCostVec.push_back(calDistCost(predictTraj));
				velCostVec.push_back(calVelCost(tempV, tempTheta));
				predictTraj.clear();
				predictTraj.shrink_to_fit();
			}
	int i, bestIndex;
	int len = vVec.size();
	double headingTotalCost = 0, distTotalCost = 0;
	for (i = 0; i < len; i++) {
		headingTotalCost += headingCostVec[i];
		distTotalCost += distCostVec[i];
	}
	for (i = 0; i < len; i++) {
		tempCost = calcTrajCost(headingCostVec[i]/headingTotalCost, distCostVec[i]/distTotalCost, velCostVec[i]);
		if (tempCost < bestCost) {
			bestCost = tempCost;
			bestIndex = i;
		}
	}
	bestV = vVec[bestIndex];
	bestW = wVec[bestIndex];
	bestTheta = thetaVec[bestIndex];
	vx = (int)(bestV * cos(_robot.orientation() + bestTheta));
	vy = (int)(bestV * sin(_robot.orientation() + bestTheta));
	w = bestW;
	vVec.clear();
	vVec.shrink_to_fit();
	wVec.clear();
	wVec.shrink_to_fit();
	thetaVec.clear();
	thetaVec.shrink_to_fit();
	headingCostVec.clear();
	headingCostVec.shrink_to_fit();
	distCostVec.clear();
	distCostVec.shrink_to_fit();
	velCostVec.clear();
	velCostVec.shrink_to_fit();
	return true;
}