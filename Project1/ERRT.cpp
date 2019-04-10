#include "ERRT.h"
#include <iostream>
#include <ctime>
//#define WIDTH 450
//#define LENGTH 600
#define WIDTH 400
#define LENGTH 600
#define ORIGINAL_X -300
#define ORIGINAL_Y -200
#define STEP 3 * ROBOTSIZE
#define PGOAL 0.2
#define PWAYPOINT 0.3

bool ERRTPlanner::generateRandP(Coord& randP) {
	double p;
	p = rand() / double(RAND_MAX);		//产生0-1之间的随机数
	// std::cout << "p = " << p << std::endl;
	if (p < PGOAL) {
		randP = _end;
		return true;
	}
	else if (p < PGOAL + PWAYPOINT && !_waypoint.empty()) {
		int len, index;
		len = _waypoint.size();
		index = rand() % len;
		randP = _waypoint[index];
		return true;
	}
	else {
		randP = Coord(ORIGINAL_X + rand() % LENGTH, ORIGINAL_Y + rand() % WIDTH);
		return true;
	}
	return false;
}

bool ERRTPlanner::findNearestP(int& parentID, Coord& randP) {
	if (_errt.nodes.empty())	return false;
	double bestDist, distance;
	int i, bestIndex, len;
	bestIndex = 0;
	bestDist = _errt.nodes[0].pos.dist(randP);
	len = _errt.nodes.size();
	for (i = 1; i < len; i++) {
		distance = _errt.nodes[i].pos.dist(randP);
		if (distance < bestDist) {
			bestIndex = i;
			bestDist = distance;
		}
	}
	parentID = bestIndex;
	return true;
}

bool ERRTPlanner::generateNewP(Coord& nearestP, Coord& randP, Coord& newP) {
	double distance, deltaX, deltaY, k;
	distance = nearestP.dist(randP);
	k = STEP / distance;
	deltaX = randP.x() - nearestP.x();
	deltaY = randP.y() - nearestP.y();
	newP = Coord(nearestP.x() + k*deltaX, nearestP.y() + k*deltaY);
	return true;
}

bool ERRTPlanner::checkCollision(Coord& nearestP, Coord& newP) {
	int i, len;
	Robot newRobot = Robot(newP, 0, 0, 0);
	len = _obsVec.size();

	for (i = 0; i < len; i++) {
		if (_obsVec[i].isCollision(newRobot) || _obsVec[i].isCollision(nearestP, newP)) {
			std::cout << "Robot " << i << " is collision" << std::endl;
			std::cout << "Robot's position is x = " << _obsVec[i].pos().x() << " y = " << _obsVec[i].pos().y() << std::endl;
			return true;
		}
	}
//	std::cout << "No collision" << std::endl;
	return false;
}

bool ERRTPlanner::checkNewP2Goal(Coord& newP) {
	if (newP.dist(_end) < _robotSize)	return true;
	else	return false;
}

void ERRTPlanner::findPath(TNodeVector& path) {
	int id = _errt.nodes.size() - 1;
	ERRTNode Tnode;
	while (id >= 0)	{
		Tnode = _errt.nodes[id];
		path.push_back(Tnode);
		id = Tnode.parentID;
	}
}

void ERRTPlanner::smoothPath(TNodeVector& path, CoordVector& sPath) {
	int i = 0;
	int len = path.size();
	sPath.push_back(path[i].pos);
	while (true) {
		auto node1 = path[i];
		for (int j = len - 1; j >= i + 1; j--) {
			auto node3 = path[j];
			if (checkCollision(node1.pos, node3.pos)) {
				continue;
			}
			else {

				sPath.push_back(path[j].pos);
				// path[j].parentID = i;
				i = j;
				break;
			}
		}

		if (i == len - 2) {
			sPath.push_back(path[i+1].pos);
			break;
		}
		if (i == len - 1) {
			break;
		}
		i++;
	}
}
void ERRTPlanner::extendRRTPath(CoordVector& formerPath, CoordVector& latterPath) {
	double insertThreshold = 20 * ROBOTSIZE;
	double insertSect = 10 * ROBOTSIZE ;
	for (int i = 0; i < formerPath.size() - 1; i++) {
		Coord firstNode = formerPath[i];
		Coord nextNode = formerPath[i + 1];
		latterPath.push_back(firstNode);
		double distInNode = firstNode.dist(nextNode);
		double thetaInNode = atan2(nextNode.getY() - firstNode.getY(), nextNode.getX() - firstNode.getX());
		if (distInNode > insertThreshold) {
			Coord insertNode = firstNode;
			while (true) {
				if (insertNode.dist(nextNode) < insertSect) {
					break;
				}
				else {
					insertNode.setX(insertNode.getX() + insertSect * cos(thetaInNode));
					insertNode.setY(insertNode.getY() + insertSect * sin(thetaInNode));
					latterPath.push_back(insertNode);
				}
			}
			latterPath.push_back(nextNode);
		}
		else {
			latterPath.push_back(nextNode);
		}
	}
}
bool ERRTPlanner::findERRTPath(CoordVector& finalVec, CoordVector& origVec) {
	srand((unsigned)time(NULL));
	_errt.addNewTNode(ERRTNode(_start));
	bool goalSuccessFlag = false;
	int nodeNum = 1;
	Coord randP, nearestP, newP;
	CoordVector randPVec;
	int parentID;
	while (!goalSuccessFlag && nodeNum < 1000) {
		if (!generateRandP(randP))	continue;	//产生随机点，成功则返回真
		if(!findNearestP(parentID, randP))	continue;	//找到最近邻点的ID，成功则返回真
		nearestP = _errt.nodes[parentID].pos;	//最近邻点的坐标
		if (!generateNewP(nearestP, randP, newP))	continue;	//产生新点坐标，成功则返回真
		if (checkCollision(nearestP, newP))	continue;	//检测新点坐标有无碰，有碰返回真
		if (checkNewP2Goal(newP)) {				//判断新点是否到达目标点，到达返回真
			_errt.addNewTNode(ERRTNode(_end, nodeNum, parentID));
			goalSuccessFlag = true;
			break;
		}
		else {
			_errt.addNewTNode(ERRTNode(newP, nodeNum, parentID));
		}
		randPVec.push_back(randP);
		nodeNum++;
	}
	// std::cout << "wait" << std::endl;
	CoordVector trajVec;
	if (goalSuccessFlag) {
		TNodeVector path;
		CoordVector sPath;
		int i, len1, len2;
		finalVec.clear();
		finalVec.shrink_to_fit();
		origVec.clear();
		origVec.shrink_to_fit();
		// trajVec.swap(CoordVector());
		findPath(path);
		smoothPath(path, sPath);
		len1 = sPath.size();
		if (len1 == 1) {
			return false;
		}
		len2 = path.size();
		for (i = len1 - 1; i >= 0; i--)
			trajVec.push_back(sPath[i]);
			// trajVec.push_back(path[i].pos);
		for (i = len2 - 1; i >= 0; i--) {
			origVec.push_back(path[i].pos);
		}
		// finalVec = trajVec;
		extendRRTPath(trajVec, finalVec);
		return true;
	}
	else
		return false;
}