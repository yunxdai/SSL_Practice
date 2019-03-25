#include "ERRT.h"
#include <ctime>
#define WIDTH 450
#define LENGTH 600
#define STEP 10
#define PGOAL 0.2
#define PWAYPOINT 0.3

bool ERRTPlanner::generateRandP(Coord& randP) {
	double p;
	srand((unsigned)time(NULL));
	p = rand() / double(RAND_MAX);		//����0-1֮��������
	if (p < PGOAL) {
		randP = _end;
		return true;
	}
	else if (p < PGOAL + PWAYPOINT) {
		int len, index;
		len = _waypoint.size();
		index = rand() % len;
		randP = _waypoint[index];
		return true;
	}
	else {
		randP = Coord(rand() % LENGTH, rand() % WIDTH);
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
		if (_obsVec[i].isCollision(newRobot) || _obsVec[i].isCollision(nearestP, newP))	return true;
	}
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
	int i, id, len;
	bool flag;
	id = 0;
	len = path.size();
	sPath.push_back(path[id].pos);
	while (id != len - 1){
		flag = false;
		for (i = len - 1; i > id + 1; i--) {
			if(checkCollision(path[id].pos, path[i].pos))	continue;
			flag = true;
			break;
		}
		if (flag) {
			path[id].parentID = i;
			id = i;
		}
		else {
			id++;
		}
		sPath.push_back(path[id].pos);
	}
}

bool ERRTPlanner::findERRTPath(CoordVector& trajVec) {
	_errt.addNewTNode(ERRTNode(_start));
	bool goalSuccessFlag = false;
	int nodeNum = 1;
	Coord randP, nearestP, newP;
	int parentID;
	while (!goalSuccessFlag && nodeNum < 1000) {
		if (!generateRandP(randP))	continue;	//��������㣬�ɹ��򷵻���
		if(!findNearestP(parentID, randP))	continue;	//�ҵ�����ڵ��ID���ɹ��򷵻���
		nearestP = _errt.nodes[parentID].pos;	//����ڵ������
		if (!generateNewP(nearestP, randP, newP))	continue;	//�����µ����꣬�ɹ��򷵻���
		if (checkCollision(nearestP, newP))	continue;	//����µ�����������������������
		if (checkNewP2Goal(newP)) {				//�ж��µ��Ƿ񵽴�Ŀ��㣬���ﷵ����
			_errt.addNewTNode(ERRTNode(_end, nodeNum, parentID));
			goalSuccessFlag = true;
			break;
		}
		else {
			_errt.addNewTNode(ERRTNode(newP, nodeNum, parentID));
		}
		nodeNum++;
	}
	if (goalSuccessFlag) {
		TNodeVector path;
		CoordVector sPath;
		int i, len;
		trajVec.swap(CoordVector(trajVec));
		findPath(path);
		smoothPath(path, sPath);
		len = sPath.size();
		for (i = len - 1; i >= 0; i--)
			trajVec.push_back(sPath[i]);
		return true;
	}
	else
		return false;
}