#pragma once
#ifndef ERRT_H
#define ERRT_H
#include "model.h"
#include <vector>
class ERRTNode;
typedef std::vector<Coord> CoordVector;
typedef std::vector<Robot> RobotVector;
typedef std::vector<ERRTNode> TNodeVector;

class ERRTNode
{
public:
	ERRTNode() : ID(0), parentID(-1) {}
	ERRTNode(Coord p) : pos(p), ID(0), parentID(-1) {}
	ERRTNode(Coord p, int id) : pos(p), ID(id), parentID(-1) {}
	ERRTNode(Coord p, int id, int parent) : pos(p), ID(id), parentID(parent) {}
	~ERRTNode() {}
	int ID, parentID;
	Coord pos;
};

class ERRTTree
{
public:
	ERRTTree() {}
	ERRTTree(Coord ele) { nodes.push_back(ERRTNode(ele)); }
	ERRTTree(ERRTNode ele) { nodes.push_back(ele); }
	~ERRTTree() { nodes.clear(); }
	void addNewTNode(ERRTNode ele) { nodes.push_back(ele); }
	TNodeVector nodes;
};

class ERRTPlanner
{
public:
	ERRTPlanner() {}
	ERRTPlanner(Robot robot, Coord goal, RobotVector obs, CoordVector waypoint) : _robot(robot), _end(goal), _obsVec(obs), _waypoint(waypoint) { _start = _robot.pos(); _robotSize = _robot.size(); }
	~ERRTPlanner() {}
	bool generateRandP(Coord& randP);
	bool findNearestP(int& parentID, Coord& randP);
	bool generateNewP(Coord& nearestP, Coord& randP, Coord& newP);
	bool checkCollision(Coord& nearestP, Coord& newP);
	bool checkNewP2Goal(Coord& newP);
	void findPath(TNodeVector& path);
	void smoothPath(TNodeVector& path, CoordVector& sPath);
	bool findERRTPath(CoordVector& trajVec, CoordVector& origVec);	
	void extendRRTPath(CoordVector& formerPath, CoordVector& latterPath);
private:
	ERRTTree _errt;
	Robot _robot;
	Coord _start, _end;
	double _robotSize;
	RobotVector _obsVec;
	CoordVector _waypoint;
};


#endif // !ERRT_H
