#include <iostream>
#include <thread>
#include "serial_port.h"
#include <WinSock2.h>
#include <math.h>

#include<windows.h>
#include "vision_detection.pb.h"
#include "zss_debug.pb.h"
#include "grSim_Packet.pb.h"
#include "Motion_Info.h"
#include "model.h"
#include "ERRT.h"
#define pi 3.1415926
#define MyRobotBlue false
#define MyRobotID 3
#define SendRobotID 3
#define REAL true

using namespace std;
#pragma comment(lib, "libprotobuf.lib")
#pragma comment(lib, "ws2_32.lib")


bool recvFlag = false;
bool drawFlag = false;
bool proccessFlag = false;
bool pathChangeFlag = false;
bool sendFlag = false;
double global_vx, global_vy, global_w;
CoordVector errt_path;
CoordVector origin_path;
//Robot myRobot;
//RobotVector obsRobot;
//CoordVector path, sPath;
Coord goal(-207, -134);
bool Socket_Init(void)
{
	WSAData wsd; // 初始化信息
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {/*进行WinSocket的初始化,
												windows 初始化socket网络库，申请2，2的版本，windows socket编程必须先初始化。*/
		cout << "WSAStartup Error = " << WSAGetLastError() << endl;
		return false;
	}
	return true;
}
bool SocketRecvInit(SOCKET &soRecv, SOCKADDR_IN &si_local, SOCKADDR_IN &si_remote, char* &pszRecv, const char* &ADDR, const int RecvPort)

{
	//创建socket
	
	soRecv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);		//AF_INET 协议族:决定了要用ipv4地址（32位的）与端口号（16位的）的组合 //SOCK_DGRAM --  UDP类型，不保证数据接收的顺序，非可靠连接；
	if (soRecv == SOCKET_ERROR) {
		cout << "socket Error = " << WSAGetLastError() << endl;
		return false;
	}

	si_local.sin_family = AF_INET;
	si_local.sin_port = htons(RecvPort);
	si_local.sin_addr.s_addr = inet_addr(ADDR);
	if (::bind(soRecv, (SOCKADDR*)&si_local, sizeof(si_local)) == SOCKET_ERROR) {
		cout << "bind error = " << WSAGetLastError() << endl;
		return false;
	}

	if (pszRecv == NULL) {
		cout << "pszRecv new char Error " << endl;
		return false;
	}
	return true;

}
bool SocketSendInit(SOCKET &soSend, SOCKADDR_IN &si_local, const char* &ADDR, const int SendPort)
{
	soSend = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (soSend == SOCKET_ERROR) {
		cout << "socket Error = " << WSAGetLastError() << endl;
		return false;
	}
	
	si_local.sin_family = AF_INET;
	si_local.sin_port = htons(SendPort);
	si_local.sin_addr.s_addr = inet_addr(ADDR);
	return true;

}
void HandleRecvData(Vision_DetectionFrame &vision, char* &pszRecv, RobotVector &obstacle, Robot &myRobot) {
	/*
	Input: pszRecv, vision
	function: 处理读入的数据流
	Output: 障碍机器人信息，被控机器人信息
	*/
	// 坐标问题：读进来的坐标是grSim上显示坐标值的十倍
	vision.ParseFromArray(pszRecv, 4096);
	//cout << "blue car num = " << vision.robots_blue_size() << endl;
	//cout << "yellow car num = " << vision.robots_yellow_size() << endl;
	obstacle.clear();
	obstacle.shrink_to_fit();
	if (MyRobotBlue) {
		for (int i = 0; i < vision.robots_blue_size(); i++) {
			if (i > 8) {
				break;
			}
			else
			{
				auto car = vision.robots_blue(i);
				// cout << "blue " << car.robot_id() << ": " << car.x() / 10.0 << ' ' << -car.y() / 10.0 << ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
				if (car.robot_id() == MyRobotID) {
					myRobot.setRobotParam(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel());
					// cout << "my robot in blue: " << endl << "robot_x = " << car.x() << " robot_y = " << car.y() << endl;
				}
				else {
					obstacle.push_back(Robot(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
				}
			}
			
		}
		for (int i = 0; i < vision.robots_yellow_size(); i++) {
			if (i > 8) {
				break;
			}
			else
			{
				auto car = vision.robots_yellow(i);
				// cout << "yellow " << car.robot_id() << ": " << car.x() / 10.0<< ' ' << -car.y() / 10.0<< ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
				obstacle.push_back(Robot(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
			}
			
		}
	}
	else {
		for (int i = 0; i < vision.robots_yellow_size(); i++) {
			if (i > 8) {
				break;
			}
			else
			{
				auto car = vision.robots_yellow(i);
				// cout << "yellow " << car.robot_id() << ": " << car.x() / 10.0<< ' ' << -car.y() / 10.0<< ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
				if (car.robot_id() == MyRobotID) {
					myRobot.setRobotParam(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel());
					// cout << "my robot in yellow: " << endl << "robot_x = " << car.x() << " robot_y = " << car.y() << endl;
					// cout << "now is my robot in yellow: " << car.robot_id() << endl;
				}
				else {
					obstacle.push_back(Robot(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
				}
			}
			
		}
		for (int i = 0; i < vision.robots_blue_size(); i++) {
			if (i > 8) {
				break;
			}
			else
			{
				auto car = vision.robots_blue(i);
				// cout << "blue " << car.robot_id() << ": " << car.x()/10.0 << ' ' << -car.y()/10.0 << ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
				obstacle.push_back(Robot(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
			}
			
		}
	}
}
void recvFunc(SOCKET recvSocket, sockaddr_in remoteAddr) {
	char* bufRecv = new char[4096];
	int bufSize = 4096;
	int remoteAddrLen = sizeof(remoteAddr);
	int nRet;
	while (true)
			
		if (recvFlag) nRet = recvfrom(recvSocket, bufRecv, bufSize, 0, (SOCKADDR*)&remoteAddr, &remoteAddrLen);
}
void sendFunc() {
	// global_vx = global_vy = global_w = 0;
	CSerialPort robotSerialPort("COM6", 115200UL, 8, NOPARITY, ONESTOPBIT);
	while (REAL) {
		if (robotSerialPort.openComm())
			if (robotSerialPort.getReadyToSend()) {
				::cout << "Serial Initilized" << endl;
				break;
			}
			else	::cout << "get com ready to send fail. try again" << endl;
		else	::cout << "open com fail. try again" << endl;
	}
	if (REAL) {
		while (true) {
			if (sendFlag)
			{
				robotSerialPort.sendMessage(SendRobotID, global_vx, global_vy, 0);
			}				
		}
	}
}
bool rePlanCheck(Coord robotPos, Coord targetPos, RobotVector& obsVec) {
	bool ret = false;
	for (auto obstacle : obsVec) {
		if (obstacle.isCollision(targetPos) || obstacle.isCollision(robotPos)) {
			cout << "obstacle on the way" << endl;
			printf("robotPos: (%f, %f)\n", robotPos.getX(), robotPos.getY());
			printf("targetPos: (%f, %f)\n", targetPos.getX(), targetPos.getY());
			printf("obstacle: (%f, %f)\n", obstacle.pos().getX(), obstacle.pos().getY());
			ret = true;
			break;
		}
	}
	return ret;
}
void generateVelocity(Coord robotPos, Coord targetPos, double& vx, double& vy, double orientation , double v_total) {
	
	double resultantSpeed = v_total;
	if (v_total == 140) {
		resultantSpeed = 140;
	}
	else if (v_total < 50) {
		resultantSpeed = 50;
	}
	else
	{
		resultantSpeed = v_total;
	}
	double alpha;
	double theta = atan2(targetPos.getY() - robotPos.getY(), targetPos.getX() - robotPos.getX());
	
	if (robotPos.getX() < targetPos.getX()) {
		if ((targetPos.getY() - robotPos.getY()) > 0) {
			alpha = pi + theta;
		}
		else {
			alpha = -pi + theta;
		}
	}
	else {
		if ((targetPos.getY() - robotPos.getY()) > 0) {
			alpha = theta;
		}
		else {
			alpha = theta;
		}
	}
	/*else if (orientation > 0)	alpha = pi - theta - orientation;
	else	alpha = pi + theta + orientation;*/
	double jiajiao = orientation - alpha;

	int signal_x = (goal.getX() < 0) ? 1 : -1;
	int signal_y = (goal.getY() < 0) ? 1 : -1;

	vx =  signal_x * resultantSpeed * cos(jiajiao);
	vy = - signal_y * resultantSpeed * sin(jiajiao);
}
void drawPath(SOCKET soDebug, SOCKADDR_IN si_debug) {
	while (true) {
		if (drawFlag) {
			ZSS::Protocol::Debug_Msgs msgs;
			for (int i = 0; i < errt_path.size() - 1; i++) {
				auto node = errt_path[i];
				auto next_node = errt_path[i + 1];
				auto *msg = msgs.add_msgs();
				msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
				msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
				auto* line = msg->mutable_line();
				auto* start = line->mutable_start();
				auto* end = line->mutable_end();
				start->set_x(node.getX());
				start->set_y(node.getY());
				end->set_x(next_node.getX());
				end->set_y(next_node.getY());
				line->set_forward(TRUE);
				line->set_back(TRUE);
			}
			for (int i = 0; i < origin_path.size() - 1; i++) {
				auto node = origin_path[i];
				auto next_node = origin_path[i + 1];
				auto *msg = msgs.add_msgs();
				msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
				msg->set_color(ZSS::Protocol::Debug_Msg_Color_GREEN);
				auto* line = msg->mutable_line();
				auto* start = line->mutable_start();
				auto* end = line->mutable_end();
				start->set_x(node.getX());
				start->set_y(node.getY());
				end->set_x(next_node.getX());
				end->set_y(next_node.getY());
				line->set_forward(TRUE);
				line->set_back(TRUE);
			}
			int MSGsize = msgs.ByteSize();
			char* MSGRecv = new char[MSGsize];
			msgs.SerializePartialToArray(MSGRecv, MSGsize);
			int nRet = sendto(soDebug, MSGRecv, MSGsize, 0, (SOCKADDR*)&si_debug, sizeof(SOCKADDR));
		}
	}
}

int main(void)
{

	//启动Winsock
	if (Socket_Init() == false) return 0;
	SOCKADDR_IN si_local;    //远程发送机地址和本机接收机地址
	// 接收部分
	SOCKET soRecv;         //发送, 接收SOCKET
	const int RecvPort = 23333;
	const char* RecvADDR = "127.0.0.1";
	SOCKADDR_IN si_remote;
	char *pszRecv = new char[4096];
	if (SocketRecvInit(soRecv, si_local, si_remote, pszRecv, RecvADDR, RecvPort) == false) return 1;
	// 发送部分
	const int SendPort = 20011;
	SOCKET soSend;
	const char* SendADDR = RecvADDR;
	if (SocketSendInit(soSend, si_local, SendADDR, SendPort) == false) return 1;
	// 画图部分
	SOCKADDR_IN si_debug;
	const int DebugPort = 20001;
	SOCKET soDebug;
	const char* DebugADDR = RecvADDR;
	if (SocketSendInit(soDebug, si_debug, DebugADDR, DebugPort) == false) return 1;
	
	Vision_DetectionFrame vision; // 图像类
	

	// 建立线程
	thread recvThread(recvFunc, soRecv, si_remote);
	recvThread.detach();
	thread sendThread(sendFunc);
	sendThread.detach();
	thread drawThread(drawPath, soDebug, si_debug);
	drawThread.detach();


	// 串口初始化
	
	bool reRRTFlag = true;

	// 机器人信息
	Robot myRobot;
	RobotVector obsRobot;

	int PathNodeID = 0;
	int nRet = 0; // 接收的数据长度
	int dwSendSize = 0;
	dwSendSize = sizeof(si_remote); //本地接收变量的大小


	double vx = 0;
	double vy = 0;


	while (true) {
		// 数据接收
		recvFlag = false;
		Sleep(10);
		nRet = recvfrom(soRecv, pszRecv, 4096, 0, (SOCKADDR*)&si_remote, &dwSendSize);
		recvFlag = true;
		pszRecv[nRet] = '\0';
		HandleRecvData(vision, pszRecv, obsRobot, myRobot); // 处理场上车辆及障碍物信息
		cout << "reRRTFlag: " << reRRTFlag << endl;
		// ERRT PLANNING
		if (reRRTFlag) {
			cout << "path replanning" << endl;
			CoordVector PastSuccess;
			Sleep(10);
			ERRTPlanner errt_planner = ERRTPlanner(myRobot, goal, obsRobot, PastSuccess);
			drawFlag = false;
			if (errt_planner.findERRTPath(errt_path, origin_path) == false) continue;
			// cout << "goal : x = " << goal.getX() << " y = " << goal.getY() << endl;
			// Coord end = *(errt_path.end() - 1);
			// cout << "end: x = " << end.getX() << " y = " << end.getY() << endl;
			cout << "path found" << endl;
			/*if (PastSuccess.empty())
				PastSuccess = errt_path;
			else {
				PastSuccess.clear();
				PastSuccess.shrink_to_fit();
				PastSuccess = errt_path;
			}*/
			reRRTFlag = false;
			PathNodeID = 1;
		}
		// 路径显示部分
		if (!errt_path.empty()) {
			drawFlag = true;
		}
		double theta = -myRobot.orientation();
		Coord current = myRobot.pos();
		Coord expect = errt_path[PathNodeID];
			
		Coord target;
		if (current.dist(expect) < 1.5 * ROBOTSIZE) {
			//cout << "update next point as subgoal" << endl;
			target = errt_path[++PathNodeID];
		}
		else {
			//cout << "hold current point as subgoal" << endl;
			target = errt_path[PathNodeID];
		}
		 // 动态重规划
		//if (rePlanCheck(current, target, obsRobot)) {
		//	
		//	cout << "obstacle in the way, replan" << endl;
		//	cout << "goal : x = " << goal.getX() << " y = " << goal.getY() << endl;
		//	reRRTFlag = true;
		//	sendFlag = false;
		//	PathNodeID = 1;
		//	// 速度发送
		//	continue;
		//}
		//else 
		if (current.dist(goal) < 3  *  ROBOTSIZE) {
			generateVelocity(current, target, vx, vy, theta, current.dist(goal) / (3 * ROBOTSIZE) * 120);
			sendFlag = false;
			Sleep(1);
			global_vx = vx;
			global_vy = vy;
			global_w = 0;
			sendFlag = true;
			
			if (current.dist(goal) < 1.5 * ROBOTSIZE) {
				cout << "goal arrived" << endl;
				goal.setX(-goal.getX());
				goal.setY(-goal.getY());
				//if (goal == Coord(-207, -134)) {
				//	goal = Coord(207, 134);
				//	// cout << "goal changed to 220,0" << endl;
				//}
				//else {
				//	goal = Coord(-207, -134);
				//	// cout << "goal changed to -220,0" << endl;
				//}
				reRRTFlag = true;
				PathNodeID = 1;
			}
			cout << "goal : x = " << goal.getX() << " y = " << goal.getY() << endl;
			// 速度发送
		}
		else {
			generateVelocity(current, target, vx, vy, theta, 150);
			cout << "vx = " << vx << " vy = " << vy << endl;
			sendFlag = false;
			Sleep(1);
			global_vx = vx;
			global_vy = vy;
			global_w = 0;
			sendFlag = true;
			cout << "velocity send" << endl;
			cout << "goal : x = " << goal.getX() << " y = " << goal.getY() << endl;
			
			// 速度发送
			
			/*Motion_Info robot(SendRobotID, vx, vy, 0, !MyRobotBlue);
			int CommandSize = robot.Get_Size();
			char* CommandArray = robot.Get_pszRecv();
			nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));*/
		}
		cout << "sendFlag = " << sendFlag << endl;
	}
	closesocket(soRecv);
	delete[] pszRecv;
	closesocket(soSend);
	WSACleanup();
	::system("pause");
	return 0;

}