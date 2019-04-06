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
# define MyRobotBlue false
# define MyRobotID 3
#define SendRobotID 3
using namespace std;
#pragma comment(lib, "libprotobuf.lib")
#pragma comment(lib, "ws2_32.lib")




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
	if (MyRobotBlue) {
		for (int i = 0; i < vision.robots_blue_size(); i++) {
			auto car = vision.robots_blue(i);
			// cout << "blue " << car.robot_id() << ": " << car.x() / 10.0 << ' ' << -car.y() / 10.0 << ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
			if (car.robot_id() == MyRobotID) {
				myRobot.setRobotParam(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel());
				cout << "my robot in blue: " << endl << "robot_x = " << car.x() << " robot_y = " << car.y()  << endl;
			}
			else {
				obstacle.push_back(Robot(car.x() / 10.0, -car.y() / 10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
			}
		}
		for (int i = 0; i < vision.robots_yellow_size(); i++) {
			auto car = vision.robots_yellow(i);
			// cout << "yellow " << car.robot_id() << ": " << car.x() / 10.0<< ' ' << -car.y() / 10.0<< ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
			obstacle.push_back(Robot(car.x()/10.0, -car.y()/10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
		}
	}
	else {
		for (int i = 0; i < vision.robots_yellow_size(); i++) {
			auto car = vision.robots_yellow(i);
			// cout << "yellow " << car.robot_id() << ": " << car.x() / 10.0<< ' ' << -car.y() / 10.0<< ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
			if (car.robot_id() == MyRobotID) {
				myRobot.setRobotParam(car.x()/10.0, -car.y()/10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel());
				// cout << "my robot in yellow: " << endl << "robot_x = " << car.x() << " robot_y = " << car.y() << endl;
				// cout << "now is my robot in yellow: " << car.robot_id() << endl;
			}
			else {
				obstacle.push_back(Robot(car.x()/10.0, -car.y()/10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
			}
		}
		for (int i = 0; i < vision.robots_blue_size(); i++) {
			auto car = vision.robots_blue(i);
			// cout << "blue " << car.robot_id() << ": " << car.x()/10.0 << ' ' << -car.y()/10.0 << ' ' << car.vel_x() << ' ' << car.vel_y() << ' ' << car.rotate_vel() << ' ' << endl;
			obstacle.push_back(Robot(car.x()/10.0, -car.y()/10.0, car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
		}
	}
}
double countT;
void Ttracj(Coord& start, Coord& end, double& vx, double& vy, double & w, double direction, double orientation) {
	if (start.dist(end) < 3 * ROBOTSIZE) {
		vx = vy = 0;
	}
	/*else if (start.dist(end) < 200) {
		double v = 200;
		vx = v * cos(direction - orientation);
		vy = v * sin(direction - orientation);

	}*/
	else{// if (start.dist(end) < 2 * ROBOTSIZE) {
		//double v = start.dist(end) ;
		double v = 120;
		cout << "dist = " << v << endl;
		vx = v * cos(direction - orientation);
		vy = v * sin(direction - orientation);
		w = 0;
		//w = direction - orientation;
		// vx = 100 / start.dist(end) * (end.getX() - start.getX());
		// vy = 100 / start.dist(end) * (end.getY() - start.getY());
	}
	/*
	else {
		vx = -sqrt(start.dist(end)) * (end.getX() - start.getX());
		vy = -sqrt(start.dist(end)) * (end.getY() - start.getY());
	}
	*/
	//// angle 为机器人与目标点的方向 与机器人朝向的夹角
	//double v = sqrt(vx*vx + vy * vy);
	//double deltaT = 0.01;
	//double ACCMAX = 1000;
	//double Vmax = 200;
	//countT = (ACCMAX + Vmax * Vmax) / (Vmax*ACCMAX);
	//double ta = Vmax / ACCMAX;
	//double t = 0;
	//// vx 梯形
	//
	//if (0 < t < ta) {
	//	v += ACCMAX * deltaT;
	//	t += deltaT;
	//}
	//else if (t < countT - ta) {
	//	v = v;
	//	t += deltaT;
	//}
	//else if (t<countT){
	//	v -= ACCMAX * deltaT;
	//	t += deltaT;
	//}
	//else {
	//	v = 0;
	//}
	//vx = v * cos(angle);
	//vy = v * sin(angle);
	//cout << "vx = " << vx << " vy = " << vy << endl;
}



bool recvFlag = false;
void recvFunc(SOCKET recvSocket, sockaddr_in remoteAddr) {
	char* bufRecv = new char[4096];
	int bufSize = 4096;
	int remoteAddrLen = sizeof(remoteAddr);
	int nRet;
	while (true)
		if (recvFlag)	nRet = recvfrom(recvSocket, bufRecv, bufSize, 0, (SOCKADDR*)&remoteAddr, &remoteAddrLen);
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
	


	Vision_DetectionFrame vision;
	int loop = 0;
	CoordVector PastSuccess;
	thread recvThread(recvFunc, soRecv, si_remote);
	recvThread.detach();

	CSerialPort robotSerial("COM6", 115200UL, 8, NOPARITY, ONESTOPBIT);
	robotSerial.openComm();
	robotSerial.getReadyToSend();
	//robotSerial.sendMessage(MyRobotID, 100, 100, 0);

	
	while (true) {
		// vision类
		
		Robot myRobot;
		RobotVector obsRobot;


		Coord goal(0,0);


		recvFlag = false;
		Sleep(10);
		int nRet = 0; // 接收的数据长度
		
		int dwSendSize = 0;
		dwSendSize = sizeof(si_remote); //本地接收变量的大小
		nRet = recvfrom(soRecv, pszRecv, 4096, 0, (SOCKADDR*)&si_remote, &dwSendSize);
		recvFlag = true;

		if (nRet == SOCKET_ERROR) {
			cout << "recvfrom Error " << WSAGetLastError() << endl;
			break;
		}
		else if (nRet == 0) {
			break;
		}
		else {
			// 处理输入数据
			pszRecv[nRet] = '\0';

			HandleRecvData(vision, pszRecv, obsRobot, myRobot);
		} 
		
		
		// ERRT PLANNING
		ERRTPlanner errt_planner = ERRTPlanner(myRobot, goal, obsRobot, PastSuccess);
		CoordVector errt_path;
		CoordVector origin_path;
		if (errt_planner.findERRTPath(errt_path,origin_path) == false) continue;
		if (PastSuccess.empty())
			PastSuccess = errt_path;
		else {
			PastSuccess.clear();
			PastSuccess.shrink_to_fit();
			PastSuccess = errt_path;
		}


		
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
			// cout << "i = " << i << endl;
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
			// cout << "i = " << i << endl;
		}
		int MSGsize = msgs.ByteSize();
		char* MSGRecv = new char[MSGsize];
		msgs.SerializePartialToArray(MSGRecv, MSGsize);
		nRet = sendto(soDebug, MSGRecv, MSGsize, 0, (SOCKADDR*)&si_debug, sizeof(SOCKADDR));


		// Coord start = errt_path[0];
		/*Coord medium = errt_path[1];*/
		// Coord start(195, -95);
		Coord medium = errt_path[1];

		
		while (true) {
			nRet = recvfrom(soRecv, pszRecv, 4096, 0, (SOCKADDR*)&si_remote, &dwSendSize);
			HandleRecvData(vision, pszRecv, obsRobot, myRobot);
			Coord start = myRobot.pos();
			if (start.dist(medium) < 3 * ROBOTSIZE) {
				break;
			}
			for (auto obstacle : obsRobot) {
				if (obstacle.isCollision(myRobot)) {
					break;
				}
			}
			double theta = -myRobot.orientation();
			double vx, vy, w;
			double direction = atan2(medium.getY() - start.getY(), medium.getX() - start.getX());
			cout << "orientation = " << theta << " direction = " << direction << endl;
			Ttracj(start, medium, vx, vy, w, direction, theta);
			cout << "vx = " << vx << " vy = " << vy << endl;
			/*Motion_Info robot(SendRobotID, vx, vy, 0, !MyRobotBlue);
			int CommandSize = robot.Get_Size();
			char* CommandArray = robot.Get_pszRecv();*/
			robotSerial.sendMessage(SendRobotID, vx, vy, 0);
			// nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
			/*Motion_Info robot2(SendRobotID, 0, 0, 0, !MyRobotBlue);
			CommandSize = robot2.Get_Size();
			CommandArray = robot2.Get_pszRecv();
			robotSerial.sendMessage(SendRobotID, 0, 0, 0);*/
			// nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));

			/*if (medGoal.dist(myRobot.pos()) < 1) {
				Motion_Info robot(MyRobotID, 0, 0, 0, !MyRobotBlue);
				int CommandSize = robot.Get_Size();
				char* CommandArray = robot.Get_pszRecv();
				nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
				break;
			}
			else {
				if (fabs(theta - direction) > pi / 5) {
					double vangl;
					if (theta > direction) {
						cout << "theta = " << theta << "direction = " << direction << endl;
						vangl = 1;
					}
					else {
						cout << "theta = " << theta << "direction = " << direction << endl;
						vangl = -1;
					}
					Motion_Info robot(MyRobotID, 0, 0, vangl , !MyRobotBlue);
					int CommandSize = robot.Get_Size();
					char* CommandArray = robot.Get_pszRecv();
					nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
				}
				else {
					Motion_Info robot(MyRobotID, 1, 0, 0, !MyRobotBlue);
					int CommandSize = robot.Get_Size();
					char* CommandArray = robot.Get_pszRecv();
					nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
				}
			}*/
		}
		loop++;
	}
	closesocket(soRecv);
	delete[] pszRecv;
	closesocket(soSend);
	WSACleanup();
	::system("pause");
	return 0;

}