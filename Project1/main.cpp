#include <iostream>
#include <thread>
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
# define MyRobotBlue true
# define MyRobotID 0
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
				cout << "my robot in yellow: " << endl << "robot_x = " << car.x() << " robot_y = " << car.y() << endl;
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
void generateMotion(double& vtang, double& vnorm, double& vangl, CoordVector& errtpath, const Robot& myRobot, double &t) {
	Coord start = errtpath[0];
	cout << "start_x = " << start.getX() << " ---- start_y = " << start.getY() << endl;
	Coord goal(0, 0);
	cout << "goal_x = " << goal.getX() << " ---- goal_y = " << goal.getY() << endl;
	// Coord goal = errtpath[1];
	double dir = myRobot.orientation();
	cout << "robot orientation = " << dir << endl;
	double dist = start.dist(goal);
	double towards = atan2(goal.getX() - start.getX(), goal.getY() - start.getY());
	cout << "towards = "<< towards << endl;
	double gamma = 2;
	double vx = (goal.getX() - start.getX()) / dist * gamma;
	double vy = -(goal.getY() - start.getY()) / dist * gamma;
	vx = 0;
	vy = 1;
	t = dist / sqrt(vx * vx + vy * vy);
	vtang = vx * cos(dir) + vy * sin(dir);
	vnorm = vx * sin(dir) - vy * cos(dir);
	cout << "tang_vel: " << vtang << " ---- norm_vel: " << vnorm << endl << endl;
	vangl = (towards - dir) / t;
	vangl = 0;
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
	//设置端口号
	

	


	Vision_DetectionFrame vision;
	int loop = 0;
	CoordVector PastSuccess;
	thread recvThread(recvFunc, soRecv, si_remote);
	recvThread.detach();
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
		/*float ball_x, ball_y, ball_vx, ball_vy;
		float car_x, car_y, car_vx, car_vy, car_ort, car_w;*/

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
			
			/*auto ball = vision.balls();
			ball_x = ball.x();
			ball_y = ball.y();
			ball_vx = ball.vel_x();
			ball_vy = ball.vel_x();

			int blue_size = vision.robots_blue_size();
			for (int j = 0; j < blue_size; j++) {
				auto blue = vision.robots_blue(j);
				if (blue.robot_id() == 2) {

					car_x = blue.x();
					car_y = blue.y();
					car_vx = blue.vel_x();
					car_vy = blue.vel_y();
					car_ort = blue.orientation();
					car_w = blue.rotate_vel();

					cout << "id: " << blue.robot_id() << " x: " << blue.x() << " y: " << blue.y() << endl;
				}
			}*/
		} 
		
		
		// ERRT PLANNING
		ERRTPlanner errt_planner = ERRTPlanner(myRobot, goal, obsRobot, PastSuccess);
		CoordVector errt_path;
		if (errt_planner.findERRTPath(errt_path) == false) return 0;
		if (PastSuccess.empty())
			PastSuccess = errt_path;
		else {
			PastSuccess.clear();
			PastSuccess.shrink_to_fit();
			PastSuccess = errt_path;
		}


		/*
		ZSS::Protocol::Debug_Msgs msgs;
		auto *msg = msgs.add_msgs();
		msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
		msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
		ZSS::Protocol::Debug_Line* line = new ZSS::Protocol::Debug_Line;
		ZSS::Protocol::Point* start = new ZSS::Protocol::Point;
		ZSS::Protocol::Point* end = new ZSS::Protocol::Point;
		start->set_x(1);
		start->set_y(1);
		end->set_x(100);
		end->set_y(100);
		line->set_allocated_start(start);
		line->set_allocated_end(end);
		line->set_forward(TRUE);
		line->set_back(TRUE);
		msg->set_allocated_line(line);
		
		int MSGsize = msgs.ByteSize();
		char* MSGRecv = new char[MSGsize];
		// msg.SerializePartialToArray(pszRecv, size);
		msgs.SerializePartialToArray(MSGRecv, MSGsize);
		nRet = sendto(soSend, MSGRecv, MSGsize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));

		*/
		

		


		double vtang, vnorm, vangl, tPeriod;
		generateMotion(vtang, vnorm, vangl, errt_path, myRobot, tPeriod);
		cout << "time needed = " << tPeriod << endl;
		

		grSim_Packet packet;
		auto* command = packet.mutable_commands();
		command->set_timestamp(0);
		command->set_isteamyellow(false);
		auto* robot_command1 = command->add_robot_commands();
		robot_command1->set_id(MyRobotID);
		robot_command1->set_kickspeedx(0);
		robot_command1->set_kickspeedz(0);
		robot_command1->set_veltangent(vtang);
		robot_command1->set_velnormal(vnorm);
		robot_command1->set_velangular(vangl);
		robot_command1->set_spinner(0);
		robot_command1->set_wheelsspeed(false);

		auto robot_command2 = command->add_robot_commands();
		robot_command1->set_id(MyRobotID);
		robot_command1->set_kickspeedx(0);
		robot_command1->set_kickspeedz(0);
		robot_command1->set_veltangent(vtang);
		robot_command1->set_velnormal(vnorm);
		robot_command1->set_velangular(vangl);
		robot_command1->set_spinner(0);
		robot_command1->set_wheelsspeed(false);



		int CommandSize = packet.ByteSize();
		char* CommandArray = new char[CommandSize];
		packet.SerializePartialToArray(pszRecv, CommandSize);

		/*
		1Motion_Info robot2(MyRobotID, vtang, vnorm, vangl, !MyRobotBlue);
		int CommandSize = robot2.Get_Size();
		char* CommandArray = robot2.Get_pszRecv();
		nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
		*/
		Sleep(tPeriod);
		// Sleep(tPeriod * 100);
		
		Motion_Info robot1(MyRobotID, 0, 0, 0, !MyRobotBlue);
		CommandSize = robot1.Get_Size();
		CommandArray = robot1.Get_pszRecv();
		nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
		
		/*
		
		
		while (true) {
			nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
			if (nRet == SOCKET_ERROR) {
				cout << "sendto Error " << WSAGetLastError() << endl;
				break;
			}
			nRet = recvfrom(soRecv, pszRecv, 4096, 0, (SOCKADDR*)&si_remote, &dwSendSize);
			HandleRecvData(vision, pszRecv, obsRobot, myRobot);
			if (goal.dist(myRobot.pos()) < ROBOTSIZE) {
				Motion_Info robot1(MyRobotID, 0, 0, 0, !MyRobotBlue);
				CommandSize = robot1.Get_Size();
				CommandArray = robot1.Get_pszRecv();

				nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
				if (nRet == SOCKET_ERROR) {
					cout << "sendto Error " << WSAGetLastError() << endl;
					break;
				}
				break;
			}
		}
		*/
		loop++;
	}
	closesocket(soRecv);
	delete[] pszRecv;
	closesocket(soSend);
	WSACleanup();
	::system("pause");
	return 0;

}
/*
float clip(float num, const float min, const float max) {
if (num > max) num = max;
else if (num < min) num = min;
return num;
}
float angle_normalize(float angle) {
if (angle > pi)	angle -= 2 * pi;
else if (angle < -pi) angle += 2 * pi;
return angle;
}
*/
