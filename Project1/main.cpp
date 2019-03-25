#include <iostream>
#include <WinSock2.h>
#include <math.h>
#include "vision_detection.pb.h"
#include "grSim_Packet.pb.h"
#include "Motion_Info.h"
#include "model.h"
#include "ERRT.h"

#define pi 3.1415926
# define MyRobotBlue true
# define MyRobotID 5
using namespace std;
#pragma comment(lib, "libprotobuf.lib")
#pragma comment(lib, "ws2_32.lib")

void generateMotion(float &vx, float &w, float ball_x, float ball_y, float ball_vx, float ball_vy, float car_x, float car_y, float car_vx, float car_vy, float car_ort, float car_w);
float angle_normalize(float angle);
float clip(float num, const float min, const float max);




bool Socket_Init(void)
{
	WSAData wsd; // 初始化信息
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {/*进行WinSocket的初始化,
												windows 初始化socket网络库，申请2，2的版本，windows socket编程必须先初始化。*/
		cout << "WSAStartup Error = " << WSAGetLastError() << endl;
		return false;
	}
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
	if (bind(soRecv, (SOCKADDR*)&si_local, sizeof(si_local)) == SOCKET_ERROR) {
		cout << "bind error = " << WSAGetLastError() << endl;
		return false;
	}

	if (pszRecv == NULL) {
		cout << "pszRecv new char Error " << endl;
		return false;
	}

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

}
void HandleRecvData(Vision_DetectionFrame &vision, char* &pszRecv, RobotVector &obstacle, Robot &myRobot) {
	/*
	Input: pszRecv, vision
	function: 处理读入的数据流
	Output: 障碍机器人信息，被控机器人信息
	*/
	vision.ParseFromArray(pszRecv, 4096);
	if (MyRobotBlue) {
		for (int i = 0; i < vision.robots_blue_size; i++) {
			auto car = vision.robots_blue(i);
			if (car.robot_id == MyRobotID) {
				myRobot.setRobotParam(car.x(), car.y(), car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel());
			}
			else {
				obstacle.push_back(Robot(car.x(), car.y(), car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
			}
		}
		for (int i = 0; i < vision.robots_yellow_size; i++) {
			auto car = vision.robots_blue(i);
			obstacle.push_back(Robot(car.x(), car.y(), car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
		}
	}
	else {
		for (int i = 0; i < vision.robots_yellow_size; i++) {
			auto car = vision.robots_yellow(i);
			if (car.robot_id == MyRobotID) {
				myRobot.setRobotParam(car.x(), car.y(), car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel());
			}
			else {
				obstacle.push_back(Robot(car.x(), car.y(), car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
			}
		}
		for (int i = 0; i < vision.robots_blue_size; i++) {
			auto car = vision.robots_yellow(i);
			obstacle.push_back(Robot(car.x(), car.y(), car.vel_x(), car.vel_y(), car.orientation(), car.rotate_vel()));
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
	//设置端口号
	

	// vision类
	static Vision_DetectionFrame vision;



	int loop = 0;
	while (true) {
		Robot myRobot;
		RobotVector obsRobot;
		int dwSendSize = 0; 
		int nRet = 0; // 接收的数据长度
		dwSendSize = sizeof(si_remote); //本地接收变量的大小
		nRet = recvfrom(soRecv, pszRecv, 4096, 0, (SOCKADDR*)&si_remote, &dwSendSize);
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
			HandleRecvData(vision,pszRecv, obsRobot, myRobot);
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
		// 根据输入产生输出
		/*
		TODO: ERRT PLANNING
		*/
		/*
		TODO: generate vx,vy,w based on ERRT PLANING result
		set the next point of ERRT as goal point, 
		generate velocity that drive robot towards that goal (function should return vtang, vnorm and vangl)
		*/
		
		Motion_Info robot(MyRobotID, vtang, vnorm, vangl, !MyRobotBlue);
		int CommandSize = robot.Get_Size();
		char* CommandArray = robot.Get_pszRecv();

		nRet = sendto(soSend, CommandArray, CommandSize, 0, (SOCKADDR*)&si_local, sizeof(SOCKADDR));
		if (nRet == SOCKET_ERROR) {
			cout << "sendto Error " << WSAGetLastError() << endl;
			break;
		}
		loop++;
	}
	closesocket(soRecv);
	delete[] pszRecv;
	closesocket(soSend);
	WSACleanup();
	system("pause");
	return 0;

}
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

void generateMotion(float &vx, float &w, float ball_x, float ball_y, float ball_vx, float ball_vy, float car_x, float car_y, float car_vx, float car_vy, float car_ort, float car_w) {
	float robot2ball_theta = atan2(ball_y - car_y, ball_x - car_x);
	float del_theta = angle_normalize(robot2ball_theta - car_ort);
	cout << "del_theta: " << del_theta << endl;
	float vel;
	if (abs(del_theta) < 5.0 / 180 * pi) vel = 2;
	else if (abs(del_theta) < 10.0 / 180 * pi) vel = 1.0;
	else if (abs(del_theta) < 30.0 / 180 * pi) vel = 0.5;
	else vel = 0;
	vx = clip(vel, -3, 3);
	w = clip(del_theta, -2, 2);
}
