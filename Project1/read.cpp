#include <iostream>
#include <WinSock2.h>

#include "vision_detection.pb.h"
#pragma comment(lib, "libprotobuf.lib")
#pragma comment(lib,"ws2_32.lib")
using namespace std;
int main(void) {
	WORD socketVersion = MAKEWORD(2, 2);
	WSADATA wsadata;
	if (WSAStartup(socketVersion, &wsadata) != 0) {
		cout << "WSAStartup Error = " << WSAGetLastError << endl;
		return 0;
	}
	SOCKET socketRecv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketRecv == SOCKET_ERROR) {
		cout << "socket Error = " << WSAGetLastError << endl;
		return 1;
	}
	int Port = 23333; // ¶Ë¿ÚºÅ
	SOCKADDR_IN si_local;
	si_local.sin_family = AF_INET;
	si_local.sin_port = htons(Port);
	si_local.sin_addr.s_addr = inet_addr("127.0.0.1");
	if (bind(socketRecv, (SOCKADDR*)&si_local, sizeof(si_local)) == SOCKET_ERROR) {
		cout << "bind error = " << WSAGetLastError() << endl;
		return 1;
	}
	SOCKADDR_IN si_remote;
	char *pszRecv = new char[4096];
	if (pszRecv == NULL) {
		cout << "pszRecv new char Error " << endl;
		return 0;
	}
	int dwSendSize = 0;
	int nRet = 0;
	static Vision_DetectionFrame vision;
	/*for (int i = 0; i < 30; i++)*/
	while(true) {
		dwSendSize = sizeof(si_remote);
		nRet = recvfrom(socketRecv, pszRecv, 4096, 0, (SOCKADDR*)&si_remote, &dwSendSize);
		
		if (nRet == SOCKET_ERROR) {
			cout << "recvfrom Error " << WSAGetLastError() << endl;
			break;
		}
		else if (nRet == 0) {
			break;
		}
		else {
			vision.ParseFromArray(pszRecv, 4096);
			auto ball = vision.balls();
			int blue_size = vision.robots_blue_size();
			for (int j = 0; j < blue_size; j++) {
				auto blue = vision.robots_blue(j);
				if (blue.robot_id() == 0) {
					cout << "id" << blue.robot_id() <<"x" << blue.x() <<"y" << blue.y() << endl;
				}
			}
		}

	}
	closesocket(socketRecv);
	delete[] pszRecv;

	WSACleanup();
	system("pause");
	return 0;
}