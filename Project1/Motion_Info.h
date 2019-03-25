#pragma once
# include "grSim_Packet.pb.h"

class Motion_Info
{
public:
	Motion_Info();
	Motion_Info(int id, int Vtang = 0, int Vnorm = 0, int Vangl = 0, bool iyellow = false, int kickspdz = 0, int kickspdx = 0, int spinner = 0, bool wheel = false);
	~Motion_Info();

	char* Get_pszRecv(void);
	int Get_Size(void);
private:
	grSim_Packet packet;
	char* pszRecv;
	int size;
};

