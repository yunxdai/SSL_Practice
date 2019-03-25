#include "Motion_Info.h"


Motion_Info::Motion_Info(int id, int Vtang, int Vnorm, int Vangl, bool isyellow, int kickspdz, int kickspdx, int spinner, bool wheel)
{
	auto* command = packet.mutable_commands();
	command->set_timestamp(0);
	command->set_isteamyellow(isyellow);
	auto* robot_command = command->add_robot_commands();
	robot_command->set_id(id);
	robot_command->set_kickspeedx(kickspdx);
	robot_command->set_kickspeedz(kickspdz);
	robot_command->set_veltangent(Vtang);
	robot_command->set_velnormal(Vnorm);
	robot_command->set_velangular(Vangl);
	robot_command->set_spinner(spinner);
	robot_command->set_wheelsspeed(wheel);

	size = packet.ByteSize();
	pszRecv = new char[size];
	packet.SerializePartialToArray(pszRecv, size);
}
char* Motion_Info::Get_pszRecv(void) {
	return pszRecv;
}
int Motion_Info::Get_Size(void) {
	return size;
}
Motion_Info::~Motion_Info()
{
	delete[] pszRecv;
}
