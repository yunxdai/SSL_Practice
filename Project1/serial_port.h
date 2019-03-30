#pragma once
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <afx.h>
#include <Windows.h>
#include <cstddef>
#include <cstdlib>
#include <cassert>

// ���崮������
#ifdef _UNICODE
typedef CString porttype;
#else
typedef std::string porttype;
#endif // _UNICODE

typedef unsigned long ulong;
typedef unsigned char uchar;


class CSerialPort
{
public:
	CSerialPort(
		const porttype & portNum,		// ���ں�
		DWORD baudRate = 9600,			// ������
		BYTE  byteSize = 8,				// ����λ
		BYTE  parityBit = NOPARITY,		// ����λ
		BYTE  stopBit = ONESTOPBIT		// ֹͣλ
		);

	~CSerialPort();

public:

	bool openComm();										// �򿪴���
	void closeComm();										// �رմ���
	bool writeToComm(BYTE data[], DWORD dwLegnth);			// ��������
	bool readFromComm(char buffer[], DWORD dwLength);		// ��ȡ����
	bool getReadyToSend();
	bool encodeData(int robotID, int Vx, int Vy, int w);
	bool sendMessage(int robotID, int Vx, int Vy, int w);
private:
	BYTE _startPacket1[25] = { 0xff,0xb0,0x01,0x02,0x03,0x00,0x00,0x00,
							   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
							   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x31 };
	BYTE _startPacket2[25] = { 0xff,0xb0,0x04,0x05,0x06,0x10,0x00,0x00,
							   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
							   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x85 };
	BYTE _encodePacket[25] = { 0 };
	DWORD _packetSize = 25;
	HANDLE m_hComm;		// ͨ���豸
	porttype m_portNum; // ���ں�
	DWORD m_dwBaudRate; // ������
	BYTE  m_byteSize;	// ����λ
	BYTE  m_parityBit;  // У��λ
	BYTE  m_stopBit;	// ֹͣλ
	bool  m_bOpen;		// ���ڿ��ر�־
private:

	enum BufferSize
	{
		MIN_BUFFER_SIZE = 256,
		BUFFER_SIZE = 512,
		MAX_BUFFER_SIZE = 1024
	};

	// ���ô��ں�
	void setPortNum(const porttype &portNum)
	{
		this->m_portNum = portNum;
	}
	// ���ò�����
	void setBaudRate(const ulong baudRate)
	{
		this->m_dwBaudRate = baudRate;
	}
	// ��������λ
	void setByteSize(const uchar byteSize)
	{
		this->m_byteSize = byteSize;
	}
	// ���ü���λ
	void setParityBit(const uchar parityBit)
	{
		this->m_parityBit = parityBit;
	}
	// ����ֹͣλ
	void setStopBit(const uchar stopBit)
	{
		this->m_stopBit = stopBit;
	}

	// ��ȡ���ں�
	porttype getPortNum() { return m_portNum; }
	// ��ȡ������
	ulong getBaudRate() { return m_dwBaudRate; }
	// ��ȡ����λ
	uchar getByteSize() { return m_byteSize; }
	// ��ȡ����λ
	uchar getParityBit() { return m_parityBit; }
	// ��ȡֹͣλ
	uchar getStopBit() { return m_stopBit; }
};




#endif		// SERIAL_PORT_H
