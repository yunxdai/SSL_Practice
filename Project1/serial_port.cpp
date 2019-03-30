// #include <stdafx.h>
#include <iostream>
#include "serial_port.h"

CSerialPort::CSerialPort(
	const porttype & portNum,
	DWORD baudRate /* = 9600 */,
	BYTE byteSize /* = 8 */,
	BYTE parityBit /* = NOPARITY */,
	BYTE stopBit /* = ONESTOPBIT */
	) : m_portNum(portNum),
	m_dwBaudRate(baudRate),
	m_byteSize(byteSize),
	m_parityBit(parityBit),
	m_stopBit(stopBit),
	m_bOpen(false)
{

}

CSerialPort::~CSerialPort()
{

}

// �򿪴��ڳɹ�,���� true
bool CSerialPort::openComm()
{
#ifdef _UNICODE
	m_hComm = CreateFile(m_portNum,
		GENERIC_READ | GENERIC_WRITE, //�������д
		0,		//��ռ��ʽ
		NULL, OPEN_EXISTING,   //�򿪶����Ǵ���
		0,   //ͬ����ʽ
		NULL
		);
#else
	m_hComm = CreateFileA(m_portNum.c_str(),
		GENERIC_READ | GENERIC_WRITE, //�������д
		0,		//��ռ��ʽ
		NULL, OPEN_EXISTING,   //�򿪶����Ǵ���
		0,   //ͬ����ʽ
		NULL
		);
#endif

	if (m_hComm == INVALID_HANDLE_VALUE)
	{
#ifdef _UNICODE
		TCHAR szBuf[1024] = { 0 };
		wsprintf(szBuf, L"�� %s ʧ��,����: %d", m_portNum, GetLastError());
		MessageBox(NULL, szBuf, L"Warnning", MB_OK);
#else
		char szBuf[1024] = { 0 };
		sprintf_s(szBuf, "�� %s ʧ��,����: %d", m_portNum, GetLastError());
		MessageBox(NULL, szBuf, "Warnning", MB_OK);
#endif // _UNICODE

		return false;

	}
	else
	{
		DCB dcb;
		SetupComm(m_hComm, MAX_BUFFER_SIZE, MAX_BUFFER_SIZE);	// ���ö�д��������С
		GetCommState(m_hComm, &dcb);
		dcb.BaudRate = m_dwBaudRate;
		dcb.ByteSize = m_byteSize;
		dcb.Parity = m_parityBit;
		dcb.StopBits = m_stopBit;

		if (!SetCommState(m_hComm, &dcb))
		{
#ifdef _UNICODE
			TCHAR szBuf[1024] = { 0 };
			wsprintf(szBuf, L"��������ʧ��,�������: %d", GetLastError());
			MessageBox(NULL, szBuf, TEXT("ERROR"), MB_OK);
#else
			char szBuf[1024] = { 0 };
			wsprintf(szBuf, "��������ʧ��,�������: %d", GetLastError());
			MessageBox(NULL, szBuf, "ERROR", MB_OK);
#endif
			return false;
		}

	}

	//�ڶ�д����ǰ���� PurgeComm ������ջ�����
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_TXABORT | PURGE_TXABORT);

	m_bOpen = true;

	return true;

}

// �رմ���
void CSerialPort::closeComm()
{
	CloseHandle(m_hComm);
}

// �򴮿ڷ�������
bool CSerialPort::writeToComm(BYTE data[], DWORD dwLength)
{
#ifdef _DEBUG
	assert(m_bOpen == true || dwLength > 0);
	return false;
#endif // _DEBUG
	DWORD dwError = 0;
	if (ClearCommError(m_hComm, &dwError, NULL) && dwError > 0)
	{
		PurgeComm(m_hComm, PURGE_TXABORT | PURGE_TXCLEAR);
	}

	DWORD dwTx = 0;
	BOOL ret = FALSE;
	ret = WriteFile(m_hComm, data, dwLength, &dwTx, NULL);
	std::cout << "ret in WriteToComm " << ret << std::endl;
	if (ret == FALSE)
	{
#ifdef _UNICODE
		TCHAR szBuf[1024] = { 0 };
		wsprintf(szBuf, _T("д������ʧ��,�������: %d"), GetLastError());
		MessageBox(NULL, szBuf, L"ERROR", MB_OK);
#else
		char szBuf[1024] = { 0 };
		sprintf_s(szBuf, "д������ʧ��, �������: %d", GetLastError());
		MessageBox(NULL, szBuf, "Error", MB_OK);
#endif // _UNICODE

		return false;
	}

	return true;


}

// �Ӵ����ж�ȡ����
bool CSerialPort::readFromComm(char buffer[], DWORD dwLength)
{
#ifdef _DEBUG
	assert(m_bOpen == true || dwLength > 0);
	return false;
#endif // _DEBUG

	COMSTAT comStat;
	DWORD dwError = 0;
	if (ClearCommError(m_hComm, &dwError, &comStat) && dwError > 0)
	{
		PurgeComm(m_hComm, PURGE_RXABORT | PURGE_RXCLEAR);
	}

	DWORD dwRx = 0;		// ������ֽ���
	BOOL ret = FALSE;
	BYTE* byReadData = new BYTE[dwLength];
	char szTmp[4] = { 0 };
	int sizeOfBytes = sizeof(szTmp);
	ret = ReadFile(m_hComm, byReadData, dwLength, &dwRx, NULL);	// ��������

	if (ret == TRUE)
	{
		for (UINT i = 0; i < dwRx; ++i)
		{
			sprintf_s(szTmp, "%02x", byReadData[i]);
			strcat_s(buffer, sizeOfBytes*dwLength, szTmp);
		}

		// �ͷ��ڴ�
		delete byReadData;

		return true;
	}
	else
	{
#ifdef _UNICODE
		TCHAR szBuf[1024] = { 0 };
		wsprintf(szBuf, _T("���ݶ�ȡʧ��,�������: %d"), GetLastError());
		MessageBox(NULL, szBuf, L"Error", MB_OK);
#else
		char szBuf[1024] = { 0 };
		wsprintf(szBuf, "���ݶ�ȡʧ��,�������: %d", GetLastError());
		MessageBox(NULL, szBuf, "Error", MB_OK);
#endif // _UNICODE

		return false;
	}

	return true;
}

bool CSerialPort::getReadyToSend() {
	// writeToComm(_startPacket1, 25);
	
	// writeToComm(_startPacket2, 25);
	
	int tryCount = 0;
	bool readFlag = false;
	if (writeToComm(_startPacket1, _packetSize)) {
		/*char* readBuf;
		while (tryCount < 2000)
			if (readFromComm(readBuf, 8)) {
				readFlag = true;
				break;
			}
			else {
				Sleep(10);
				tryCount++;
			}*/
		Sleep(1000);
		int ret = writeToComm(_startPacket2, _packetSize);
		Sleep(1000);
		std::cout << "ret in getReady" << ret << std::endl;
		return true;
	}
	else {
		std::cout << "write false" << std::endl;
		return false;
	}
}

bool CSerialPort::encodeData(int robotID, int Vx, int Vy, int w) {
	_encodePacket[0] = 0xff;
	for (int i = 1; i < _packetSize; i++) {
		_encodePacket[i] = 0x00;
	}
	//RobotID
	if (robotID >= 8)
		_encodePacket[1] = 0x01 << (robotID - 8);
	else
		_encodePacket[1] = 0x00;
	_encodePacket[2] = 0x01 << robotID;
	// misc
	_encodePacket[3] = _encodePacket[3] | (false << 7);
	//shoot or chip
	_encodePacket[3] = _encodePacket[3] | (false << 6);
	//power level
	_encodePacket[3] = _encodePacket[3] | (0 << 4);
	// other
	_encodePacket[3] = _encodePacket[3] & 0xf0;
	_encodePacket[3] = _encodePacket[3] | 0x01;

	// velx
	if (Vx < 0) _encodePacket[4] = _encodePacket[4] | (0x20);
	_encodePacket[4] = _encodePacket[4] | ((abs(Vx) & 0x1f0) >> 4);
	//    qDebug() << "ddq debuging: " << (abs(velX) & 0x1f0);
	_encodePacket[5] = _encodePacket[5] | ((abs(Vx) & 0x0f) << 4);
	// vely
	if (Vy < 0) _encodePacket[5] = _encodePacket[5] | (0x08);
	_encodePacket[5] = _encodePacket[5] | ((abs(Vy) & 0x1c0) >> 6);
	_encodePacket[6] = _encodePacket[6] | ((abs(Vy) & 0x3f) << 2);
	// w
	if (w < 0) _encodePacket[6] = _encodePacket[6] | 0x02;
	_encodePacket[6] = _encodePacket[6] | ((abs(w) & 0x100) >> 8);
	_encodePacket[7] = _encodePacket[7] | (abs(w) & 0x0ff);

	// shoot power
	_encodePacket[8] = 0x00;

	_encodePacket[21] = 0x07;
	return true;
}

bool CSerialPort::sendMessage(int robotID, int Vx, int Vy, int w) {
	if (!encodeData(robotID, Vx, Vy, w))		return false;
	PurgeComm(m_hComm, PURGE_TXABORT | PURGE_TXCLEAR);
	writeToComm(_encodePacket, _packetSize);
	return true;
}
