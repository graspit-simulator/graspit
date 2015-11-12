#ifndef _serialport_h_
#define _serialport_h_

#include "windows.h"
#include <time.h>

void sleep(clock_t wait);

class SerialPort{
public:
	enum PortNumber{COM1, COM2, COM3, COM4, COM5, COM6};
private:
	DCB dcb;
	HANDLE hCom;
	int lastError;

public:
	SerialPort(PortNumber port, int baudRate);
	~SerialPort();
	int getError(){return lastError;}

	int writeString(char *string);
	int writeNBytes(int nBytes, unsigned char *record);
	int readString(int nBytes, char *string);
	int readTerminatedString(int nBytes, char *string);
	int readStringWithTerminator(int nBytes, char *string, char terminator);
	int readNBytes(int nBytes, unsigned char *string);
	void useTimeouts();
};

#endif