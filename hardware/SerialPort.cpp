#include <stdio.h>
#include "SerialPort.h"

void sleep( clock_t wait )
{
   clock_t goal;
   goal = wait + clock();
   while( goal > clock() )
      ;
}

SerialPort::SerialPort(PortNumber port, int baudRate)
{

	LPCTSTR comPort;
	switch (port) {
		case COM1:
			comPort = "COM1";
			break;
		case COM2:
			comPort = "COM2";
			break;
		case COM3:
			comPort = "COM3";
			break;
		case COM4:
			comPort = "COM4";
			break;
		case COM5:
			comPort = "COM5";
			break;
		case COM6:
			comPort = "COM6";
			break;
		default:
			fprintf(stderr,"Invalid communication port requested\n");
			lastError = 5;
			return;
	}
	hCom = CreateFile( comPort,
					GENERIC_READ | GENERIC_WRITE,
					0,
					NULL,
					OPEN_EXISTING,
					0,
					NULL
					);

	if (hCom == INVALID_HANDLE_VALUE) {
		fprintf(stderr,"Could not open port!\n");
		lastError = 1;
		return;
	}

	BOOL fSuccess = GetCommState(hCom, &dcb);
	if (!fSuccess) 	{
      fprintf (stderr,"Could not get initial dcb! Error %d\n", GetLastError());
      lastError = 3;
	  return;
	}

	dcb.ByteSize=8;
	dcb.Parity=NOPARITY;
	dcb.StopBits=ONESTOPBIT;

	switch(baudRate) {
		case 9600:
			dcb.BaudRate=CBR_9600;
			break;
		case 38400:
			dcb.BaudRate=CBR_38400;
			break;
		case 115200:
			dcb.BaudRate=CBR_115200;
			break;
		default:
			dcb.BaudRate=CBR_38400;
			fprintf(stderr,"Baud rate not understood, using 38400 by default\n");
			break;
	}

	fSuccess = SetCommState(hCom,&dcb);
	if (!fSuccess){
		fprintf(stderr, "Could not set port settings, error %d\n",GetLastError());
		lastError = 2;
		return;
	}

	fSuccess = GetCommState(hCom,&dcb);
	if (!fSuccess) {
		fprintf(stderr,"GetCommState failed, error %d. ", GetLastError());
		lastError = 3;
		return;
	}
	fprintf(stderr,"BAUD rate is %d\n",dcb.BaudRate);
	lastError = 0;
}

SerialPort::~SerialPort()
{
	CloseHandle(hCom);
}

int SerialPort::writeString(char *string)
{
	if (strlen(string)==0) {
		return 0;
	}
	DWORD result;
	WriteFile(hCom,string,strlen(string),&result,NULL);
	return result;
}

int SerialPort::writeNBytes(int nBytes, unsigned char *record)
{
	if (nBytes==0) {
		return 0;
	}
	DWORD result;
	WriteFile(hCom,record,nBytes,&result,NULL);
	return result;
}

int SerialPort::readString(int nBytes, char *string)
{
	DWORD result;
	ReadFile(hCom,string,nBytes,&result,NULL);
	return result;
}

int SerialPort::readTerminatedString(int nBytes, char *string)
{
	return this->readStringWithTerminator(nBytes, string, '\0');
}

int SerialPort::readStringWithTerminator(int nBytes, char *string, char terminator)
{
	if (nBytes == 0)
		return 0;

	DWORD result;

	float timeout = 2 * CLOCKS_PER_SEC;

	clock_t goal;
    goal = timeout + clock();
//	fprintf(stderr,"read terminated string\n");
	int currentPosition = 0;
	while (currentPosition == 0 || string[currentPosition-1] != terminator) {

		if (goal < clock() ) {
			fprintf(stderr,"Port timeout!\n");
			return 0;
		}

//		ReadFile(hCom, string+currentPosition, nBytes-currentPosition, &result, NULL);
		ReadFile(hCom, string+currentPosition, 1, &result, NULL);
		currentPosition += result;
		if ( currentPosition >= nBytes ) {
			fprintf(stderr,"Read terminated string overflow\n");
			break;
		}
//		fprintf(stderr,"Keep reading\n");
	}
	if(currentPosition < nBytes && terminator != '\0')
		string[currentPosition] = '\0';
	return currentPosition;
}

int SerialPort::readNBytes(int nBytes, unsigned char *string)
{
	if (nBytes == 0)
		return 0;

	DWORD result;

	float timeout = 2 * CLOCKS_PER_SEC;

	clock_t goal;
    goal = timeout + clock();

	int currentPosition = 0;
	while (currentPosition < nBytes) {
		if (goal < clock() ) {
			fprintf(stderr,"Port timeout!\n");
			return 0;
		}
		ReadFile(hCom, string+currentPosition, nBytes-currentPosition, &result, NULL);
		currentPosition += result;
	}

	if ( currentPosition > nBytes ) {
		fprintf(stderr,"Possible overflow: expected %d bytes and read %d\n",nBytes, currentPosition);
	}
	return currentPosition;
}

void SerialPort::useTimeouts()
{
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 400;
	timeouts.ReadTotalTimeoutMultiplier = 50;
	timeouts.ReadTotalTimeoutConstant = 40000;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 2000;

	if(!SetCommTimeouts(hCom, &timeouts)) {
		fprintf(stderr, "Unable to set timeouts for serial port.\n");
	}
}
