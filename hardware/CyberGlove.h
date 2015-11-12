#ifndef _cyberglove_h_
#define _cyberglove_h_

#include "SerialPort.h"

#define N_SENSOR_VALUES 24

class CyberGlove {
private:
	SerialPort *port;
	unsigned char *sensorValues;
public:
	CyberGlove();
	~CyberGlove();

	int testGlove();

	int instantRead();
	unsigned char getSensorValue(int i);
	const unsigned char *getAllValues(){return sensorValues;}
};

#endif