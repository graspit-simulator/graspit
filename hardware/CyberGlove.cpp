#include "CyberGlove.h"
#include <stdio.h>

CyberGlove::CyberGlove()
{
	port = new SerialPort(SerialPort::COM6, 38400);
	if ( port->getError() ) {
		fprintf(stderr,"Raw Cyber Glove failed to initialize serial port\n");
	}

	sensorValues = new unsigned char[N_SENSOR_VALUES];
}

CyberGlove::~CyberGlove()
{
	delete port;
	delete [] sensorValues;
}

int CyberGlove::testGlove()
{
	if (!port->writeString("?G") ) {
		fprintf(stderr,"Write failed\n");
		return 0;
	}

	char* result = new char[20];
	int read = port->readTerminatedString(20, result);
	if (read <= 0) {
		fprintf(stderr,"No answer read!\n");
		return 0;
	}
	if (result[read-1] != 0) {
		fprintf(stderr,"last bit is not 0:\n");
		return 0;
	}

	int r;
	switch(result[2]) {
		case 0:
			fprintf(stderr,"No glove and not initialized\n");
			r = 0;
			break;
		case 1:
			fprintf(stderr,"Glove present but not initialized\n");
			r = 0;
			break;
		case 2:
			fprintf(stderr,"Glove not present but initialized (strange...)\n");
			r = 0;
			break;
		case 3:
			r = 1;
			break;
		default:
			fprintf(stderr,"Unknown value returned: %d %c\n",result[3], result[3]);
			r = 0;
			break;
	}
	delete [] result;
	return r;
}

int CyberGlove::instantRead()
{
	port->writeString("G");

	char result[N_SENSOR_VALUES + 5];
	int nRead = port->readTerminatedString(N_SENSOR_VALUES + 5, result);
	if (nRead != N_SENSOR_VALUES) {
		fprintf(stderr,"Wrong number of sensor values returned!\n");
		return 0;
	}
	memcpy(sensorValues, result+1, N_SENSOR_VALUES-1);
	return 1;
}

unsigned char CyberGlove::getSensorValue(int i)
{
	if (i<0 || i>=N_SENSOR_VALUES) {
		fprintf(stderr,"Sensor id out of range!\n");
		return 0;
	}
	return sensorValues[i];	
}