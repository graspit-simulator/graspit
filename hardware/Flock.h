#ifndef _flock_h_
#define _flock_h_

#include "SerialPort.h"

class Flock {
public:
	enum opMode{POS_ANG, POS_MAT};
private:
	int nBirds;
	SerialPort *port;

	opMode currentOpMode;
	double angles[3];
	double position[3];
	double rotationMatrix[9];

	bool relativeOrientation;
	double basePosition[3];
	double baseAngles[3];

	void angleAlign(double *a);

	int translateRecord(int nBytes, unsigned char *record, unsigned short *words);
	int encodeRecord(int nWords, unsigned char *record, unsigned short *words);
	int readAndTranslate(int nBytes, unsigned char *record, unsigned short *words);

	void talkToBird(int b);
	void initFlock();
public:
	Flock(int n=1);
	~Flock();

	virtual int instantRead(int b=1);

	virtual void getPosition(double *p){memcpy(p, position, 3*sizeof(double));}
	virtual void getAngles(double *p){memcpy(p, angles, 3*sizeof(double));}
	virtual void getRotationMatrix(double *p){memcpy(p, rotationMatrix, 9*sizeof(double));}

	int setOpMode(opMode m);
	int setRelativeFrame(int b=1);
	int setAbsoluteFrame();
};

#endif