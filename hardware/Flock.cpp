#include "Flock.h"
#include <stdio.h>
#include <math.h>

/*
	These constants tell us how many bits to discard as noise from either position or angular data
	In the case of position, 7 l.s. bits correspond to ~3mm
*/
#define DISCARD_POSITION_BITS 0
#define DISCARD_MATRIX_BITS 6

void printChars(int nBytes, unsigned char *records)
{
	int i,j;
	unsigned char mask = 0x80;
	for (i=0; i<nBytes; i++) {
		for (j=0; j<8; j++) {
			if ( j%4==0 ) fprintf(stderr," ");
			if (  ( records[i] << j ) & mask )
				fprintf(stderr,"1");
			else
				fprintf(stderr,"0");
		}
		fprintf(stderr,"\n");
	}
}

void printWords(int nBytes, unsigned short *words)
{
	int i,j;
	unsigned short mask = 0x8000;
	for (i=0; i<nBytes; i++) {
		for (j=0; j<16; j++) {
			if ( j%4==0 ) fprintf(stderr," ");
			if ( j%8==0 ) fprintf(stderr," ");
			if (  ( words[i] << j ) & mask )
				fprintf(stderr,"1");
			else
				fprintf(stderr,"0");
		}
		fprintf(stderr,"\n");
	}
}

void discardBits(int nWords, unsigned short *words, int nDiscardBits)
{
	if (nDiscardBits==0) return;

	unsigned short mask = 0xFFFF;
	mask = mask << nDiscardBits;

	for (int i=0; i<nWords; i++) {
//		printWords(1, &words[i]);
		words[i] = words[i] & mask;
//		printWords(1, &words[i]);
	}
//	fprintf(stderr,"\n");
}

void normalize(double &a, double &b, double &c)
{
	double len = sqrt( a*a + b*b + c*c );
	a = a/len;
	b = b/len;
	c = c/len;
}

void normalize (double *m)
{
	normalize( m[0], m[1], m[2] );
	normalize( m[3], m[4], m[5] );
	normalize( m[6], m[7], m[8] );
}
short toSigned(unsigned short w)
{
	short r;
	memcpy(&r, &w, sizeof(short) );
	return r;
}

unsigned short toUnsigned(short w)
{
	unsigned short r;
	memcpy(&r, &w, sizeof(short) );
	return r;
}

Flock::Flock(int n)
{
	nBirds = n;
	if ( n<1 || n>3) {
		fprintf(stderr,"Wrong number of birds requested!\n");
	}

	port = new SerialPort(SerialPort::COM4, 115200);
	if ( port->getError() ) {
		fprintf(stderr,"Flock of Birds failed to initialize serial port\n");
	}
	initFlock();

	//wait for auto-config

	sleep(1*CLOCKS_PER_SEC);
	setOpMode(POS_MAT);
	setAbsoluteFrame();
}

Flock::~Flock()
{
	delete port;
}

int Flock::readAndTranslate(int nBytes, unsigned char *record, unsigned short *words)
{
	if (!port->readNBytes(nBytes,record)) {
		fprintf(stderr,"Flock read error trying to read %d bytes\n",nBytes);
		return 0;
	}

	unsigned char mask = 0x80;
	if ( !(record[0] & mask) ) {
		fprintf(stderr,"Flock instant read error: first bit is zero!\n");
		return 0;
	}

	if (!translateRecord(nBytes, record, words)) {
		fprintf(stderr,"Flock read: translation failed\n");
		return 0;
	}

	return 1;
}

int Flock::instantRead(int b)
{
	if ( b<=0 || b>nBirds) {
		fprintf(stderr,"Wrong bird for instant read (%d)\n",b);
	}
	talkToBird(b);
	int nBytes;
	port->writeString("B");

	switch (currentOpMode) {
		case POS_ANG:
			nBytes = 12;
			break;
		case POS_MAT: 
			nBytes = 24;
			break;
		default:
			fprintf(stderr,"Unknown current OpMode in Flock instant read\n");
			return 0;
	}

	unsigned char *record = new unsigned char[nBytes];
	unsigned short *words = new unsigned short[nBytes / 2];
	if (!readAndTranslate(nBytes, record, words)) return 0;

	switch (currentOpMode) {
		case POS_ANG:
			for (int i=0; i<3; i++) {
				position[i] =  toSigned(words[i]) * 36.0 / 32768.0; //translate to inches
				position[i] = position[i] * 25.4; //convert to millimeters
				if (relativeOrientation) position[i] -= basePosition[i];
				angles[i] = toSigned(words[5-i]) * 180.0 / 32768.0; //degrees
			}
			break;
		case POS_MAT:
			discardBits(3, words, DISCARD_POSITION_BITS);
			for (int i=0; i<3; i++) {
				position[i] =  toSigned(words[i]) * 36.0 / 32768.0; //translate to inches
				position[i] = position[i] * 25.4; //convert to millimeters
				if (relativeOrientation) position[i] -= basePosition[i];
			}
			discardBits(9, &words[3], DISCARD_MATRIX_BITS);
			for (int i=0; i<9; i++) {
				rotationMatrix[i] = toSigned(words[3+i]) * 1.0 / 32768.0; //translate to [-1 .. 1]
			}
			normalize( rotationMatrix );
			break;
	}

	delete [] record;
	delete [] words;
	return 1;
}

int Flock::translateRecord(int nBytes, unsigned char *record, unsigned short *words)
{
	// record comes as a sequence of 2-byte words, with the least signifcant byte first
	if ((nBytes % 2) != 0 ) {
		fprintf(stderr,"Odd number of bytes for translation!\n");
		return 0;
	}
	int i;
	//shift each Least Significant Byte left one bit
	for (i=0; i<nBytes; i+=2) {
		record[i] = record[i] << 1;
	}

	//combine 2-byte words
	int b = nBytes / 2;
	for (i=0; i<b; i++) {
		words[i] = record[2*i] + 256 * record[2*i+1];
		// shift resulting word left one bit
		words[i] = words[i] << 1;
	}
	return 1;
}

int Flock::encodeRecord(int nWords, unsigned char *record, unsigned short *words)
{
	for (int i=0; i<nWords; i++) {
		record[2*i] = words[i] % 256;
		record[2*i+1] = words[i] / 256;
	}
	return 1;
}

int Flock::setOpMode(opMode m)
{
	int i,retval = 0;
	for (i=1; i<=nBirds; i++) {
		talkToBird(i);
		switch(m) {
			case POS_ANG:
				// position / angle
				port->writeString("Y");
				currentOpMode = POS_ANG;
				retval = 1;
				break;
			case POS_MAT:
				// position / matrix
				port->writeString("Z");
				currentOpMode = POS_MAT;
				retval = 1;
				break;
			default:
				fprintf(stderr,"Unknown operation mode for Flock requested\n");
				retval = 0;
		}
	}
	return retval;
}

int Flock::setRelativeFrame(int b)
{
	int i;
	opMode prevOpMode = currentOpMode;

	//talkToBird(b);
	setOpMode(POS_ANG);

	//remove any previous alignments so we get real base frame
	double zeroAng[] = {0,0,0};
	talkToBird(b);
	angleAlign(zeroAng);

	//get base frame; read twice to be sure that un-alignment has worked
	instantRead(b);
	instantRead(b);
	for (i=0; i<3; i++)
		baseAngles[i] = angles[i];

	//align everybody else with base frame
	for (i=1; i<=nBirds; i++) {
		talkToBird(i);
		angleAlign(baseAngles);
	}

	//set base position
	instantRead(b);
	for (i=0; i<3; i++) {
		basePosition[i] = position[i];
	}

	//return to previous operation mode
	if (prevOpMode != POS_ANG)
		setOpMode(prevOpMode);

	relativeOrientation = true;
	return 1;
}

void Flock::angleAlign(double *a)
{
	unsigned short unsAng[3];
	for (int i=0; i<3; i++)
		unsAng[i] = toUnsigned( (short)(a[i] * 32768.0 / 180.0) );

	unsigned char record[6];
	encodeRecord(3, record, unsAng);

	port->writeString("q");
	port->writeNBytes(6,record);
}

int Flock::setAbsoluteFrame()
{
	double zeroAng[] = {0,0,0};
	for (int b=0; b<=nBirds; b++) {
		talkToBird(b);
		angleAlign(zeroAng);
	}
	relativeOrientation = false;
	return 1;
}

void Flock::initFlock()
{
	//change value
	port->writeString("P");

	//parameter number - FBB auto-config
	unsigned char c = 50;
	port->writeNBytes(1, &c);

	//number of birds in 2's complement
	c = nBirds;
	port->writeNBytes(1, &c);
}

void Flock::talkToBird(int b)
{
	if ( b<=0 || b>nBirds) {
		fprintf(stderr,"Wrong bird requested (%d)\n",b);
		return;
	}
	//nothing to do if there's only one bird in the flock
	if (nBirds==1) {
		return;
	}

	unsigned char c = (unsigned char)(16 + 32 + 64 + 128);
	c += (unsigned char) b;
	port->writeNBytes(1, &c);
}

