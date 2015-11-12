#include "CyberGlove.h"
#include "Flock.h"
#include "flockThread.h"
#include <stdio.h>

#include <process.h>

void main()
{
	/*
	CyberGlove *glove = new CyberGlove();

	if ( !glove->testGlove() ) {
		fprintf(stderr,"Glove error!\n");
		delete glove;
		exit(0);
	} else {
		fprintf(stderr,"Glove OK!\n");
	}
	while (1) {
		if ( !glove->instantRead()) {
			fprintf(stderr,"Read error!\n");
			break;
		}
		fprintf(stderr,"Values: %d %d\n",glove->getSensorValue(1), glove->getSensorValue(2));
	}
	delete glove;
	*/
	/*
	Flock *flock = new Flock(1);
	flock->setOpMode(Flock::POS_MAT);
	double p1[3],m1[9];
	while (1) {
		if (!flock->instantRead(1)) {
			fprintf(stderr,"Read error!\n");
			break;
		}
		flock->getPosition(p1);
		flock->getRotationMatrix(m1);
		fprintf(stderr,"X1: %f  Y1: %f  Z1:%f \n",p1[0], p1[1], p1[2]);
		fprintf(stderr,"R1: %f  R2: %f  R3:%f \n",m1[0], m1[1], m1[2]);
	}

	delete flock;
	*/
	FlockThread *flock = new FlockThread(1);
	flock->setOpMode(Flock::POS_MAT);
	double p1[3],m1[9];

//	_beginthread(Foo::run,0,(void*)flock);

	_beginthread(FlockRunThread,0,(void*)flock);

	while (1) {
		flock->getPosition(p1);
		flock->getRotationMatrix(m1);
		fprintf(stderr,"X1: %f  Y1: %f  Z1:%f \n",p1[0], p1[1], p1[2]);
		//fprintf(stderr,"R1: %f  R2: %f  R3:%f \n",m1[0], m1[1], m1[2]);
	}

	delete flock;
}