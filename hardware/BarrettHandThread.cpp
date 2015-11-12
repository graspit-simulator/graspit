#include "BarrettHandThread.h"
#include <stdio.h>

BarrettHandThread::BarrettHandThread(){

	mMutex = CreateMutex(NULL, FALSE, NULL);
	mSemaphore = CreateSemaphore(NULL, 0,1,NULL);
	
	mThreadHandle = NULL;
}

BarrettHandThread::~BarrettHandThread() {
	if (mThreadHandle) {
		WaitForSingleObject(mMutex, INFINITE);
		mStatus = DEAD;
		ReleaseMutex(mMutex);
		ReleaseSemaphore(mSemaphore, 1, NULL);
		DWORD code = STILL_ACTIVE;
		while (code == STILL_ACTIVE) {
			GetExitCodeThread(mThreadHandle, &code);
		}
		fprintf(stderr,"Barrett thread has terminated\n");
	}
}

void
BarrettHandThread::MoveTogether(double *destinations) {

	WaitForSingleObject(mMutex, INFINITE);
	if (mStatus == IDLE) {
		mCurrentCommand = MOVE_TOGETHER;
		memcpy(mDof, destinations, 4*sizeof(double));
		mStatus = READY;
		ReleaseSemaphore(mSemaphore, 1, NULL);
	} else {
		fprintf(stderr,"Hand busy; command skipped\n");
	}
	ReleaseMutex(mMutex);
}

void
BarrettHandThread::Open(int fingers) {

	WaitForSingleObject(mMutex, INFINITE);
	if (mStatus == IDLE) {
		mCurrentCommand = OPEN;
		mCommandFingers = fingers;
		mStatus = READY;
		ReleaseSemaphore(mSemaphore, 1, NULL);
	} else {
		fprintf(stderr,"Hand busy; command skipped\n");
	}
	ReleaseMutex(mMutex);
}

void
BarrettHandThread::Close(int fingers) {

	WaitForSingleObject(mMutex, INFINITE);
	if (mStatus == IDLE) {
		mCurrentCommand = CLOSE;
		mCommandFingers = fingers;
		mStatus = READY;
		ReleaseSemaphore(mSemaphore, 1, NULL);
	} else {
		fprintf(stderr,"Hand busy; command skipped\n");
	}
	ReleaseMutex(mMutex);
}

bool
BarrettHandThread::isBusy()
{
	bool b = false;
	WaitForSingleObject(mMutex, INFINITE);
	if (mStatus != IDLE) b = true;
	ReleaseMutex(mMutex);
	return b;
}

void
BarrettHandThread::threadLoop()
{
	fprintf(stderr,"BHT entering loop\n");
	while (mStatus != DEAD) {
		WaitForSingleObject(mSemaphore, INFINITE);
		WaitForSingleObject(mMutex, INFINITE);
		if (mStatus == DEAD) {
			ReleaseMutex(mMutex);
			break;
		}
		if (mStatus != READY) {
			fprintf(stderr,"Incorrect status value in Barrett thread\n");
		}
		mStatus = BUSY;
		ReleaseMutex(mMutex);
		switch(mCurrentCommand) {
			case MOVE_TOGETHER:
				BarrettHand::MoveTogether(mDof);
				break;
			case OPEN:
				BarrettHand::Open(mCommandFingers);
				break;
			case CLOSE:
				BarrettHand::Close(mCommandFingers);
				break;
			default:
				fprintf(stderr,"Unknown threaded command to BarrettThread!\n");
				break;
		}
		//we need to wait some more so that the motors are powered down
		Sleep(300);
		WaitForSingleObject(mMutex, INFINITE);
		if (mStatus != BUSY) {
			fprintf(stderr,"Incorrect status value in Barrett thread\n");
		}
		mStatus = IDLE;
		ReleaseMutex(mMutex);
		//fprintf(stderr,"Barrett thread has processed command\n");
	}
}

void
BarrettHandThread::startThread()
{
	mStatus = IDLE;
	SetMode(BarrettHand::MODE_READBACK);
	mThreadHandle = CreateThread(
		NULL,
		0,
		BarrettRunThread,
		(void*)this,
		0,
		&mThreadId);

}

DWORD WINAPI BarrettRunThread(void *t)
{
	BarrettHandThread *bt = (BarrettHandThread*)t;
	bt->threadLoop();
	return 0;
}