#include "flockThread.h"
#include <stdio.h>

void
FlockThread::getPosition(double *p)
{
	WaitForSingleObject(mMutex, INFINITE);
	Flock::getPosition(p);
	ReleaseMutex(mMutex);
}

void
FlockThread::getAngles(double *p)
{
	WaitForSingleObject(mMutex, INFINITE);
	Flock::getAngles(p);
	ReleaseMutex(mMutex);
}

void
FlockThread::getRotationMatrix(double *p)
{
	WaitForSingleObject(mMutex, INFINITE);
	Flock::getRotationMatrix(p);
	ReleaseMutex(mMutex);
}

int 
FlockThread::instantRead(int b)
{
	WaitForSingleObject(mMutex, INFINITE);
	int v = Flock::instantRead(b);
	mNewData = true;
	ReleaseMutex(mMutex);
	return v;
}

void
FlockThread::startThread()
{
	mDone = false;
	mNewData = false;
	mThreadHandle = CreateThread(
		NULL,
		0,
		FlockRunThread,
		(void*)this,
		0,
		&mThreadId);
	//only return after first valid read
	while ( !newData() );
}

void
FlockThread::stopThread()
{
	WaitForSingleObject(mMutex, INFINITE);
	mDone = true;
	ReleaseMutex(mMutex);
}

bool
FlockThread::newData()
{
	bool d;
	WaitForSingleObject(mMutex, INFINITE);
	d = mNewData;
	ReleaseMutex(mMutex);
	return d;
}

bool
FlockThread::isDone()
{
	bool d;
	WaitForSingleObject(mMutex, INFINITE);
	d = mDone;
	ReleaseMutex(mMutex);
	return d;
}

bool
FlockThread::isFinished()
{
	if (!mThreadHandle) return true;
	DWORD code;
	GetExitCodeThread(mThreadHandle, &code);
	if (code == STILL_ACTIVE) return false;
	return true;
}

DWORD WINAPI FlockRunThread(void *f)
{
	FlockThread *ft = (FlockThread*)f;
	while (!ft->isDone()) {
		ft->instantRead();
		Sleep(90);
	}
	fprintf(stderr,"Flock thread finished gracefully\n");
	return 0;
}