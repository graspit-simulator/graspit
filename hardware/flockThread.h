#include "flock.h"

class FlockThread : public Flock
{
protected:
	HANDLE mMutex;
	HANDLE mThreadHandle;
	DWORD mThreadId;
	bool mDone;
	bool mNewData;

	virtual int instantRead(int b=1);
	friend DWORD WINAPI FlockRunThread(void*);
public:
	FlockThread(int n=1) : Flock(n){mMutex = CreateMutex(NULL, FALSE, NULL);mThreadHandle = NULL;}

	virtual void getPosition(double *p);
	virtual void getAngles(double *p);
	virtual void getRotationMatrix(double *p);

	bool isDone();
	bool newData();
	void startThread();
	void stopThread();
	bool isFinished();
};

DWORD WINAPI FlockRunThread(void *);

class Foo
{
public:
	static void run(void *){}
};