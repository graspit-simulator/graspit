#include "windows.h"
#include "BarrettHand.h"

/*
	This class is intended to do blocking Barrett communication, so we know when the hand has finished whatever
	it was doing. However, for now it does not have everything implemented like it should
*/
class BarrettHandThread : public BarrettHand{
protected:
	enum Status{IDLE, READY, BUSY, DEAD};
	enum ThreadCommands{MOVE_TOGETHER, CLOSE, OPEN};

	Status mStatus;
	BarrettHand *mRealHand;
	HANDLE mMutex, mSemaphore, mThreadHandle;
	DWORD mThreadId;

	ThreadCommands mCurrentCommand;
	double mDof[4];
	int mCommandFingers;
	bool mNewCommand;
	bool mHandReady;
	bool mDone;

public:	
	BarrettHandThread();
	~BarrettHandThread();

	void MoveTogether(double *destinations);
	void Open(int fingers);
	void Close(int fingers);
	bool isBusy();

	void threadLoop();
	void startThread();
	void stopThread();
	bool isFinished();
};

DWORD WINAPI BarrettRunThread(void *);
