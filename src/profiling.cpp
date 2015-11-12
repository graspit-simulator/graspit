//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: profiling.cpp,v 1.6 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#include "profiling.h"

namespace Profiling {

ProfileInstance::ProfileInstance()
{
	mCount = -1;
	mRunning = false;
	mName = "UNNAMED";
}

double 
ProfileInstance::getTotalTimeMicroseconds()
{
	UINT64 totalTime = mElapsedTime;
	if (mRunning) {
		UINT64 currentTime;
		PROF_GET_TIME(currentTime);
		totalTime += currentTime - mStartTime;
	}
	double result;
	PROF_CONVERT_TO_MICROS(totalTime,result);
	return result;
}

void ProfileInstance::print()
{
	std::cerr << mName << ": ";
	if (mCount > 0) std::cerr << "Count is "<< mCount << "; ";
	double totalTime = getTotalTimeMicroseconds();
	if (totalTime > 0) {
		std::cerr << "Time is " << ((float)totalTime)/1000 << "ms";
		if (mRunning) std::cerr << " (still running)";
		std::cerr << "; ";
	}
	std::cerr << std::endl;
}

Profiler::Profiler()
{
	mSize = 0;
	resize(10);
	mNextIndex = 0;
#ifdef WIN32
	LARGE_INTEGER tmp;
	QueryPerformanceFrequency(&tmp);
	COUNTS_PER_SEC = tmp.QuadPart;
	if (COUNTS_PER_SEC==0) {
		std::cerr << "High performance timer not availabled! PROFILER NOT OPERATIONAL.\n";
		COUNTS_PER_SEC = 1;
	}
#endif
}

Profiler::~Profiler()
{
}

void Profiler::resize(int size) 
{
	if (size <= mSize) return;
	mSize = size;
	mPI.resize(mSize,ProfileInstance());
}

int Profiler::getNewIndex(char *name)
{
	if (mNextIndex >= mSize) {
		resize(2*mSize);
	}
	mPI[mNextIndex].setName(name);
	mPI[mNextIndex].reset();
	return mNextIndex++;
}

void Profiler::resetAll()
{
	for (int i=0; i<mNextIndex; i++) {
		mPI[i].reset();
	}
}

void Profiler::printAll()
{
	for (int i=0; i<mNextIndex; i++) {
		mPI[i].print();
	}
}

}
