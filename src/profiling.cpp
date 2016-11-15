//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: profiling.cpp,v 1.6.4.1 2009/07/23 21:18:02 cmatei Exp $
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
  if (mRunning) 
  {
    stopTimer();
    startTimer();
  }
  double result;
  PROF_CONVERT_TO_MICROS(mElapsedTime, result);
  return result;
}

void ProfileInstance::print()
{
  double totalTime = getTotalTimeMicroseconds();
  if (mCount == 0 && totalTime <= 0) return;
  std::cerr << mName << ": ";
  if (mCount > 0) std::cerr << "Count is "<< mCount << "; ";
  if (totalTime > 0) 
  {
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
  if (COUNTS_PER_SEC==0) 
  {
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

int Profiler::getNewIndex(const char *name)
{
  if (mNextIndex >= mSize) 
  {
    resize(2*mSize);
  }
  mPI[mNextIndex].setName(name);
  mPI[mNextIndex].reset();
  return mNextIndex++;
}

void Profiler::resetAll()
{
  for (int i=0; i<mNextIndex; i++) 
  {
    mPI[i].reset();
  }
}

void Profiler::printAll()
{
  for (int i=0; i<mNextIndex; i++) 
  {
    mPI[i].print();
  }
}
  
}
