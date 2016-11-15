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

#ifndef _profiling_h_
#define _profiling_h_

/*! \file
	Implements a number of tools for simple, but easy to use and efficient 
	code profiling. It should add a tiny overhead to the caller code, so
	it can be called lots of times.

	Usage example:
	----------------------------
	#define PROF_ENABLED
	#include "profiling.h"

	PROF_DECLARE(TOTAL_TIMER);
	PROF_DECLARE(FOO_TIMER);

	void foo() {
		PROF_TIMER_FUNC(FOO_TIMER);
		for (int i=0; i<1000; i++) {}
	}

	int main(int, char**) {

		PROF_START_TIMER(TOTAL_TIMER);
		for (int i=0; i<100000; i++) {
			for (int j=0; j<2000; j++) {}
			foo();
		}
		PROF_STOP_TIMER(TOTAL_TIMER);

		//Total time in program
		PROF_PRINT(TOTAL_TIMER);
		//Number of calls to and time spent in foo() function
		PROF_PRINT(FOO_TIMER);

		return 0;
	}
	-------------------------------

	Usage "manual":

	The main concept is a "timer", of which you can define as many as you 
	want. When you define a timer, you associate a name with it, and then 
	you can start it, stop it, print it, or have it time a particular 
	function. Any timer will also double as a counter, and you can reset or 
	increment its count whenever you want.

	<ul>
	<li> #include "profiling.h" in any source file. 
	<li> to enable the profiler, also #define PROF_ENABLED just before 
	the include directive. To disable profiling, just remove the PROF_ENABLED 
	definition and you can leave all the other profiler calls in, they will be 
	pre-processed out.
	<li> it is preferable to include your profiling definitions in source, not
	header files, so that the definition PROF_ENABLED doesn't propagate to 
	unexpected places.
	</ul>
	To declare a new timer, use:

	PROF_DECLARE(my_timer_name);

	This can be placed in any source file in your project. Just be sure to place
	timer declarations at the global scope (not inside of any functions). Don't
	worry about namespace pollution, everything profiler-related ends up in its
	own namespace behind the scenes.

	The name then becomes the unique identifier that you can refer to a timer
	through. The main feature of this framework is that you can use literal 
	names, thus making it easy to use. However, behind the scenes they are 
	converted into static ints and add no overhead when they need to be 
	matched against timers. On the other hand, using any undeclared timer is 
	caught at compile-time.

	To use the same timer in a different file than it was declared in, place 
	an extern declaration in the file you use it in:

	PROF_EXTERN(my_timer_name);

	See below macros for what you can do with a timer.
*/

#include <iostream>
#include <vector>
#include <string>
#include "assert.h"

namespace Profiling {
	class Profiler;
	inline Profiler &getProfiler();
}

#ifdef PROF_ENABLED

//declarations
//! Declares a new timer. 
#define PROF_DECLARE(STR) namespace Profiling{extern const int STR = getProfiler().getNewIndex(#STR);}
//! Allows the usage of a timer which was declared (with PROF_DECLARE) in a different file
#define PROF_EXTERN(STR) namespace Profiling{extern const int STR;}

//resetting
//! Resets the timer (both the elapsed time and the reference count)
#define PROF_RESET(STR) Profiling::getProfiler().reset(Profiling::STR);
//! Resets all timers in the system
#define PROF_RESET_ALL Profiling::getProfiler().resetAll();

//counting
//! Increments the timer's reference count by 1
#define PROF_COUNT(STR) Profiling::getProfiler().count(Profiling::STR);

//timing
//! Starts the timer
#define PROF_START_TIMER(STR) Profiling::getProfiler().startTimer(Profiling::STR);
//! Stops the timer and adds the elapsed time since the timer was started to the timer's internal record
#define PROF_STOP_TIMER(STR) Profiling::getProfiler().stopTimer(Profiling::STR);
//! Starts the timer. The timer will automatically stop when the function where this was called goes out of scope.
#define PROF_TIMER_FUNC(STR) Profiling::FunctionTimer fctnTimer__##STR##__(Profiling::getProfiler(),Profiling::STR);

//printing
//! Prints the internal record of a timer
#define PROF_PRINT(STR) Profiling::getProfiler().print(Profiling::STR);
//! Prints all timers in the system
#define PROF_PRINT_ALL Profiling::getProfiler().printAll();

#else

#define PROF_DECLARE(STR) ;
#define PROF_EXTERN(STR) ;
#define PROF_RESET(STR) ;
#define PROF_RESET_ALL ;
#define PROF_COUNT(STR) ;
#define PROF_START_TIMER(STR) ;
#define PROF_STOP_TIMER(STR) ;
#define PROF_TIMER_FUNC(STR) ;
#define PROF_PRINT(STR) ;
#define PROF_PRINT_ALL ;

#endif

//contains all the low-level calls for getting and processing system time
#include "timer_calls.h"

namespace Profiling {

class ProfileInstance 
{
private:
  int mCount;
  bool mRunning;
  std::string mName;
  //! The units here might be different depending on the operating system
  /*! use getTotalTimeMicroseconds() to get total timer time in microseconds. */
  PROF_TIME_UNIT mStartTime;
  PROF_DURATION_UNIT mElapsedTime;
public:
  ProfileInstance();
  void setName(const char *name){mName = name;}

  void count(){mCount++;}
  int getCount(){return mCount;}
  void reset()
  {
    mCount=0;
    PROF_RESET_DURATION(mElapsedTime);
    if (mRunning) 
    {
      PROF_GET_TIME(mStartTime);
    }
  }
  void startTimer()
  {
    if (!mRunning) 
    {
      PROF_GET_TIME(mStartTime);
      mRunning = true;
    } 
    else 
    {
      std::cerr << "Timer " << mName << " already running.\n";
    }
  }
  void stopTimer()
  {
    if (mRunning) 
    {
      PROF_TIME_UNIT currentTime;
      PROF_GET_TIME(currentTime);
      PROF_ADD_DURATION( mElapsedTime, mStartTime, currentTime);
      mRunning = false;
    } 
    else 
    {
      std::cerr << "Timer " << mName << " is not running.\n";
    }
  }
  /*! This returns the elapsed time. It does whatever conversion is necessary,
    depending on the OS, to convert to microseconds. If the timer is running
    at the moment when this is called, is also adds the currently ellapsed 
    time.
    
    This call is slower than start or stop timer, so don't abuse it.
  */
  double getTotalTimeMicroseconds();
  void print();
};

class Profiler 
{
private:
  int mNextIndex;
  int mSize;
  std::vector<ProfileInstance> mPI;
#ifdef WIN32
  UINT64 COUNTS_PER_SEC;
#endif
public:
  Profiler();
  ~Profiler();
  
  void resize(int size);
  int getNewIndex(const char *name);

  void count(int index){mPI[index].count();}
  int getCount(int index){return mPI[index].getCount();}
  void reset(int index){mPI[index].reset();}
  void startTimer(int index){mPI[index].startTimer();}
  void stopTimer(int index){mPI[index].stopTimer();}
  void print(int index){mPI[index].print();}
  
  void resetAll();
  void printAll();
#ifdef WIN32
  UINT64 getCountsPerSec(){return COUNTS_PER_SEC;}
#endif
};

Profiler& getProfiler()
{
  //the one and only instance of the profiler
  static Profiler profInstance;
  return profInstance;
 }

class FunctionTimer 
{
private:
  Profiler &mProfiler;
  int mIndex;
public:
 FunctionTimer(Profiler &prof, int index) : mProfiler(prof), mIndex(index) 
 {
   mProfiler.count(mIndex);
   mProfiler.startTimer(mIndex);
 }
  ~FunctionTimer(){mProfiler.stopTimer(mIndex);}
};

}

#endif
