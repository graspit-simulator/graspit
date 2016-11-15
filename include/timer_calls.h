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

/*! This file implements the macros for reading and converting system time.
  A total of six macros must be implemented:

  PROF_TIME_UNIT  - the data type for storing a time value
  PROF_DURATION_UNIT - the data type for storing an interval or a duration
  PROF_RESET_DURATION(DURATION) - resets a duration to zero
  PROF_GET_TIME(TIME) - gets the current system time
  PROF_ADD_DURATION(DURATION, START_TIME, END_TIME) - adds the interval between two time values to a duration
  PROF_CONVERT_TO_MICROS(DURATION,DOUBLE) - converts a duration to microseconds

  Conversion is separate from read time, since we don't want to do it every time we read time, but only
  when we need to return it.
*/

// ------------------------------------------- WINDOWS -----------------------------------------------

#ifdef WIN32

#define NOMINMAX
#include <windows.h>
#undef NOMINMAX

#define PROF_TIME_UNIT unsigned __int64
#define PROF_DURATION_UNIT unsigned __int64
#define PROF_RESET_DURATION(DURATION) DURATION=0;
#define PROF_ADD_DURATION(DURATION, START_TIME, END_TIME) DURATION += END_TIME - START_TIME; 

// Gets time in the highest resolution available to the CPU, about a nanosecond
#define PROF_GET_TIME(TIME) LARGE_INTEGER tmp;			  \
  QueryPerformanceCounter(&tmp);				  \
  TIME = tmp.QuadPart;
#define PROF_CONVERT_TO_MICROS(DURATION,DOUBLE) DOUBLE = 1.0e6 * ((double)DURATION) / getProfiler().getCountsPerSec();

/*
// Gets time in units of 100 nanoseconds as 2 4-byte words
#define PROF_GET_TIME(TIME)  FILETIME tmp;					\
  GetSystemTimeAsFileTime(&tmp);					\
  TIME = (static_cast<unsigned __int64>(tmp.dwHighDateTime) << 32) | tmp.dwLowDateTime;
#define PROF_CONVERT_TO_MICROS(DURATION, DOUBLE) DOUBLE = 0.1 * DURATION;
*/

// --------------------------------------------------------------------------------------------------
#else
// -------------------------------------------- LINUX -----------------------------------------------


//high-res version (microsecond)
#include <sys/time.h>
#include <sys/types.h>

#define PROF_TIME_UNIT struct timeval
#define PROF_DURATION_UNIT struct timeval
#define PROF_RESET_DURATION(DURATION) DURATION.tv_sec = DURATION.tv_usec = 0;
#define PROF_GET_TIME(TIME) gettimeofday(&TIME, NULL);
#define PROF_ADD_DURATION(DURATION,START,END)	\
  if (END.tv_usec < START.tv_usec) { \
    int nsec = (START.tv_usec - END.tv_usec) / 1000000 + 1; \
    START.tv_usec -= 1000000 * nsec; \
    START.tv_sec += nsec; \
  } \
  if (END.tv_usec - START.tv_usec > 1000000) { \
    int nsec = (END.tv_usec - START.tv_usec) / 1000000; \
    START.tv_usec += 1000000 * nsec; \
    START.tv_sec -= nsec; \
  } \
  DURATION.tv_sec += END.tv_sec - START.tv_sec; \
  DURATION.tv_usec += END.tv_usec - START.tv_usec;
#define PROF_CONVERT_TO_MICROS(DURATION,DOUBLE) DOUBLE = 1.0e6 * DURATION.tv_sec + DURATION.tv_usec;

/*
//version WITH ONLY ONE SECOND RESOLUTION
#include <ctime>
#include <sys/types.h>

#define PROF_TIME_UNIT u_int64_t
#define PROF_DURATION_UNIT u_int64_t
#define PROF_RESET_DURATION(STR) STR = 0;
#define PROF_GET_TIME(STR) STR = time(NULL);
#define PROF_ADD_DURATION(DURATION,START,END) DURATION += END - START; 
#define PROF_CONVERT_TO_DOUBLE(DURATION,DOUBLE) DOUBLE = 1.0e6 * DURATION;
*/
#endif
