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
// $Id: debug.h,v 1.9 2009/03/30 14:59:55 cmatei Exp $
//
//######################################################################

/*! \file
	Defines some convenience macros for output and debug, which behave 
	differently depending on whether GRASPITDBG is defined:

	DBGA(msg) always prints the message to std::err

	DBGP(msg) only prints the message if GRASPITDBG is defined, and 
	swallows it otherwise. 

	To use this, include "debug.h" in your source file, and then 
	define GRASPITDBG just before the #include if you want the debug
	output.
*/

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <iostream>

#ifdef GRASPITDBG
#define DBGP(STMT) std::cerr<<STMT<<std::endl;
#define DBGST(STMT) STMT;
#else
#define DBGP(STMT) 
#define DBGST(STMT)
#endif

#define DBGA(STMT) std::cerr<<STMT<<std::endl;

#define PRINT_STAT(STREAM_PTR,STMT) if (STREAM_PTR) *STREAM_PTR << STMT << " ";
#define DBGAF(STREAM,STMT) {STREAM<<STMT<<std::endl; DBGA(STMT)}
#define DBGPF(STREAM,STMT) {STREAM<<STMT<<std::endl; DBGP(STMT)}

#endif
