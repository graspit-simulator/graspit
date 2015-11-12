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
