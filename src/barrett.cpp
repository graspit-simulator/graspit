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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: barrett.cpp,v 1.15 2009/06/17 21:06:37 saint Exp $
//
//######################################################################

/*! \file 
  \brief Implements the %Barrett hand class, a specialized hand subclass
 */

#include <QTextStream>

#include "barrett.h"
#include "world.h"

#ifdef HARDWARE_LIB
#include "BarrettHand.h"
#include "BarrettHandThread.h"
#endif

//#define GRASPITDBG
#include "debug.h"

Barrett::~Barrett()
{
#ifdef HARDWARE_LIB
	if (mRealHand) delete mRealHand;
#endif
}

int
Barrett::loadFromXml(const TiXmlElement* root,QString rootPath)
{
  if (Robot::loadFromXml(root, rootPath) == FAILURE) return FAILURE;
  mRealHand = NULL;
  return SUCCESS;
}

/*! For now, the caller can use this interface directly, but in the 
	future all interaction will have to go through the Barrett class.
*/
BarrettHand*
Barrett::getRealHand()
{
#ifdef HARDWARE_LIB
	if (mRealHand) return mRealHand;
	
	//single threaded version
	//mRealHand = new BarrettHand();
    //mRealHand->SetMode(BarrettHand::MODE_RETURN);

	//multi-threaded version
	mRealHand = new BarrettHandThread();
	((BarrettHandThread*)mRealHand)->startThread();

	return mRealHand;
#else
	assert(0);
	return NULL;
#endif
}

/*! Returns true if the last motor command sent to the real hand
	has not finished yet. Attempts to tell when the motors of the real
	hand are uner current, as this affect things such as Flock of
	Birds measurements.
*/
bool
Barrett::isBusy() {
#ifdef HARDWARE_LIB
	if (mRealHand) return mRealHand->isBusy();
#endif
	return false;
}
