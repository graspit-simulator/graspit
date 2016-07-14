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
// $Id: eigenTorques.h,v 1.5 2009/09/12 00:14:17 cmatei Exp $
//
//######################################################################

#ifndef _eigentorques_h_
#define _eigentorques_h_

#include <vector>

#include "matrix.h"

namespace db_planner{
	class DatabaseManager;
	class Grasp;
}
class Hand;
class McGrip;

/*! An interface for processing multiple grasps, usually for running some
	sort of optimization task.
*/
class GraspProcessor {
protected:
	//! The hand used in the optimization
	Hand *mHand;
public:
	GraspProcessor(Hand *h) : mHand(h) {}
	virtual ~GraspProcessor(){}

	//! Resets the processor and prepares it for another list of grasps
	virtual void reset(){}
	//! Called for each grasp in the list
	virtual void processGrasp() = 0;
	//! Called after all the grasps have been processed
	virtual void finalize() = 0;
};

/*! Handles all the details of loading objects from the CGDB, loading grasps
	for them, setting the hand in the grasp posture etc. Then it calls an 
	instance of the GraspProcessor above to do whatever it wants for each
	grasp. After having gone through the entire list of grasps, it calls
	again the finalize() function of its GraspProcessor.
*/
class CGDBGraspProcessor
{
private:
	//! The hand being used
	Hand *mHand;
	//! The instance that does the actual processing
	GraspProcessor* mProcessor;
	//! The maximum number of grasps to be processed. -1 means process all grasps
	int mMaxGrasps;
	//! The number of grasps processed so far
	int mProcessedGrasps;
	//! A pointer to the one and only system-wide database manager
	db_planner::DatabaseManager *mDbMgr;

	//! Executes a bunch of grasps and calls the Processor on each of them
	void processGrasps(std::vector<db_planner::Grasp*> *grasps);
public:
	//! Also gets the system-wide database manager
	CGDBGraspProcessor(Hand *h);
	//! Also resets and deletes the processor
	~CGDBGraspProcessor(){delete mProcessor;}
	//! The main function, runs the entire set of grasps
	void run();
	//! Resets and gets ready for a new batch of grasps
	void reset(){mProcessedGrasps = 0; mProcessor->reset();}
};

/*! Does a GFO for each grasp to compute the optimal joint torques and
	stores the torques for each grasp. Then it prints them to a file.
	It also prints statistics about the results of the optimizations.
*/
class EigenTorqueComputer : public GraspProcessor
{
private:
	int mNumOptimal, mNumUnfeasible, mNumErrors;
	//! Stores the optimal torques for each grasp
	std::vector<Matrix> mOptimalTorques;
public:
	EigenTorqueComputer(Hand *h) : GraspProcessor(h){}
	//! Resets the processor and prepares it for another list of grasps
	virtual void reset();
	//! Called for each grasp in the list
	virtual void processGrasp();
	//! Called after all the grasps have been processed
	virtual void finalize();
};

/*! Computes unbalanced forces for the McGrip for all the 
	grasps, and saves the result to a file.
*/
class McGripAnalyzer : public GraspProcessor
{
private:
	FILE *fp;
public:
	McGripAnalyzer(Hand *h) : GraspProcessor(h) {}

	virtual void reset(){
		fp = fopen("McGripAnalysis.txt","w");
	}
	//! Computes and outputs unbalanced force
	virtual void processGrasp();
	virtual void finalize(){
		fclose(fp);
	}
};

/*! Builds a large optimization problem for the McGrip over a set of
	multiple grasps. This optimization attempts to solve for both 
	contact forces for each grasp and hand construction parameters
	over all grasps that provide the most stable grasps.

	For each grasps, it computes the respective blocks of the 
	optimization equations. Then, in the \a finalize() stage, it puts
	them all together into one big optimization and (hopefully)
	solves it.
*/
class McGripOptimizer : public GraspProcessor
{
private:
	//! A list of the [JTD -I] matrices for each grasp
	std::list<Matrix*> JTD_negI_list;
	//! A list of the -B matrices for each grasp
	std::list<Matrix*> NegB_list;
	//! A list of the delta_Theta matrices for each grasp
	std::list<Matrix*> Th_list;
	//! A list of the [G 0] matrices for each grasp
	std::list<Matrix*> GO_list;
	//! A list of the [F 0] matrices for each grasp
	std::list<Matrix*> FO_list;
	//! A list of the [S 0] matrices for each grasp
	std::list<Matrix*> SO_list;
	//! A list of the lower bounds vector for each grasp
	std::list<Matrix*> lowerBounds_list;
	//! A list of the upper bounds vector for each grasp
	std::list<Matrix*> upperBounds_list;

	//! Clears the list and deletes all matrices inside
	void clearList(std::list<Matrix*> &list);
	//! Prints all matrices in the list into a file
	void printList(const std::list<Matrix*> &list, FILE *fp);
public:
	McGripOptimizer(Hand *h) : GraspProcessor(h) {}
	virtual ~McGripOptimizer(){reset();}

	//! Clears all the lists of matrices
	virtual void reset();

	//! Adds the individual matrices for the current grasp to the respective list
	virtual void processGrasp();

	//! Actually run the optimization
	virtual void finalize();
};

#endif
