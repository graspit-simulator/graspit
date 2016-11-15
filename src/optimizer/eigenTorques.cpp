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
// $Id: eigenTorques.cpp,v 1.8 2010/03/06 03:40:10 cmatei Exp $
//
//######################################################################

#include "eigenTorques.h"

#include <iostream>
#include <fstream>
#include <limits>

#include "robot.h"
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "grasp.h"
#include "mcGrip.h"

#include "DBase/DBPlanner/grasp.h"
#include "DBase/graspit_db_grasp.h"
#include "DBase/graspit_db_model.h"
#include "DBase/DBPlanner/sql_database_manager.h"
#include "searchState.h"
#include "matrix.h"

#include "debug.h"

/*! For now, also hard-codes in the type of processing that's done.
*/
CGDBGraspProcessor::CGDBGraspProcessor(Hand *h) : mHand(h) 
{
	mDbMgr = graspItGUI->getIVmgr()->getDBMgr();
	mProcessor = new EigenTorqueComputer(mHand);
	mProcessor = new McGripOptimizer(mHand);
	//mProcessor = new McGripAnalyzer(mHand);

	mProcessedGrasps = 0;
	mMaxGrasps = -1;
//	mMaxGrasps = 30;
}

void 
CGDBGraspProcessor::processGrasps(std::vector<db_planner::Grasp*> *grasps)
{
	std::vector<db_planner::Grasp*>::iterator it;
	for (it=grasps->begin(); it!=grasps->end(); it++) {
		if (mMaxGrasps >=0 && mProcessedGrasps >= mMaxGrasps) break;
		//silently assume that the grasps are actually GraspitDBGrasp
		static_cast<GraspitDBGrasp*>(*it)->getFinalGraspPlanningState()->execute();
		/*
		  //hack for a few grasps which seem not to have f-c
		if ((*it)->EpsilonQuality() == 0.00685985 ||
		    (*it)->EpsilonQuality() == 0.00647612 ) {
		  DBGA("Skipping grasp with epsilon quality: " << (*it)->EpsilonQuality());
		  continue;
		}
		*/
		mHand->getWorld()->findAllContacts();
		mHand->getWorld()->updateGrasps();
		int numContacts = mHand->getGrasp()->getNumContacts();
		DBGA("Processing grasp number " << mProcessedGrasps << "; " <<  numContacts << " contacts.");
		mProcessor->processGrasp();

		//hack for the mcgrip optimizer; also transpose the grasp
		bool transposeGrasp = true;
		if (transposeGrasp) {
			//interchange the dofs of the two chain
			double dofs[6];
			mHand->getDOFVals(dofs);
			for (int j=0; j<3; j++) {
				std::swap(dofs[j], dofs[3+j]);
			}
			mHand->forceDOFVals(dofs);
			//rotate the hand
			transf handTran = rotate_transf(M_PI, vec3::Z);
			transf newTran = mHand->getApproachTran().inverse() * handTran * 
							 mHand->getApproachTran() * mHand->getTran();
			mHand->setTran(newTran);
			if (!mHand->getWorld()->noCollision()) {
				DBGA("Transposed grasp in COLLISION!!!");
				assert(0);
			}
			mHand->getWorld()->findAllContacts();
			mHand->getWorld()->updateGrasps();
			if (mHand->getGrasp()->getNumContacts() != numContacts) {
				DBGA("Transposed grasp has " << mHand->getGrasp()->getNumContacts() << " contacts.");
				continue;
				assert(0);
			}
			DBGA("Processing grasp transpose");
			mProcessor->processGrasp();
		}

		mProcessedGrasps ++;
	}
}

void
CGDBGraspProcessor::run()
{
	std::vector<db_planner::Model*> models;
	//get the old-style for which the McGrip grasps work
	mDbMgr->ModelList(&models, db_planner::FilterList::OLD_STYLE_IV);
	std::vector<db_planner::Model*>::iterator it;
	GraspableBody *currentModel = NULL;
	//reset the stats and the results
	mProcessor->reset();
	for (it=models.begin(); it!=models.end(); it++) {
		if (mMaxGrasps >=0 && mProcessedGrasps >= mMaxGrasps) break;

		//delete the previous model; careful, don't delete it
		if (currentModel) {
			mHand->getWorld()->destroyElement(currentModel, false);
			currentModel = NULL;
		}
		GraspitDBModel *gModel = dynamic_cast<GraspitDBModel*>(*it);
		if (!gModel) {
			assert(0);
			continue;
		}
		//load the current model
		if (!gModel->geometryLoaded()) {
			gModel->load(mHand->getWorld());
		}
		gModel->getGraspableBody()->addToIvc();
		mHand->getWorld()->addBody(gModel->getGraspableBody());
		currentModel = gModel->getGraspableBody();
		//get all the grasps
		std::vector<db_planner::Grasp*> grasps;
		if(!mDbMgr->GetGrasps(*gModel,GraspitDBGrasp::getHandDBName(mHand).toStdString(), &grasps)){
			DBGP("Load grasps failed - no grasps found for model " << gModel->ModelName());
			continue;
		}
		//keep the human ones
		std::vector<db_planner::Grasp*>::iterator it2 = grasps.begin();
		while (it2!=grasps.end()) {
			if( QString((*it2)->GetSource().c_str()) == "HUMAN_REFINED") {
				it2++;
			} else {
				delete (*it2);
				it2 = grasps.erase(it2);
			}
		}
		//and process them
		processGrasps(&grasps);
		//clean up
		while (!grasps.empty()) {
			delete grasps.back();
			grasps.pop_back();
		}
	}

	mProcessor->finalize();

	if (currentModel) {
		mHand->getWorld()->destroyElement(currentModel, false);
	}
	while(!models.empty()) {
		DBGA("Deleting model");
		delete models.back();
		DBGA("Model deleted");
		models.pop_back();
	}
}

void
EigenTorqueComputer::reset()
{
	mOptimalTorques.clear();
	mNumOptimal = mNumUnfeasible = mNumErrors = 0;
}

void
EigenTorqueComputer::processGrasp()
{
	int result;
	if (!mHand->isA("McGrip")) {
		Matrix tau(Matrix::ZEROES<Matrix>(mHand->getNumJoints(),1));
		result = mHand->getGrasp()->computeQuasistaticForcesAndTorques(&tau, Grasp::GRASP_FORCE_EXISTENCE);
		if (!result) {
			mOptimalTorques.push_back(tau);
		}
	} else {
		Matrix p(8,1);
		double objVal;
		result = static_cast<McGripGrasp*>(mHand->getGrasp())->tendonAndHandOptimization(&p, objVal);
	}
	if (!result) {
		DBGA("Optimization success");
		mNumOptimal++;
	} else if (result > 0) {
		DBGA("Optimized problem not feasible");
		mNumUnfeasible++;
	} else {
		DBGA("Error in computation!");
		mNumErrors++;
	}
}

void
EigenTorqueComputer::finalize()
{
	DBGA("   Optimal: " << mNumOptimal);
	DBGA("Unfeasible: " << mNumUnfeasible);
	DBGA("    Errors: " << mNumErrors);

	if (!mHand->isA("McGrip")) {
		std::fstream outFile;
		outFile.open("torques.txt", std::ios::out);
		std::vector<Matrix>::iterator it;
		for (it=mOptimalTorques.begin(); it!=mOptimalTorques.end(); it++) {
			outFile << (*it).transposed() << std::endl;
		}
		outFile.close();
		DBGA("Results output to file torques.txt");
	}
}

void
McGripOptimizer::clearList(std::list<Matrix*> &list) {
	while (!list.empty()) {
		delete list.back();
		list.pop_back();
	}
}

void 
McGripOptimizer::printList(const std::list<Matrix*> &list, FILE *fp)
{
	std::list<Matrix*>::const_iterator it;
	for(it = list.begin(); it!=list.end(); it++) {
		(*it)->print(fp);
		fprintf(fp,"\n");
	}
}


void
McGripOptimizer::reset()
{
	clearList(JTD_negI_list);
	clearList(NegB_list);
	clearList(GO_list);
	clearList(FO_list);
	clearList(SO_list);
	clearList(Th_list);
	clearList(lowerBounds_list);
	clearList(upperBounds_list);
}

void
McGripOptimizer::processGrasp()
{
	//get all the contacts from the grasp
	std::list<Contact*> contacts;
	for (int i=0; i<mHand->getGrasp()->getNumContacts(); i++) {
		contacts.push_back(mHand->getGrasp()->getContact(i));
	}
	//get all the joints, the same way as in all optimization routines
	std::list<Joint*> joints;
	for (int c=0; c<mHand->getNumChains(); c++) {
		std::list<Joint*> chainJoints = mHand->getChain(c)->getJoints();
		joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
	}


	//process them like usual in all the optimization routines
	Matrix J(mHand->getGrasp()->contactJacobian(joints, contacts));
	Matrix D(Contact::frictionForceBlockMatrix(contacts));
	Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
	Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
	Matrix G(mHand->getGrasp()->graspMapMatrix(R,D));
	Matrix JTran(J.transposed());
	Matrix JTD(JTran.rows(), D.cols());
	matrixMultiply(JTran, D, JTD);

	//get the routing matrices
	Matrix *B, *a;
	static_cast<McGrip*>(mHand)->getRoutingMatrices(&B, &a);

	//place each in the appropriate list

	Matrix *JTD_negI = new Matrix( Matrix::BLOCKROW<Matrix>(JTD, Matrix::NEGEYE(a->rows(), a->rows())) );
	JTD_negI_list.push_back(JTD_negI);

	Matrix *NegB = new Matrix(*B);
	NegB->multiply(-1.0);
	NegB_list.push_back(NegB);

	Matrix *GO = new Matrix( Matrix::BLOCKROW<Matrix>(G, Matrix::ZEROES<Matrix>(G.rows(), a->rows())) );
	GO_list.push_back(GO);

	Matrix *FO = new Matrix( Matrix::BLOCKROW<Matrix>(F, Matrix::ZEROES<Matrix>(F.rows(), a->rows())) );
	FO_list.push_back(FO);

	Matrix S(1, JTD.cols());
	S.setAllElements(1.0);
	Matrix *SO = new Matrix( Matrix::BLOCKROW<Matrix>(S, Matrix::ZEROES<Matrix>(1, a->rows())) );
	SO_list.push_back(SO);

	Matrix *lowerBounds = new Matrix( Matrix::BLOCKCOLUMN<Matrix>(Matrix::ZEROES<Matrix>(JTD.cols(),1), *a) );
	lowerBounds_list.push_back(lowerBounds);
	Matrix *upperBounds = new Matrix( Matrix::BLOCKCOLUMN<Matrix>(Matrix::MAX_VECTOR(JTD.cols()), *a) );
	upperBounds_list.push_back(upperBounds);

	//the joint displacement matrix
	Matrix *Th;
	static_cast<McGrip*>(mHand)->getJointDisplacementMatrix(&Th);
	Th_list.push_back(Th);

	delete a;
	delete B;
}

/*! This is the overall form of the optimization:

	x = [beta_1 a_1 beta_2 a_2 ... beta_n a_n  l  r  d]^T

	| J_1TD_1 -I                         -B_1 |
	|          J_2TD_2 -I                -B_2 |
	|                    ...                  | = Q
	|                         J_nTD_n -I -B_n |

	| F_1 0                 0 |
	|       F_2 0           0 |
	|             ...       0 | = F
	|                 F_n 0 0 |

	| G_1 0                 0 |
	|       G_2 0           0 |
	|             ...       0 | = G
	|                 G_n 0 0 |

	| 1 0             0 |
	|     1 0         0 |
	|         ...     0 | = S
	|             1 0 0 |

	lowerBounds = [  0  a_1  0  a_2 ...  0  a_n l_min r_min d_min ] ^T
	upperBounds = [ inf a_1 inf a_2 ... inf a_n l_max r_max d_max ] ^T

	Minimize x^T Q^T Q x (joint equilibrium)
	subject to:
	G x = 0 (no resultant wrench on object)
	S x = 1 (some contact force applied to object)
	F x <= 0 (all contact forces inside friction cones)
	lowerBounds <= x <= upperBounds
*/	
void
McGripOptimizer::finalize()
{
	DBGA("Processing " << JTD_negI_list.size() << " matrices");

	int bCols = NegB_list.front()->cols();

	//objective
	SparseMatrix Q( Matrix::BLOCKROW<SparseMatrix>( Matrix::BLOCKDIAG<SparseMatrix>(&JTD_negI_list), 
							Matrix::BLOCKCOLUMN<SparseMatrix>(&NegB_list) ) );

	//equality constraint
	SparseMatrix GO_block( Matrix::BLOCKDIAG<SparseMatrix>(&GO_list) );
	SparseMatrix SO_block( Matrix::BLOCKDIAG<SparseMatrix>(&SO_list) );
	assert(GO_block.cols() == SO_block.cols());

	bool optimize_r_d;

	//version with fixed r and d
	/*
	SparseMatrix LeftHandEq( Matrix::ZEROES<SparseMatrix>(GO_block.rows() + SO_block.rows(), 
							      GO_block.cols() + bCols) );
	Matrix rightHandEq( Matrix::ZEROES<Matrix>(GO_block.rows() + SO_block.rows(), 1) );
	optimize_r_d = false;
	*/

	//version with mobile r and d
	SparseMatrix LeftHandEq( Matrix::ZEROES<SparseMatrix>(GO_block.rows() + SO_block.rows() + 1, 
							      GO_block.cols() + bCols) );
	Matrix rightHandEq( Matrix::ZEROES<Matrix>(GO_block.rows() + SO_block.rows() + 1, 1) );
	optimize_r_d = true;

	LeftHandEq.copySubMatrix(0, 0, GO_block);
	LeftHandEq.copySubMatrix(GO_block.rows(), 0, SO_block);

	for (int i=0; i<SO_block.rows(); i++) {
		rightHandEq.elem( GO_block.rows() + i, 0) = 1.0;
	}
	
	if (optimize_r_d) {
	  Matrix rd(1,2);
	  rd.setAllElements(1.0);
	  //copy to lower right corner
	  LeftHandEq.copySubMatrix( LeftHandEq.rows()-1, LeftHandEq.cols()-2, rd);
	  //constraint: sum of r and d
	  rightHandEq.elem( rightHandEq.rows()-1, 0) = 25.0;
	}

	//inequality constraint
	SparseMatrix FO_block( Matrix::BLOCKDIAG<SparseMatrix>(&FO_list) );

	SparseMatrix LeftHandInEq( Matrix::ZEROES<SparseMatrix>(FO_block.rows(), FO_block.cols() + bCols) );
	LeftHandInEq.copySubMatrix(0, 0, FO_block);

	Matrix rightHandInEq( Matrix::ZEROES<Matrix>(FO_block.rows(), 1) );

	//bounds
	//the bounds on the overall hand parameters
	Matrix lowerParameterBounds(8, 1);
	Matrix upperParameterBounds(8, 1);
	//tendon routing l's
	for (int i=0; i<6; i++) {
		lowerParameterBounds.elem(i,0) = -5.0;
		upperParameterBounds.elem(i,0) =  5.0;
	}
	if (!optimize_r_d) {
	  //joint radius
	  lowerParameterBounds.elem(6,0) = static_cast<McGrip*>(mHand)->getJointRadius();
	  upperParameterBounds.elem(6,0) = static_cast<McGrip*>(mHand)->getJointRadius();
	  //link length
	  lowerParameterBounds.elem(7,0) = static_cast<McGrip*>(mHand)->getLinkLength();
	  upperParameterBounds.elem(7,0) = static_cast<McGrip*>(mHand)->getLinkLength();
	} else {
	  //joint radius
	  lowerParameterBounds.elem(6,0) = 3.0;
	  upperParameterBounds.elem(6,0) = 10.0;
	  //link length
	  lowerParameterBounds.elem(7,0) = 15.0;
	  upperParameterBounds.elem(7,0) = 22.0;
	}

	//overall bounds for the system
	Matrix lowerBounds( Matrix::BLOCKCOLUMN<Matrix>( Matrix::BLOCKCOLUMN<Matrix>(&lowerBounds_list),
							 lowerParameterBounds ) );
	Matrix upperBounds( Matrix::BLOCKCOLUMN<Matrix>( Matrix::BLOCKCOLUMN<Matrix>(&upperBounds_list),
							 upperParameterBounds ) );

	DBGA("Matrices assembled. Q size: " << Q.rows() << " by " << Q.cols());
	/*
	//debug code. printout of all matrices
	FILE *fp = fopen("mcgrip_optimizer.txt","w");
	fprintf(fp, "JTD_negI matrices:\n");
	printList(JTD_negI_list, fp);
	fprintf(fp, "NegB matrices:\n");
	printList(NegB_list, fp);
	fprintf(fp,"Q matrix:\n");
	Q.print(fp);

	fprintf(fp,"GO matrices:\n");
	printList(GO_list, fp);
	fprintf(fp,"Left hand eq. matrix:\n");
	LeftHandEq.print(fp);
	fprintf(fp,"right hand eq. matrix:\n");
	rightHandEq.print(fp);

	fprintf(fp,"FO matrices:\n");
	printList(FO_list, fp);
	fprintf(fp,"Left hand ineq. matrix:\n");
	LeftHandInEq.print(fp);
	fprintf(fp,"right hand ineq. matrix:\n");
	rightHandInEq.print(fp);

	fprintf(fp,"Bounds:\n");
	for(int i=0; i<lowerBounds.rows(); i++) {
		fprintf(fp,"%f <= x <= %f\n",lowerBounds.elem(i,0), upperBounds.elem(i,0));
	}
	fclose(fp);
	*/
	//matrix of unknowns
	Matrix solution(Q.cols(), 1);

	DBGA("Calling solver");
	double objVal;
	int result = factorizedQPSolver(Q,
					LeftHandEq, rightHandEq,
					LeftHandInEq, rightHandInEq,
					lowerBounds, upperBounds,
					solution, &objVal);
	DBGA("Solver complete. Result: " << result << ". Objective: " << objVal);

	//the parameters are the last 8 entries in the solution
	Matrix parameters(8,1);
	parameters.copySubBlock(0, 0, 8, 1, solution, solution.rows()-8, 0);
	DBGA("Parameters:\n" << parameters);

	DBGA("Building joint stiffness optimization matrices");
	//optimize joint stiffnesses
	SparseMatrix QTh( Matrix::BLOCKROW<SparseMatrix>( Q, Matrix::BLOCKCOLUMN<SparseMatrix>(&Th_list) ) );
	SparseMatrix LEqTh( Matrix::BLOCKROW<SparseMatrix>( LeftHandEq, 
							    Matrix::ZEROES<SparseMatrix>(LeftHandEq.rows(), 6) ) );
	SparseMatrix LInEqTh( Matrix::BLOCKROW<SparseMatrix>( LeftHandInEq, 
							      Matrix::ZEROES<SparseMatrix>(LeftHandInEq.rows(), 6) ) );
	//the bounds on the overall hand parameters including joints
	Matrix lowerParameterBoundsTh(14, 1);
	Matrix upperParameterBoundsTh(14, 1);
	//we use the previously optimized values to fix these
	//lowerParameterBoundsTh.copySubMatrix(0, 0, parameters);
	//upperParameterBoundsTh.copySubMatrix(0, 0, parameters);
	//optimize everything at the same time
	lowerParameterBoundsTh.copySubMatrix(0, 0, lowerParameterBounds);
	upperParameterBoundsTh.copySubMatrix(0, 0, upperParameterBounds);

	//for now, stiffnesses just positive
	//also, we can not build with 0 stifness so just put some small value in
	//it seems that returned numbers are in the 1.0 range
	//let's try with 1.0 min and 2.0 max, in practice we might be able to build that
	for (int i=0; i<6; i++) {
		lowerParameterBoundsTh.elem(8+i, 0) = 1.0;
		upperParameterBoundsTh.elem(8+i, 0) = 2.0;
	}
	//overall bounds for the system
	Matrix lowerBoundsTh( Matrix::BLOCKCOLUMN<Matrix>( Matrix::BLOCKCOLUMN<Matrix>(&lowerBounds_list),
							   lowerParameterBoundsTh ) );
	Matrix upperBoundsTh( Matrix::BLOCKCOLUMN<Matrix>( Matrix::BLOCKCOLUMN<Matrix>(&upperBounds_list),
							   upperParameterBoundsTh ) );
	Matrix solutionTh(QTh.cols(), 1);

	DBGA("Calling solver for joint stiffness");
	result = factorizedQPSolver(QTh,
								LEqTh, rightHandEq,
								LInEqTh, rightHandInEq,
								lowerBoundsTh, upperBoundsTh,
								solutionTh, &objVal);
	DBGA("Solver complete. Result: " << result << ". Objective: " << objVal);

	//the parameters are the last 14 entries in the solution
	Matrix parametersTh(14,1);
	parametersTh.copySubBlock(0, 0, 14, 1, solutionTh, solutionTh.rows()-14, 0);
	DBGA("Parameters:\n" << parametersTh);
}

void McGripAnalyzer::processGrasp()
{
	Matrix p(8,1);
	double objVal;
	int result = static_cast<McGripGrasp*>(mHand->getGrasp())->tendonAndHandOptimization(&p, objVal);

	if (result < 0) {
		fprintf(fp,"-1.0\n");
		DBGA("Computation ERROR");
	} else {
		fprintf(fp,"%f\n",objVal);
		DBGA("Unbalanced: " << objVal);
	}
}
