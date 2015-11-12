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
// Author(s):  Hao Dang
//
// $Id: staubliControlDlg.cpp,v 1.1 2009/09/14 18:04:41 hao Exp $
//
//######################################################################

#include "staubliControlDlg.h"

#include <iostream>
#include <ostream>

#include "graspitGUI.h"
#include "world.h"
#include "ivmgr.h"
#include "robot.h"
#include "barrett.h"

#ifdef HARDWARE_LIB
#include "BarrettHand.h"
#include "TX60L.h"
#include "include/soapStub.h"
#endif

void StaubliControlDlg::init(){
	for(int i = 0; i < graspItGUI->getIVmgr()->getWorld()->getNumRobots(); ++i){
		if(graspItGUI->getIVmgr()->getWorld()->getRobot(i)->getName() == QString("TX60L"))
			mStaubliSimulated = graspItGUI->getIVmgr()->getWorld()->getRobot(i);
		else if(graspItGUI->getIVmgr()->getWorld()->getRobot(i)->getName() == QString("Barrett"))
			mBarrettSimulated = (Barrett*)(graspItGUI->getIVmgr()->getWorld()->getRobot(i));
	}
}

void StaubliControlDlg::destroy(){

}

void StaubliControlDlg::exitButton_clicked(){
	this->accept();
}

void StaubliControlDlg::connectToStaubliButton_clicked(){
	//already connected
	if(mStaubliReal){
		std::cout << "already connected" << std::endl;
		return;
	}
	mStaubliReal = new TX60L();
	if(mStaubliReal->Login(staubliAddress->text().toStdString(),
		userName->text().toStdString(),
		password->text().toStdString())){
			std::cout << "connected" << std::endl;
	}else{
		std::cout << "fail to connect" << std::endl;
		delete mStaubliReal;
		mStaubliReal = NULL;
	}
}

void StaubliControlDlg::getStaubliStatusButton_clicked(){
	if(!mStaubliReal)
		return;
	std::vector<double> jnts, pose;
	jnts.resize(6);
	pose.resize(6);
	mStaubliReal->GetRobotJoints(jnts);
	mStaubliReal->GetRobotCartesianPosition(pose);
	char jntStatus[1024], poseStatus[1024];
	sprintf(jntStatus, "joints: %f, %f, %f, %f, %f, %f", jnts[0], jnts[1], jnts[2], jnts[3], jnts[4], jnts[5]);
	staubliJointsLine->setText(QString(jntStatus));
	sprintf(poseStatus, "pose: %f, %f, %f, %f, %f, %f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
	staubliPoseLine->setText(QString(poseStatus));
	//update simulation
	updateTX60L(jnts);
/*	std::cout << mStaubliSimulated->getChain(0)->getLink(mStaubliSimulated->getChain(0)->getNumLinks() - 1)->getTran().rotation().w << " " <<
		mStaubliSimulated->getChain(0)->getLink(mStaubliSimulated->getChain(0)->getNumLinks() - 1)->getTran().rotation().x << " " <<
		mStaubliSimulated->getChain(0)->getLink(mStaubliSimulated->getChain(0)->getNumLinks() - 1)->getTran().rotation().y << " " <<
		mStaubliSimulated->getChain(0)->getLink(mStaubliSimulated->getChain(0)->getNumLinks() - 1)->getTran().rotation().z << std::endl;
*/
}

void StaubliControlDlg::staubliGoButton_clicked(){
	std::vector<double> pose;
	pose.resize(7);
	sscanf(staubliDestination->text().toStdString().c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&pose[0],&pose[1],&pose[2],&pose[3],&pose[4],&pose[5],&pose[6]);

	//calculate the rotation description used by Staubli from quaternion used by GraspIt/CGDB
	Quaternion q(pose[3], pose[4], pose[5], pose[6]);

	transf barrettInEndEffector(Quaternion(0.999822, 0.00780279, 0.00258668, -0.0169571),vec3(-0.234758, 0.880228, 114.899)* 1.0/1000.0);
	transf target(q,vec3(pose[0],pose[1],pose[2]));
	transf t = barrettInEndEffector.inverse();
	transf forwardKinematics = t * target;

	mat3 m;
	m = m.transpose();
	forwardKinematics.rotation().ToRotationMatrix(m);
	ns6__Frame * frame = new ns6__Frame();
	frame->nx = m.element(0,0);
	frame->ny = m.element(0,1);
	frame->nz = m.element(0,2);

	frame->ox = m.element(1,0);
	frame->oy = m.element(1,1);
	frame->oz = m.element(1,1);

	frame->ax = m.element(2,0);
	frame->ay = m.element(2,1);
	frame->az = m.element(2,2);

	double rx, ry, rz;
	mStaubliReal->GetRxRyRzCoord(frame, &rx, &ry, &rz);
	delete frame;

	//whether send or not, update simulation
	std::vector<double> currentJoints,newJoints,newPose;
	newPose.push_back(forwardKinematics.translation().x());
	newPose.push_back(forwardKinematics.translation().y());
	newPose.push_back(forwardKinematics.translation().z());
	newPose.push_back(rx);
	newPose.push_back(ry);
	newPose.push_back(rz);
	currentJoints.resize(6);
	newJoints.resize(6);
	mStaubliReal->GetRobotJoints(currentJoints);
	mStaubliReal->InverseKinematics(newPose, currentJoints, newJoints);
	updateTX60L(newJoints);

	//send command to staubli as well
	if(sendCommandToStaubli->isChecked()){
		mStaubliReal->Power(true);
		mStaubliReal->ResetMotion();
		mStaubliReal->MoveLine(pose);
	}
}

void StaubliControlDlg::initializeBarrettButton_clicked(){
	if(!mBarrettReal){
		std::cout << "No real Barrett found\n";
		return;
	}
	mBarrettReal->HandInitialize(BARRETT_ALL);
	mBarrettReal->smoothSpreadParams();
}

void StaubliControlDlg::getBarrettButton_clicked(){
	if(mBarrettReal){
		std::cout << "Barrett already obtained\n";
		return;
	}
	if(!mBarrettSimulated){
		std::cout << "No simulated Barrett found\n";
		return;
	}
	mBarrettReal = mBarrettSimulated->getRealHand();
	if(mBarrettReal){
		std::cout << "Barrett found\n";
	} else {
		std::cout << "Barrett not found\n";
		return;
	}
}

void StaubliControlDlg::barrettGoButton_clicked(){
	std::vector<double> dofVals;
	dofVals.resize(4);
	sscanf(barrettDestination->text().toStdString().c_str(), "%lf,%lf,%lf,%lf",&dofVals[0],&dofVals[1],&dofVals[2],&dofVals[3]);
	mBarrettSimulated->forceDOFVals(&dofVals[0]);

	if(sendCommandToBarrett->isChecked()){
		mBarrettReal->MoveTogether(&dofVals[0]);
	}
}

void StaubliControlDlg::getBarrettStatusButton_clicked(){
	if(!mBarrettReal){
		std::cout << "No real Barrett hand found\n";
		return;
	}
	int realVals[4];
	realVals[1] = mBarrettReal->GetFingerPosition(BARRETT_FINGER1);
	realVals[2] = mBarrettReal->GetFingerPosition(BARRETT_FINGER2);
	realVals[3] = mBarrettReal->GetFingerPosition(BARRETT_FINGER3);
	realVals[0] = mBarrettReal->GetFingerPosition(BARRETT_SPREAD);

	if(mBarrettReal->GetBreakawayDetected(BARRETT_FINGER1)) {
		int breakawayPosition = mBarrettReal->GetBreakawayPosition(BARRETT_FINGER1);
		fprintf(stderr, "Finger 1 breakaway at %i\n", breakawayPosition);
	}
	if(mBarrettReal->GetBreakawayDetected(BARRETT_FINGER2)) {
		int breakawayPosition = mBarrettReal->GetBreakawayPosition(BARRETT_FINGER2);
		fprintf(stderr, "Finger 2 breakaway at %i\n", breakawayPosition);
	}
	if(mBarrettReal->GetBreakawayDetected(BARRETT_FINGER3)) {
		int breakawayPosition = mBarrettReal->GetBreakawayPosition(BARRETT_FINGER3);
		fprintf(stderr, "Finger 2 breakaway at %i\n", breakawayPosition);
	}

	double dofVals[4];
	dofVals[0] = mBarrettReal->InternalToRadians(BARRETT_SPREAD, realVals[0]);
	dofVals[1] = mBarrettReal->InternalToRadians(BARRETT_FINGER1, realVals[1]);
	dofVals[2] = mBarrettReal->InternalToRadians(BARRETT_FINGER2, realVals[2]);
	dofVals[3] = mBarrettReal->InternalToRadians(BARRETT_FINGER3, realVals[3]);

	char jntStatus[1024], poseStatus[1024];
	sprintf(jntStatus, "joints: %f, %f, %f, %f", dofVals[0], dofVals[1], dofVals[2], dofVals[3]);
	barrettJointsLine->setText(QString(jntStatus));
	sprintf(poseStatus, "pose: %f, %f, %f, %f, %f, %f, %f", mBarrettSimulated->getBase()->getTran().translation().x(),
		mBarrettSimulated->getBase()->getTran().translation().y(),
		mBarrettSimulated->getBase()->getTran().translation().z(),
		mBarrettSimulated->getBase()->getTran().rotation().w,
		mBarrettSimulated->getBase()->getTran().rotation().x,
		mBarrettSimulated->getBase()->getTran().rotation().y,
		mBarrettSimulated->getBase()->getTran().rotation().z);
	barrettPoseLine->setText(QString(poseStatus));
	//update simulation
	mBarrettSimulated->forceDOFVals(dofVals);
}

void StaubliControlDlg::getLocButton_clicked(){
	if(graspItGUI->getIVmgr()->getWorld()->getNumSelectedBodies() < 1) return;
	transf t = graspItGUI->getIVmgr()->getWorld()->getSelectedBody(0)->getTran();
	std::cout << graspItGUI->getIVmgr()->getWorld()->getSelectedBody(0)->getName().toStdString() << std::endl;
	char loc[1024];
	sprintf(loc, "%lf, %lf, %lf", t.translation().x(), t.translation().y(), t.translation().z());
	locLabel->setText(QString(loc));
}

void StaubliControlDlg::updateTX60L(std::vector<double> dofs){
	std::vector<double>stepSize, currentDof;
	currentDof.resize(mStaubliSimulated->getNumDOF());
	mStaubliSimulated->getDOFVals(&currentDof[0]);

	stepSize.resize(mStaubliSimulated->getNumDOF());
	for (int i = 0; i < mStaubliSimulated->getNumDOF(); i ++) {
		stepSize[i] = (currentDof[i] > dofs[i] ? -1.0 : 1.0) * M_PI/3600.0;
	}
	mStaubliSimulated->moveDOFToContacts(&dofs[0], &stepSize[0], true, true);
}


void StaubliControlDlg::barrettCloseButton_clicked(){
	mBarrettReal->Close(BARRETT_ALL_FINGERS);
	mBarrettSimulated->autoGrasp(true);
}

void StaubliControlDlg::barrettOpenButton_clicked(){
	mBarrettReal->Open(BARRETT_ALL_FINGERS);
	std::vector<double> dofs;
	dofs.resize(mBarrettSimulated->getNumDOF());
	for(int i = 0; i < mBarrettSimulated->getNumDOF(); i ++){
		dofs[i] = mBarrettSimulated->getDOF(i)->getMin();
	}
	mBarrettSimulated->forceDOFVals(&dofs[0]);
}