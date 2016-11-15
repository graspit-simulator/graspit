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
// Author(s): Matei T. Ciocarlie and Hao Dang
//
// $Id: contactExaminerDlg.cpp,v 1.8 2009/07/02 20:59:49 cmatei Exp $
//
//######################################################################

#include "contactExaminerDlg.h"

#include <QFileDialog>

#include "world.h"
#include "robot.h"
#include "contact.h"
#include "grasp.h"
#include "quality.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "body.h"
#include "mainWindow.h"

#include "debug.h"

void ContactExaminerDlg::init()
{
	mWorld = graspItGUI->getIVmgr()->getWorld();
	//check the hand
	if(mWorld->getCurrentHand()){
		mHand = mWorld->getCurrentHand();
		handRadioButton->setEnabled(TRUE);
	}
	//check the object
	if(mWorld->getNumGB() > 0){
		mObject = mWorld->getGB(0);
		objectRadioButton->setEnabled(TRUE);
	}
	if (mHand) {
		handRadioButton->setChecked(true);
	} else if (mObject) {
		objectRadioButton->setChecked(true);
	}
	modeSelected();
}

void ContactExaminerDlg::markButton_clicked()
{
	if(objectRadioButton->isChecked()){
		collectObjectContacts();
	} else if(handRadioButton->isChecked()) {
		collectHandContacts();
	} else {
		return;
	}
	markedLabel->setNum( (int)mMarkedContacts.size() );
}

void ContactExaminerDlg::collectHandContacts()
{
	int f,l;
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;
	Contact *newContact;

	//the distinctive sign of a virtual contact is that it is its own mate...
	mHand->showVirtualContacts(true);
	contactList = mHand->getPalm()->getContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		if ( (*cp)->getMate() != (*cp) ) {
			newContact = new VirtualContact(-1, 0, *cp);
			((VirtualContact*)newContact)->setBody( mHand->getPalm() );
			mMarkedContacts.push_back(newContact);
			mHand->getPalm()->addVirtualContact((VirtualContact*)newContact);
		}
	}

	for(f=0;f<mHand->getNumFingers();f++) {
		for (l=0;l<mHand->getFinger(f)->getNumLinks();l++) {
			contactList = mHand->getFinger(f)->getLink(l)->getContacts();
			for (cp=contactList.begin();cp!=contactList.end();cp++){
				if ( (*cp)->getMate() != (*cp) ) {
					newContact = new VirtualContact(f, l, *cp);
					((VirtualContact*)newContact)->setBody( mHand->getFinger(f)->getLink(l) );
					mMarkedContacts.push_back(newContact);
					mHand->getFinger(f)->getLink(l)->addVirtualContact((VirtualContact*)newContact);
				}
			}
		}
	}
}

void ContactExaminerDlg::collectObjectContacts()
{
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;
	Contact *newContact;
	// only get the real contact, rather than the virtual contacts
	contactList = mObject->getContacts();
	for(cp = contactList.begin(); cp != contactList.end(); ++cp){
		newContact = new VirtualContact(-1, 0, *cp);
		((VirtualContact*)newContact)->setBody(mObject);
		mMarkedContacts.push_back(newContact);
		mObject->addVirtualContact((VirtualContact*)newContact);
	}
}

void ContactExaminerDlg::loadButton_clicked()
{
	QString fn = QFileDialog::getOpenFileName( this, "Select virtual contact files to load",
		QString(getenv("GRASPIT"))+QString("/models/virtual"),"Virtual Contacts (*.vgr)" );
	QStringList::iterator it;
	if(fn.count() == 0)
		return;

	clearButton_clicked();

	if (objectRadioButton->isChecked()) {
		mObject->loadContactData(fn);
	} else if (handRadioButton->isChecked()){
		mHand->loadContactData(fn);
		mHand->showVirtualContacts(true);
	}
}

void ContactExaminerDlg::saveButton_clicked()
{
	QString fn = QFileDialog::getSaveFileName( this, "Select filename", 
		QString(getenv("GRASPIT"))+QString("/models/virtual"),"Virtual Grasp Files (*.vgr)" );
	if ( !fn.isEmpty() ) {
		if (fn.section('.',1).isEmpty())
			fn.append(".vgr");
	} else {
		return;
	}

	FILE *fp = fopen(fn.latin1(), "w");
	if (!fp) {
		fprintf(stderr,"Failed to open file for writing\n");
	}

	if(handRadioButton->isChecked()){
		fprintf(fp,"%s\n",mHand->getName().latin1());
		fprintf(fp,"%d\n",(int)mMarkedContacts.size());
		for (int i=0; i<(int)mMarkedContacts.size(); i++) {
			((VirtualContact*)mMarkedContacts[i])->writeToFile(fp);
		}
	} else if (objectRadioButton->isChecked()){
		fprintf(fp,"%d\n",(int)mMarkedContacts.size());
		for (int i=0; i<(int)mMarkedContacts.size(); i++) {
			((VirtualContactOnObject*)mMarkedContacts[i])->writeToFile(fp);
		}
	}
	fclose(fp);
}

void ContactExaminerDlg::exitButton_clicked()
{
	QDialog::accept();
}

void ContactExaminerDlg::updateButtons()
{
	if (handRadioButton->isChecked()) {
		contactCollectionGroupBox->setTitle(QString("Contact Collection: ") + mHand->getName());
	} else if (objectRadioButton->isChecked()) {
		contactCollectionGroupBox->setTitle(QString("Contact Collection: ") + mObject->getName());
	}
}

void ContactExaminerDlg::updateQualityButton_clicked()
{
	showQuality();
}

void ContactExaminerDlg::showQuality()
{
	double q;
	if (!mQual || !mGrasp) {
		q = 0.0;
	} else {
		mGrasp->update();
		DBGA("Evaluating quality");
		q = mQual->evaluate();
	}
	if (q < 0) q = 0;
	QString qs;
	qs.setNum(q);
	qs.truncate(5);
	qualityLabel->setText(qs);
}

void ContactExaminerDlg::clearButton_clicked()
{
	if(objectRadioButton->isChecked() && mObject->getNumVirtualContacts()>0){
		mObject->breakVirtualContacts();
	} else if(handRadioButton->isChecked()) {
		mHand->getPalm()->breakVirtualContacts();
		for(int i = 0; i < mHand->getNumFingers(); ++i){
			for(int j = 0; j < mHand->getFinger(i)->getNumLinks(); ++j){
				mHand->getFinger(i)->getLink(j)->breakVirtualContacts();
			}
		}
	}
	mMarkedContacts.clear();
	markedLabel->setText("0");
	qualityLabel->setText("0");
}

void ContactExaminerDlg::destroy()
{
	if (mQual) delete mQual;
	if (objectRadioButton->isChecked()) {
		delete mGrasp;
	}
}

void ContactExaminerDlg::showGWSButton_clicked()
{
	mGrasp->update();
	graspItGUI->getMainWindow()->graspCreateProjection(mGrasp);
}

void ContactExaminerDlg::modeSelected()
{
	//0 - nothing
	//1 - hand
	//2 - object
	static int mode = 0;
	int newMode;
	if (handRadioButton->isChecked()) {
		newMode = 1;
	} else if (objectRadioButton->isChecked()) {
		newMode = 2;
	} else {
		newMode = 0;
	}

	if (mode!=newMode) {
		mMarkedContacts.clear();
		markedLabel->setText("0");

		delete mQual;
		mQual = NULL;
		if (mode!=1) {
			delete mGrasp;
			mGrasp = NULL;
		}
		if (newMode == 1) {
			mHand->getGrasp()->setObject(NULL);
			mGrasp = mHand->getGrasp();
		} else if (newMode == 2){
			mGrasp = new Grasp(NULL);
			mGrasp->setObject(mObject);
		}
		mQual = new QualEpsilon( mGrasp, QString("Virtual_grasp_qm"),"L1 Norm");
		mode = newMode;
	}
	updateButtons();
}
