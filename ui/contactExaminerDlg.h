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
// $Id: contactExaminerDlg.h,v 1.5 2009/07/02 20:59:49 cmatei Exp $
//
//######################################################################

#ifndef _contactexaminerdlg_h_
#define _contactexaminerdlg_h_

#include "ui_contactExaminerDlg.h"

#include <QDialog>
#include <vector>

class Hand;
class World;
class Contact;
class QualityMeasure;
class GraspableBody;
class Grasp;

//todo: this dialog is in need of a re-write

//! Dialog for creating, saving, loading and inspecting virtual contacts
/*! This dialog enables a user to create virtual contacts on a hand, save
	them to a file, load virtual contacts from the file and inspect them.
	It needs a hand and an actual object; the user must create traditional
	contacts between the hand and the object, then use this dialog to mark
	them as virtual. The contacts then continue to exist as virtual contacts
	even after the object has been moved away. The newly created virtual
	contacts can then be saved to a file, or previously saved virtual contacts
	can be loaded from a file.
	
	This dialog can also be used to compute a "virtual" grasp quality of
	all the virtual contacts on the hand, without need for an object. However,
	this functionality has not been tested recently. Overall, this dialog
	is in need of a re-design.

	The same functionality can be applied to objects: you can load an object,
	mark contacts as virtual ot just load pre-saved virtual contacts from
	a file, and then compute grasp quality without needing a hand at all.

	WARNING: this is code under construction, not very robust.
*/
class ContactExaminerDlg : public QDialog, public Ui::ContactExaminerDlgUI
{
	Q_OBJECT
private:
	World *mWorld;
	Hand *mHand;
	GraspableBody * mObject;
	Grasp* mGrasp;
	std::vector<Contact*> mMarkedContacts;
	QualityMeasure *mQual;

	void init();
	void destroy();
	void updateButtons();
	void showQuality();
	void collectHandContacts();
	void collectObjectContacts();

public:
	ContactExaminerDlg(QWidget *parent = 0) : QDialog(parent), 
	  mWorld(NULL), mHand(NULL), mObject(NULL), mGrasp(NULL),mQual(NULL)
	{
		setupUi(this);
		init();
		QObject::connect(handRadioButton, SIGNAL(clicked()), this, SLOT(modeSelected()));
		QObject::connect(objectRadioButton, SIGNAL(clicked()), this, SLOT(modeSelected()));
	}
	~ContactExaminerDlg(){destroy();}

public slots:

	void markButton_clicked();
	void loadButton_clicked();
	void saveButton_clicked();
	void exitButton_clicked();
	void updateQualityButton_clicked();
	void clearButton_clicked();
	void showGWSButton_clicked();
	void modeSelected();
};
#endif
