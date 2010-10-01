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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: 
//
//######################################################################

#include "ui_bodyPropDlg.h"

#include <vector>

class Body;
class DynamicBody;

/*! \file
  \brief Implements the BodyPropDlg, the body properites dialog box. 
*/

/*! \class BodyPropDlg
  \brief Creates and controls the body properties dialog box.

  This class holds all the body properties dialog box interface components
  created in QT designer and the methods for dealing with user interaction.
  
  The dialog box allows the user to change the body properties of the
  currently selected body or bodies.  The physical material, the
  transparency, and whether or not friction cones should be shown, are all
  properties that can be changed for any set of bodies.  If selected body
  is not dynamic (it is an obstacle), it can be made dynamic.  Dynamic bodies
  have other options that can be changed such as its mass, whether axes at
  the center of gravity should be shown, and whether contact forces should be
  drawn during each step of the dynamics.
*/
class BodyPropDlg : public QDialog, public Ui::BodyPropDlgUI
{
	Q_OBJECT
private:
	DynamicBody *dynBod;
	std::vector<Body *> bodyVec;
	int numBodies;
	double origMass;
	bool origAxesShown;
	bool origDynContactForcesShown;
	bool origIsDynamic;
	std::vector<int> origMaterials;
	std::vector<bool> origShowFC;
	std::vector<float> origTransparencies;

	void init();
public:
	BodyPropDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}
public slots:
	void setTransparency( int val );
	void setShowAxes( int state);
	void setShowFC( int state );
	void setDynamic( int state );
	void setShowDynContactForces( int state );
	void setMaterial( int choice );
	void showBvs();
	void revertAndClose();
};
