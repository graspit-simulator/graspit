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
// $Id: bodyPropDlg.cpp,v 1.6 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#include <vector>

#include "bodyPropDlg.h"
#include "body.h"

#include "robot.h"
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "qvalidator.h"
#include "collisionStructures.h"
#include "bBox.h"

/*!
  Initializes the interface elements in the dialog box.
  It first determines which bodies will be affected.  If a robot is selected,
  it adds each of the robot links to the vector of affected bodies.
  If more than one body is selected then the dynamic properites are disabled.
  These must be changed one body at a time.  If the dynamic properties are
  enabled a validator is set up for the mass values.  The rest of the
  initialization sets the initial values of the interface elements.  If there
  are multiple bodies selected that have differing values for certain
  properties, the elements each have a way of allowing no change to made to
  that property.  It also saves the original values of the properties in
  case the user cancels their actions.
*/
void BodyPropDlg::init()
{
  int i,c,l;
  World *w=graspItGUI->getIVmgr()->getWorld();
  std::list<WorldElement *> elemList = w->getSelectedElementList();
  std::list<WorldElement *>::iterator ep;
  
  dynBod=NULL;
  massLineEdit->setValidator(new QDoubleValidator(0,1.0e+100,6,this));
  
  for (ep=elemList.begin();ep!=elemList.end();ep++) {
    if ((*ep)->inherits("Body")) bodyVec.push_back((Body *)(*ep));
    else if ((*ep)->inherits("Robot")) {
      Robot *r = (Robot *)(*ep);
      bodyVec.push_back(r->getBase());
      for (c=0;c<r->getNumChains();c++)
	for (l=0;l<r->getChain(c)->getNumLinks();l++)
	  bodyVec.push_back(r->getChain(c)->getLink(l));
    }
  }
  numBodies = bodyVec.size();
  
  if (numBodies > 1) {
    dynamicCheckBox->setEnabled(false);
    axesCheckBox->setEnabled(false);
    dynamicForcesCheckBox->setEnabled(false);
    massLineEdit->setEnabled(false);
  }
  else {
    if (bodyVec[0]->inherits("DynamicBody")) {
      dynBod = (DynamicBody *)bodyVec[0];
      if (dynBod->isDynamic()) {
	axesCheckBox->setChecked(dynBod->axesShown());
	dynamicForcesCheckBox->setChecked(dynBod->dynContactForcesShown());
	massLineEdit->setText(QString::number(dynBod->getMass()));
      }
      dynamicCheckBox->setChecked(dynBod->isDynamic());
    }
    if (!(dynBod && dynBod->isDynamic()))  {
      axesCheckBox->setEnabled(false);
      dynamicForcesCheckBox->setEnabled(false);
      massLineEdit->setEnabled(false);
    }
  }
  
  //save original values in case user cancels
  for (i=0;i<numBodies;i++) {
    origMaterials.push_back(bodyVec[i]->getMaterial());
    origShowFC.push_back(bodyVec[i]->frictionConesShown());
    origTransparencies.push_back(bodyVec[i]->getTransparency());
  }
  if (dynBod) {
    origIsDynamic = dynBod->isDynamic();
    origMass = dynBod->getMass();
    origAxesShown = dynBod->axesShown();
    origDynContactForcesShown = dynBod->dynContactForcesShown();
  }
  else origIsDynamic = false;
  
  for (i=0;i<w->getNumMaterials();i++)
    materialComboBox->insertItem(w->getMaterialName(i));
  int firstMat = bodyVec[0]->getMaterial();
  for (i=1;i<numBodies;i++)
    if (bodyVec[i]->getMaterial() != firstMat) break;
  if (i==numBodies) materialComboBox->setCurrentItem(firstMat);
  else {
    materialComboBox->insertItem(QString("Keep Original"));
    materialComboBox->setCurrentItem(w->getNumMaterials());
  }
  
  bool showFC = bodyVec[0]->frictionConesShown();
  for (i=1;i<numBodies;i++)
    if (bodyVec[i]->frictionConesShown() != showFC) break;
  if (i==numBodies) fcCheckBox->setChecked(showFC);
  else {
    fcCheckBox->setTristate(true);
    fcCheckBox->setNoChange();
  }
  
  float transp = bodyVec[0]->getTransparency();
  for (i=1;i<numBodies;i++)
    if (bodyVec[i]->getTransparency() != transp) break;
  if (i==numBodies)  transparencySlider->setValue((int) (transp*transparencySlider->maxValue()));

  boundingCheckBox->setChecked(false);
  boundingSpinBox->setEnabled(false);
  QObject::connect( boundingCheckBox, SIGNAL(stateChanged(int)), this, SLOT(showBvs()) );
  QObject::connect( boundingSpinBox, SIGNAL(valueChanged(int)), this, SLOT(showBvs()) );
    
}

/*!
  When the transparency slider is changed, the transparency of the affected
  bodies is immediately updated to show the change.
*/
void BodyPropDlg::setTransparency( int val )
{
  float ratio = (float)val/(float)transparencySlider->maxValue();
  for (int i=0;i<numBodies;i++)
    bodyVec[i]->setTransparency(ratio);
}

/*!
  When the showAxes box is toggled, this updates the axes display for the
  selected body immediately.
 */
void BodyPropDlg::setShowAxes( int state)
{
  if (!dynBod) return;
  if (state == QCheckBox::On) dynBod->showAxes(true);
  else if (state == QCheckBox::Off) dynBod->showAxes(false);
}

/*!
  When the show friction cones box is toggled, this updates the cones display
  for the affected bodies immediately.
*/
void BodyPropDlg::setShowFC( int state )
{
  for (int i=0;i<numBodies;i++)
    if (state == QCheckBox::On) bodyVec[i]->showFrictionCones(true);
    else if (state == QCheckBox::Off) bodyVec[i]->showFrictionCones(false);
    else if (state == QCheckBox::NoChange) bodyVec[i]->showFrictionCones(origShowFC[i]);
}

/*!
  If a single body is selected, it can be set to be either a dynamic or static
  body.  If it was a static body and has never been a dynamic body before,
  then, a new DynamicBody is created to replace the original body, and it
  copies the current static body properties.  Other values are set to default
  settings.  Otherwise the useDynamics flag of a DynamicBody can be toggled.
  The lower half of the dialog box relating to dynamic properties is enabled
  and disabled based on the setting of the dynamics check box.
*/
void BodyPropDlg::setDynamic( int state )
{
  if (dynBod) {
    if (state==QCheckBox::On)  dynBod->setUseDynamics(true);
    else if (state == QCheckBox::Off) {
      dynBod->setUseDynamics(false);
      axesCheckBox->setEnabled(false);
      dynamicForcesCheckBox->setEnabled(false);
      massLineEdit->setEnabled(false);
    }
  }
  else if (state==QCheckBox::On) {
	bodyVec[0]->getWorld()->deselectElement(bodyVec[0]);
    dynBod = bodyVec[0]->getWorld()->makeBodyDynamic(bodyVec[0],DynamicBody::defaultMass);
	dynBod->getWorld()->selectElement(dynBod);
    origMass = dynBod->getMass();
    origAxesShown = dynBod->axesShown();
    origDynContactForcesShown = dynBod->dynContactForcesShown();
  }
  if (state == QCheckBox::On) {
    axesCheckBox->setEnabled(true);
    dynamicForcesCheckBox->setEnabled(true);
    massLineEdit->setEnabled(true);
    axesCheckBox->setChecked(dynBod->axesShown());
    dynamicForcesCheckBox->setChecked(dynBod->dynContactForcesShown());
    massLineEdit->setText(QString::number(dynBod->getMass()));
  }
}

/*!
  Toggles the DynamicBody 's show dynamic contact forces flag.
 */
void BodyPropDlg::setShowDynContactForces( int state )
{
  if (!dynBod) return;
  if (state == QCheckBox::On) dynBod->showDynContactForces(true);
  else if (state == QCheckBox::Off) dynBod->showDynContactForces(false);
}

/*!
  This sets the material of all selected bodies to the one chosen in the
  materials list.  If the the last item in the materials list,
  "Keep original" is chosen, this will reset each body to the material it had
  when this dialog box was opened.
*/
void BodyPropDlg::setMaterial( int choice )
{
  World *w=graspItGUI->getIVmgr()->getWorld();
  
  if (choice == w->getNumMaterials()) {
    for (int i=0;i<numBodies;i++) 
      bodyVec[i]->setMaterial(origMaterials[i]);
  }
  else {
    for (int i=0;i<numBodies;i++)
      bodyVec[i]->setMaterial(choice);
  }      
}

/*!
  If the user pushes cancel, then all the body properties will be restored
  to their original values and the dialog box is closed.
 */
void BodyPropDlg::revertAndClose()
{
 //revert to original values
  for (int i=0;i<numBodies;i++) {
    bodyVec[i]->setTransparency(origTransparencies[i]);
    bodyVec[i]->showFrictionCones(origShowFC[i]);
    bodyVec[i]->setMaterial(origMaterials[i]);
  }
  if (dynBod) {
    dynBod->setUseDynamics(origIsDynamic);
    dynBod->setMass(origMass);
    dynBod->showAxes(origAxesShown);
    dynBod->showDynContactForces(origDynContactForcesShown);
  }
  reject();
}

/*! Shows the bounding box hierarchy of the currently selected bodies
	from the collision detection system, up to the depth specified
	in the depth spin box. The bounding boxes are retrieved from the
	bodies themselves, where they are also colored. Then they are 
	attached back to the geometry root of the bodies. This is generally
	for debugging the collision detection system.
*/
void
BodyPropDlg::showBvs()
{
	std::vector<BoundingBox> bvs;
	World *w=graspItGUI->getIVmgr()->getWorld();
	int depth = -1;
	if (!boundingCheckBox->isChecked()) {
		boundingSpinBox->setEnabled(false);
	} else {
		boundingSpinBox->setEnabled(true);
		depth = boundingSpinBox->value();
	}
	for (int i=0; i<numBodies; i++) {
		bvs.clear();
		if (depth >= 0) {
			w->getBvs(bodyVec[i], depth, &bvs);
		}
		bodyVec[i]->setBVGeometry(bvs);
	}
}
