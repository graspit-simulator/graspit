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
// Authors: Steffen Knoop
//          Andrew T. Miller 
//
// $Id: grasp_grasps.cpp,v 1.2 2009/03/25 22:10:05 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_grasps.cc                                          */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the classes finalGraspPosition and plannedGrasp (used in the grasp planner)
*/

/* inventor includes */
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/SoLists.h>
#include "SoArrow.h"
#include "SoTorquePointer.h"
#include "SoComplexShape.h"
#include <Inventor/fields/SoSFFloat.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoTransformSeparator.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/sensors/SoIdleSensor.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/SbColor.h>
#include <Inventor/nodes/SoTransform.h>


#include "body.h"
#include "matvec3D.h"
#include "grasp_coordinates.h"
#include "grasp_directions.h"
#include "grasp_preshape.h"
#include "grasp_visualization.h"
#include "grasp_grasps.h"



/*!
  Initializes an empty planned grasp
*/
plannedGrasp::plannedGrasp(){
    myGraspDir = cartesianGraspDirection();
    myPreshape = PR_None;
    finalGPos = finalGraspPosition();
    myQuality = -1.;
    myCP = NULL;
}

/*!
  Copies the values from plannedGrasp \a in.
*/
plannedGrasp::plannedGrasp(const plannedGrasp& in){
    set_graspDirection(in.get_graspDirection());
    set_fixedFingerDirection(in.get_fixedFingerDirection());
    myQuality = in.get_quality();
    myPreshape = in.get_preshape();
    myGraspableBody = in.get_graspableBody();
    finalGPos = in.get_finalGraspPosition();
    myCP = in.get_graspRepresentation();
}

/*!
  Initializes a plannedGrasp with a given grasp direction \a in .
*/
plannedGrasp::plannedGrasp(cartesianGraspDirection in){
    set_graspDirection(in);
    myPreshape = PR_None;
    myQuality = -1.;
    myCP = NULL;
}

/*!
  Deletes the grasp's visual representation if neccessary.
*/
plannedGrasp::~plannedGrasp(){
    if (myCP != NULL)
	delete myCP;
}

/*!
  Computes a weighted distance measure between the current planned grasp
  and the grasp \a to .
*/
double
plannedGrasp::distanceTo(plannedGrasp to) const {
    return (DIST_DIR_WEIGHT   * myGraspDir.distanceTo(to.get_graspDirection()) +                 
	    DIST_FFD_WEIGHT   * fixedFingerDirection.distanceTo(to.get_fixedFingerDirection()) + 
	    DIST_PRESH_WEIGHT * myPreshape.distanceTo(to.get_preshape())) /                      
	(DIST_DIR_WEIGHT + DIST_FFD_WEIGHT + DIST_PRESH_WEIGHT);
}


/*!
  Returns the current grasp direction.
*/
cartesianGraspDirection 
plannedGrasp::get_graspDirection()const{
    return myGraspDir;
}

/*!
 Copies the current grasp direction from \a in .
 */
void           
plannedGrasp::set_graspDirection(cartesianGraspDirection in){
    myGraspDir.set_point(in.get_point());
    myGraspDir.set_dir(in.get_dir());
    myGraspDir.set_empty(in.get_empty());
    myGraspDir.set_gdType(in.get_gdType());
    return;
}

/*!
  Returns the thumb direction.
*/
cartesian_coordinates 
plannedGrasp::get_fixedFingerDirection()const{
    return fixedFingerDirection;
}

/*!
  Sets the thumb direction.
*/
void           
plannedGrasp::set_fixedFingerDirection(cartesian_coordinates in){
    fixedFingerDirection = in;
    return;
}

/*!
  Returns the final grasp position (after testing).
*/
finalGraspPosition
plannedGrasp::get_finalGraspPosition()const{
    return finalGPos;
}

/*!
  Sets the final grasp position.
*/
void           
plannedGrasp::set_finalGraspPosition(finalGraspPosition in){
    finalGPos = in;
    return;
}

//  bool           
//  plannedGrasp::get_isReachable()const{
//      return isReachable;
//  }

//  void           
//  plannedGrasp::set_isReachable(bool in){
//      isReachable = in;
//      return;
//  }

/*!
  Returns the quality of this grasp.
*/
double         
plannedGrasp::get_quality()const{
    return myQuality;
}

/*!
  Sets the quality of this grasp after it has been evaluated.
*/
void           
plannedGrasp::set_quality(double in){
    myQuality = in;
    return;
}

/*!
  Returns the grasp preshape used for this grasp.
*/
preshape
plannedGrasp::get_preshape()const{
    return myPreshape;
}

/*!
  Sets the grasp preshape to be used for this grasp.
*/
void           
plannedGrasp::set_preshape(preshape in){
    myPreshape = in;
}

/*!
  Returns a pointer to the grasped object.
*/
GraspableBody* 
plannedGrasp::get_graspableBody()const{
    return myGraspableBody;
}

/*!
  Sets the object associated with this grasp.
*/
void           
plannedGrasp::set_graspableBody(GraspableBody* gb){
    myGraspableBody = gb;
    return;
}

/*!
  Returns a pointer to the grasp representation for this grasp.
*/
grasp_representation*
plannedGrasp::get_graspRepresentation()const{
    return myCP;
}

/*!
  Sets the grasp representation that is associated with this grasp.
*/
void           
plannedGrasp::set_graspRepresentation(grasp_representation* gr){
    myCP = gr;
    return;
}

/*!
  Deletes the grasp representation associated with this grasp.
*/
void 
plannedGrasp::remove_graspRepresentation(){
    delete myCP;
    myCP = NULL;
    return;
}

/*!
  Compares the quality of two planned grasps.  Because of STL differences
  on windows and linux, this will return TRUE if the quality of the first
  is greater than the second under windows and the opposite under linux.
*/
#ifdef WIN32
bool
compareGraspQM::operator()(plannedGrasp* &first, plannedGrasp* &second) const
{
    if (first->get_quality() > second->get_quality())
	return true;
    return false;
}
#else
bool
compareGraspQM::operator()(plannedGrasp* &first, plannedGrasp* &second) const
{
    if (first->get_quality() < second->get_quality())
	return true;
    return false;
}
#endif

/*****************/

/*!
  Stub constructor.
*/
finalGraspPosition::finalGraspPosition(){
}

/*!
  Copies finalGraspPosition \a in .
*/
finalGraspPosition::finalGraspPosition(const finalGraspPosition& in){
    finalTran = in.get_finalTran();
    dof = in.get_dof();
}

/*!
  Clears the DOF list.
*/
finalGraspPosition::~finalGraspPosition(){
    dof.clear();
}

/*!
  Returns the final pose of the hand.
*/
transf 
finalGraspPosition::get_finalTran()const{
    return finalTran;
}

/*!
  Sets the final pose of the hand.
*/
void   
finalGraspPosition::set_finalTran(transf in){
    finalTran = in;
    return;
}

/*!
  Returns the list of final DOF values.
*/
std::list<double>
finalGraspPosition::get_dof()   const{
    return dof;
}

/*!
  Adds a DOF value to the final DOF value list.
*/
void
finalGraspPosition::add_dof(double in){
    dof.push_back(in);
}

/*!
  Changes the value of DOF value \a nr, to \a in .
*/
bool   
finalGraspPosition::change_dof(unsigned int nr, double in){
    if (nr<dof.size()){
	std::list<double>::iterator it = dof.begin();
	for (unsigned int i=0; i<nr; i++)
	    it++;
	(*it) = in;
	return true;
    }
    else return false;
}
 

/******************
   Local Variables:
   mode:c++
   End:
******************/







