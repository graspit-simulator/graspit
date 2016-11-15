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
// $Id: grasp_planner.cpp,v 1.3 2009/04/21 15:14:18 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_planner.cc                                         */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Implements the grasp_planner (part of grasp planner)
*/


/* standard c,c++ includes */
#include <stdio.h>
#include <iostream>


/* inventor includes */
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
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

#ifdef Q_WS_X11
  #include <unistd.h>
#endif

/* graspit includes */
#include "contact.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "matvec3D.h"
#include "grasp.h"

/* STL includes */
#include <list>

/* include grasp planner class */
#include "grasp_coordinates.h"
#include "grasp_directions.h"
#include "grasp_preshape.h"
#include "grasp_visualization.h"
#include "grasp_grasps.h"
#include "grasp_planner.h"

#
/* debugfile */
#ifdef GRASPITDBG
extern FILE *debugfile;
#endif

/* this plans grasps on all primitives in the scene, only for testing purposes */
//#define PLAN_GRASP_ON_ALL_BODIES    


/*!
  Sets the defaults for the planning parameters.
*/
grasp_planner::grasp_planner()
{
    /* get global stuff */
  ivmgr = graspItGUI->getIVmgr();

    myViewer = ivmgr->getViewer();
    my_body  = NULL;

    nr_of_360_deg_steps          = NR_OF_360_DEG_STEPS;
    nr_of_parallel_planes_width  = NR_OF_PARALLEL_PLANES;
    nr_of_parallel_planes_height = NR_OF_PARALLEL_PLANES;
    nr_of_parallel_planes_depth  = NR_OF_PARALLEL_PLANES;
    nr_of_180_deg_grasps         = NR_OF_180_DEG_GRASPS;
    nr_of_grasp_rotations        = NR_OF_GRASP_ROTATIONS;

    parameterMode = DEFAULT_PARAMETER_MODE;
}

/*!
  Stub destructor.
*/
grasp_planner::~grasp_planner()
{
#ifdef GRASPITDBG
    std::cout << "PL_OUT: grasp_planner destroyed." << std::endl;
#endif
}


/*!
  This is the main method that is called from the grasp_manager.
  Given a pointer to a GraspableBody and a pointer to an Inventor scene
  graph containing the shape primitives for the body, it examines each
  primitive individually and calls the various methods in this class to
  create candidate grasps for each primitive.  These are combined into a
  complete grasp list that is returned at the end of the routine.
*/
std::list<plannedGrasp*>
grasp_planner::planIt(GraspableBody* gb,SoGroup *IVPrimitives)
{
#ifdef PLAN_GRASP_ON_ALL_BODIES    
  int numGB = ivmgr->getWorld()->getNumGB();
#endif
  std::list <GraspDirection*> gd_list;
  std::list <plannedGrasp*> graspList;
  std::list <plannedGrasp*> finalList;
  std::list <plannedGrasp*>::iterator gl_it;
  
  my_body = gb;
  IVGeomPrimitives = IVPrimitives;
  
#ifdef PLAN_GRASP_ON_ALL_BODIES    
  /* get graspable body: Get all at the moment */
  for (int num=0; num<numGB; num++){
    my_body = ivmgr->getGB(num); 
#ifdef GRASPITDBG
    std::cout << "PL_OUT: get Object Nr " << num << std::endl;
#endif
#endif
    
    /* Search graspable primitive objects in object tree. */
    SoPathList pl = searchPrimitives(my_body);
    
    int length = pl.getLength();
#ifdef GRASPITDBG
    std::cout << "PL_OUT: " << length << " primitives found." << std::endl;
#endif
    
    if (length){
      /* primitive object found. Now plan a grasp on it */
#ifdef GRASPITDBG
      std::cout << "PL_OUT: Grasp planning started." << std::endl;
#endif
      
      /* determine number of grasps that will be planned to warn
	 the user in case of infinite computing time ... :-)) */
      /*
	I don't use this any more, because the planning parameters 
	are determined later and are thus not known here. It makes 
	no sense to compute the number here... But that's ok because
	the planning step doesn't take long any more (5 seconds for 
	10,000 grasps) */
      /*
	int no = determineNumberOfGrasps(pl);
	std::cout << "PL_OUT: This will be " << no << " grasps." << std::endl;*/
      
      /* plan grasp for every primitive found in the object */
      for (int item=0; item<length; item++){
#ifdef GRASPITDBG
	std::cout << "PL_OUT: Primitive " << item << std::endl;
#endif
	/*
	  Steps for every primitive:
	*/
	
	/*
	  0. Determine the planning parameters for the actual primitive,
	  depending on size and shape. If parameter mode ==0, the values
	  taken from the motif interface. 
	*/
	if (parameterMode){

#ifdef GRASPITDBG
	  std::cout <<
	    "PL_OUT: Setting the planning parameters for this primitive." <<
	    std::endl;
#endif
	  if (!set_planningParametersFromPrimitive(pl[item])) {

#ifdef GRASPITDBG
	    std::cout << "PL_OUT: Didn't work!" << std::endl;
#endif
	  }
	}
	
	/*
	  1. Find all possible grasp directions. Depends only on the
	  primitive.  Output is in local coordinate system.
	*/

#ifdef GRASPITDBG
	std::cout <<
	  "PL_OUT: getPlannedDirections on this primitive." << std::endl;
#endif
	graspList = getPlannedGraspDirections(pl[item]);
	
	/*
	  2. Fix the remaining DoFs according to heuristics. This 
	  includes the grasp preshape and the z orientation of the hand. 
	  If necessary instantiate new grasps. This happens if there are
	  several preshapes for one direction.
	*/

#ifdef GRASPITDBG
	std::cout <<
	  "PL_OUT: compute preshapes for this primitive." << std::endl;
#endif
	computeGraspPreshapes(graspList, pl[item]);
	
	/* Change from local object coordinate system to global one */
	/* This is a HACK! The pathes of primitive and the original
	   object should be computed independently and transformations
	   multiplied 
	*/
#ifdef GRASPITDBG
	std::cout << "PL_OUT: change coordinate system to global." <<
	  std::endl;
#endif
	localToGlobalCoordinates(graspList, pl[item],gb->getTran());
	
#ifdef GRASPITDBG
	std::cout << "PL_OUT: Copy all grasps to final list." << std::endl;
#endif

	for (gl_it=graspList.begin(); gl_it!=graspList.end(); gl_it++){
	  (*gl_it)->set_graspableBody(my_body);
	  
	  /* 
	     This checks if the actual grasp is alread represented in the
	     list; this may happen if the object has more than one primitive.
	     But as this check takes very very very long, it*s probalby
	     better to just check one ore two grasps twice than to wait for
	     hours at the beginnning...
	  */

#if 0
	  if (!existsInList(**gl_it, finalList))
#endif

	    finalList.push_back(new plannedGrasp(**gl_it));
	}
	graspList.clear();
      }

#ifdef GRASPITDBG
      std::cout<<"PL_OUT: Number of grasps " << finalList.size() << std::endl;
#endif
    }

#ifdef GRASPITDBG
    else std::cout<<"PL_OUT: No graspable object found in IVtree" << std::endl;
#endif

#ifdef PLAN_GRASP_ON_ALL_BODIES    
  }
#endif
  
  return finalList;
}

/*!
  This automatically sets the sampling parameters for a given shape
  primitive and a user supplied sampling density value (\a parameterMode ).
  If the \a parameterMode is 0 then the sampling parameters are provided by
  the user and this function returns immediately with \p FALSE value.
  Otherwise larger values will result in larger values of the sampling
  parameters.  Larger primitive dimensions will also result in larger values.
  As long as the parameter mode greater than 0, \p TRUE is returned.
*/
bool
grasp_planner::set_planningParametersFromPrimitive(SoPath* sop)
{
  if (parameterMode <= 0)
    return false;
  
  if (sop->getTail()->isOfType(SoCylinder::getClassTypeId())){
    /*
     * This is a cylinder!
     */
    double radius = ((SoCylinder*)sop->getTail())->radius.getValue();
    double height = ((SoCylinder*)sop->getTail())->height.getValue();
    switch (parameterMode){
    case 1:
      nr_of_parallel_planes_height = 1;
      nr_of_360_deg_steps = (int)(radius/100.0) * 4 + 4;
      nr_of_180_deg_grasps = 1;
      nr_of_grasp_rotations = 2;
      break;
    case 2:
      nr_of_parallel_planes_height = 2 * (int)(height/100.0) + 1;
      nr_of_360_deg_steps = (int)(radius/100.0) * 8 + 8;
      nr_of_180_deg_grasps = 1;
      nr_of_grasp_rotations = 4;
      break;
    case 3:
      nr_of_parallel_planes_height = 2 * (int)(height/80.0) + 3;
      nr_of_360_deg_steps = (int)(radius/50.0) * 16 + 16;
      nr_of_180_deg_grasps = 2;
      nr_of_grasp_rotations = 4;
      break;
    default:
      nr_of_parallel_planes_height = 2 * (int)(height/70.0) + 2 * parameterMode + 1;
      nr_of_360_deg_steps = (int)(radius/50.0) * 24 + 4 * parameterMode;
      nr_of_180_deg_grasps = 2;
      nr_of_grasp_rotations = parameterMode + 4;
      break;
    }
#ifdef GRASPITDBG
    std::cout << "PL_OUT: no of parallel planes in height " << nr_of_parallel_planes_height << std::endl;
    std::cout << "PL_OUT: no of 360 deg samples           " << nr_of_360_deg_steps << std::endl;
    std::cout << "PL_OUT: no of 180 deg grasps            " << nr_of_180_deg_grasps << std::endl;
    std::cout << "PL_OUT: no of grasp rotations           " << nr_of_grasp_rotations << std::endl;
#endif
  }
  else if (sop->getTail()->isOfType(SoCube::getClassTypeId())){
    /*
     * This is a cube! 
     */
    double width = ((SoCube*)sop->getTail())->width.getValue();
    double height = ((SoCube*)sop->getTail())->height.getValue();
    double depth = ((SoCube*)sop->getTail())->depth.getValue();
    switch (parameterMode){
    case 1:
      nr_of_parallel_planes_width = 1;
      nr_of_parallel_planes_height = 1;
      nr_of_parallel_planes_depth = 1;
      nr_of_180_deg_grasps = 1;
      break;
    case 2:
      nr_of_parallel_planes_width = 2 * (int)(width/100.0) + 1;
      nr_of_parallel_planes_height = 2 * (int)(height/100.0) + 1;
      nr_of_parallel_planes_depth = 2 * (int)(depth/100.0) + 1;
      nr_of_180_deg_grasps = 1;
      break;
    case 3:
      nr_of_parallel_planes_width = 2 * (int)(width/50.0) + 1;
      nr_of_parallel_planes_height = 2 * (int)(height/50.0) + 1;
      nr_of_parallel_planes_depth = 2 * (int)(depth/50.0) + 1;
      nr_of_180_deg_grasps = 2;
      break;
    default:
      nr_of_parallel_planes_width = 2 * (int)(width/50.0) + 2 * parameterMode + 1;
      nr_of_parallel_planes_height = 2 * (int)(height/50.0) + 2 * parameterMode + 1;
      nr_of_parallel_planes_depth = 2 * (int)(depth/50.0) + 2 * parameterMode + 1;
      nr_of_180_deg_grasps = 2;
      break;
    }
#ifdef GRASPITDBG
    std::cout << "PL_OUT: no of parallel planes in height " << nr_of_parallel_planes_height << std::endl;
    std::cout << "PL_OUT: no of parallel planes in width  " << nr_of_parallel_planes_width << std::endl;
    std::cout << "PL_OUT: no of parallel planes in depth  " << nr_of_parallel_planes_depth << std::endl;
    std::cout << "PL_OUT: no of 180 deg grasps            " << nr_of_180_deg_grasps << std::endl;
#endif
  }
  else if (sop->getTail()->isOfType(SoCone::getClassTypeId())){
    /*
     * This is a cone! 
     */
    double bottomRadius = ((SoCone*)sop->getTail())->bottomRadius.getValue();
    double height       = ((SoCone*)sop->getTail())->height.getValue();
    double side_height  = sqrt(bottomRadius * bottomRadius + height * height);
    switch (parameterMode){
    case 1:
      nr_of_parallel_planes_height = 1;
      nr_of_360_deg_steps = (int)(bottomRadius/100.0) * 4 + 4;
      nr_of_180_deg_grasps = 1;
      nr_of_grasp_rotations = 2;
      break;
    case 2:
      nr_of_parallel_planes_height = 2 * (int)(side_height/100.0) + 1;
      nr_of_360_deg_steps = (int)(bottomRadius/100.0) * 8 + 8;
      nr_of_180_deg_grasps = 1;
      nr_of_grasp_rotations = 4;
      break;
    case 3:
      nr_of_parallel_planes_height = 2 * (int)(side_height/80.0) + 3;
      nr_of_360_deg_steps = (int)(bottomRadius/50.0) * 16 + 16;
      nr_of_180_deg_grasps = 2;
      nr_of_grasp_rotations = 4;
      break;
    default:
      nr_of_parallel_planes_height = 2 * (int)(side_height/70.0) + 2 * parameterMode + 1;
      nr_of_360_deg_steps = (int)(bottomRadius/50.0) * 24 + 4 * parameterMode;
      nr_of_180_deg_grasps = 2;
      nr_of_grasp_rotations = parameterMode + 4;
      break;
    }
#ifdef GRASPITDBG
    std::cout << "PL_OUT: no of parallel planes in height " << nr_of_parallel_planes_height << std::endl;
    std::cout << "PL_OUT: no of 360 deg samples           " << nr_of_360_deg_steps << std::endl;
    std::cout << "PL_OUT: no of 180 deg grasps            " << nr_of_180_deg_grasps << std::endl;
    std::cout << "PL_OUT: no of grasp rotations           " << nr_of_grasp_rotations << std::endl;
#endif
  }
  else if (sop->getTail()->isOfType(SoSphere::getClassTypeId())){
    /*
     * This is a sphere!
     */
    double radius = ((SoSphere*)sop->getTail())->radius.getValue();
    switch (parameterMode){
    case 1:
      nr_of_360_deg_steps = (int)(radius/100.0) * 4 + 4;
      nr_of_grasp_rotations = 2;
      break;
    case 2:
      nr_of_360_deg_steps = (int)(radius/100.0) * 8 + 8;
      nr_of_grasp_rotations = 4;
      break;
    case 3:
      nr_of_360_deg_steps = (int)(radius/50.0) * 16 + 16;
      nr_of_grasp_rotations = 4;
      break;
    default:
      nr_of_360_deg_steps = (int)(radius/50.0) * 24 + 4 * parameterMode;
      nr_of_grasp_rotations = parameterMode + 4;
      break;
    }
#ifdef GRASPITDBG
    std::cout << "PL_OUT: no of 360 deg samples           " << nr_of_360_deg_steps << std::endl;
    std::cout << "PL_OUT: no of grasp rotations           " << nr_of_grasp_rotations << std::endl;
#endif
  }
  
  return true;
}

/*!
  This returns the number of candidate grasps that will be created for a given
  shape primitive.  It uses the values of sampling parameters to determine the
  total number of grasps.
*/
int
grasp_planner::determineNumberOfGrasps(SoPathList pl){
  int nrOfPrim = pl.getLength();
  int nrOfGrasps = 0;
  
  for (int i=0; i<nrOfPrim; i++){

    if (pl[i]->getTail()->isOfType(SoCylinder::getClassTypeId())){
      /*
       * This is a cylinder!
       */
      nrOfGrasps += nr_of_360_deg_steps * nr_of_parallel_planes_height * nr_of_180_deg_grasps * 2 +
	nr_of_grasp_rotations * 2;
    }
    else if (pl[i]->getTail()->isOfType(SoCube::getClassTypeId())){
      /*
       * This is a cube! 
       */
      nrOfGrasps += nr_of_parallel_planes_width * nr_of_parallel_planes_height * nr_of_180_deg_grasps * 12;
    }
    else if (pl[i]->getTail()->isOfType(SoCone::getClassTypeId())){
      /*
       * This is a cone! 
       */
	nrOfGrasps += nr_of_360_deg_steps * nr_of_parallel_planes_height * nr_of_180_deg_grasps * 2 + 
	    nr_of_360_deg_steps * nr_of_180_deg_grasps * 2 + 
	    nr_of_grasp_rotations * 2;
    }
    else if (pl[i]->getTail()->isOfType(SoSphere::getClassTypeId())){
      /*
       * This is a sphere!
       */
      // This is only an estimate number!
      nrOfGrasps += nr_of_grasp_rotations * nr_of_360_deg_steps * nr_of_360_deg_steps;
    }
  }
  return nrOfGrasps;
}

/*!
  Determines if the grasp \a pg exists in the grasp list \a pgList.  This
  is slow and is not currently used.

  It is also commented out due to linker problem, as the operator - is not
  actually defined.
*/
bool
grasp_planner::existsInList(plannedGrasp pg, std::list<plannedGrasp*> pgList)
{
  /*
  vec3 pDiff;
  std::list<plannedGrasp*>::iterator it;
  // sorry for this! But it looks really impressing, doesn't it?
  for (it=pgList.begin(); it!=pgList.end(); it++){
    if (((pg.get_graspDirection().get_dir()/pg.get_graspDirection().get_dir().len()) %
	 ((*it)->get_graspDirection().get_dir()/(*it)->get_graspDirection().get_dir().len())) == 1.0){
      if((pg.get_graspDirection().get_point() - (*it)->get_graspDirection().get_point()).len() != 0.0)
	pDiff = (pg.get_graspDirection().get_point() - (*it)->get_graspDirection().get_point()) / 
	  (pg.get_graspDirection().get_point() - (*it)->get_graspDirection().get_point()).len();
      else pDiff = (pg.get_graspDirection().get_point() - (*it)->get_graspDirection().get_point());
      if ((pDiff % (pg.get_graspDirection().get_dir()/pg.get_graspDirection().get_dir().len()) == 1.0) ||
	  (((-1)*pDiff) % (pg.get_graspDirection().get_dir()/pg.get_graspDirection().get_dir().len()) == 1.0) ||
	  (pDiff.len() == 0.0)){
	if((pg.get_fixedFingerDirection() - (*it)->get_fixedFingerDirection()).len() < 0.0001)
	  return true;
      }
    }
  }
  */
  pg = pg;
  pgList = pgList;
  assert(0);
  return false;
}

/*! 
  Sets the planning parameters from user supplied values.
*/
bool 
grasp_planner::set_planningParameters(int nr_of_360_deg_steps_in, 
				      int nr_of_parallel_planes_in,
				      int nr_of_180_deg_grasps_in,
				      int nr_of_grasp_rotations_in){
    /* for details see header file */
    if ((nr_of_360_deg_steps_in > 0) &&
	((nr_of_360_deg_steps_in % 2) == 0) &&
	((nr_of_parallel_planes_in > 0) &&
	 ((nr_of_parallel_planes_in % 2) == 1)) &&
	((nr_of_180_deg_grasps_in == 1) ||
	 (nr_of_180_deg_grasps_in == 2)) &&
	(nr_of_grasp_rotations_in > 0)){

	nr_of_360_deg_steps          = nr_of_360_deg_steps_in;
	nr_of_parallel_planes_width  = nr_of_parallel_planes_in;
	nr_of_parallel_planes_height = nr_of_parallel_planes_in;
	nr_of_parallel_planes_depth  = nr_of_parallel_planes_in;
	nr_of_180_deg_grasps         = nr_of_180_deg_grasps_in;
	nr_of_grasp_rotations        = nr_of_grasp_rotations_in;
	return true;
    }
    return false;
}

/*! 
  Returns the current values of the planning parameters.
*/
void 
grasp_planner::get_planningParameters(int& nr_of_360_deg_steps_in, 
				      int& nr_of_parallel_planes_in,
				      int& nr_of_180_deg_grasps_in,
				      int& nr_of_grasp_rotations_in){
    nr_of_360_deg_steps_in   = nr_of_360_deg_steps;
    nr_of_parallel_planes_in = nr_of_parallel_planes_height;
    nr_of_180_deg_grasps_in  = nr_of_180_deg_grasps;
    nr_of_grasp_rotations_in = nr_of_grasp_rotations;
}

/*!
  This searches the primitives scene graph for instances of the 4 different
  types of shape primitives: cylinders, cubes, cones, and spheres.  For each
  primitive it finds it saves the path from the root of the scene graph to
  that node in a path list.  The final list of paths is returned at the end.
*/
SoPathList 
grasp_planner::searchPrimitives(GraspableBody* bg)
{	
  SoPathList pl_ret;
  SoPathList pl;
  SoSearchAction *saction = new SoSearchAction;
  SoGroup *primGeomRoot;
  int j;
  
  bg = NULL;  // bg is not needed but gives an unused parameter warning
  primGeomRoot = IVGeomPrimitives;
  
  /* Search for cylinders */
  saction->setType(SoCylinder::getClassTypeId());
  saction->setInterest(SoSearchAction::ALL);
  saction->apply(primGeomRoot);
  
  pl     = saction->getPaths();
  for (j=0; j<pl.getLength(); j++)
    {
      if (pl[j]->getTail()->isOfType(SoCylinder::getClassTypeId()))
	{
	  pl_ret.append(pl[j]);
	}
    }
  
  /* Search for boxes */
  saction->setType(SoCube::getClassTypeId());
  saction->apply(primGeomRoot);
  
  pl     = saction->getPaths();
  for (j=0; j<pl.getLength(); j++)
    {
      if (pl[j]->getTail()->isOfType(SoCube::getClassTypeId()))
	{
	  pl_ret.append(pl[j]);
	}
      else pl.remove(j);
    }
  
  /* Search for Cones */
  saction->setType(SoCone::getClassTypeId());
  saction->apply(primGeomRoot);
  
  pl     = saction->getPaths();
  for (j=0; j<pl.getLength(); j++)
    {
      if (pl[j]->getTail()->isOfType(SoCone::getClassTypeId()))
	{
	  pl_ret.append(pl[j]);
	}
    }
  
  /* Search for Spheres */
  saction->setType(SoSphere::getClassTypeId());
  saction->apply(primGeomRoot);
  
  pl     = saction->getPaths();
  for (j=0; j<pl.getLength(); j++)
    {
      if (pl[j]->getTail()->isOfType(SoSphere::getClassTypeId()))
	{
	  pl_ret.append(pl[j]);
	}
    }
  
  delete saction;
  return pl_ret;
}

/*!
  This searches the root of the world scene graph for a specific node pointed
  to by \a node.  If it is found, it returns the path from the world root to
  that node.  Otherwise it returns \p NULL.
*/
SoPath*
grasp_planner::getGlobalPath(SoNode *node)
{
  SoPathList pl;
  //SoPath *path;
  SoSearchAction saction;
  
  /* Search for node */
  saction.setNode(node);
  saction.setInterest(SoSearchAction::ALL);
  saction.apply(ivmgr->getWorld()->getIVRoot());
  pl     = saction.getPaths();
  
  //for (int i=0; i<pl.getLength(); i++)
  if (pl.getLength())
    return new SoPath(*pl[0]);
#ifdef GRASPITDBG
  std::cout << "PL_OUT: Global path not found" << std::endl;
#endif
  return NULL;
}

/*!
  Given a list of all the candidate grasps generated for a single shape
  primitive, a path to the shape from the primitives root node, and the
  overall transform for the entire grasped object with respect to the world,
  this will transform each of the grasps to the world coordinate system.
*/
void
grasp_planner::localToGlobalCoordinates(std::list<plannedGrasp*>& graspList,
					SoPath *pathToPrim,
					const transf &objTran)
{
  std::list <plannedGrasp*>::iterator it;
  cartesianGraspDirection cgd;
  SoGetMatrixAction *gma;
  SoTransform *IVprimTran;
  transf primTran;
  
  IVprimTran= new SoTransform;
  IVprimTran->ref();
  
  // Find the transform of the primitive with respect to the object coordinate
  // frame.
  gma = new SoGetMatrixAction(myViewer->getViewportRegion());
  gma->apply(pathToPrim);
  IVprimTran->setMatrix(gma->getMatrix());
  primTran.set(IVprimTran);
  delete gma;
  IVprimTran->unref();
  
  // Perform transform from object coordinates to world coordinates.
  primTran = primTran * objTran;
  
  for (it=graspList.begin(); it!=graspList.end(); it++){
    
    //ATM: This silliness is necessary because Steffen used a vec3 for both
    //the grasp point and grasp dir
    position graspPos = (*it)->get_graspDirection().get_point()+position::ORIGIN;
    cgd.set_point(graspPos*primTran - position::ORIGIN);
    
    cgd.set_dir((*it)->get_graspDirection().get_dir()*primTran);

    // update both the grasp direction and finger direction
    (*it)->set_graspDirection(cgd);
    (*it)->set_fixedFingerDirection((*it)->get_fixedFingerDirection()*primTran);
  }  
}

/*!
  Given a path to a shape primitive, this determines the type of the primitive,
  and calls the appropriate routine to generate grasp directions for that
  primitive type.  These directions are used to create a new list of
  candidate grasps that is returned at the end of this routine.
*/
std::list <plannedGrasp*>
grasp_planner::getPlannedGraspDirections(SoPath* sop)
{
  /* allocate return list */
  std::list <GraspDirection*> gd_list;
  std::list <GraspDirection*>::iterator gd_it;
  std::list <plannedGrasp*> graspList;
  
  if (sop->getTail()->isOfType(SoCylinder::getClassTypeId())){
    /*
     * This is a cylinder! Generate cylinder grasping directions
     */
    gd_list = getCylinderGraspDirections(sop);
  }
  else if (sop->getTail()->isOfType(SoCube::getClassTypeId())){
    /*
     * This is a cube! Generate cube grasping directions
     */
    gd_list = getCubeGraspDirections(sop);
  }
  else if (sop->getTail()->isOfType(SoCone::getClassTypeId())){
    /*
     * This is a cone! Generate cone grasping directions
     */
    gd_list = getConeGraspDirections(sop);
  }
  else if (sop->getTail()->isOfType(SoSphere::getClassTypeId())){
    /*
     * This is a sphere! Generate sphere grasping directions
     */
    gd_list = getSphereGraspDirections(sop);
  }
#ifdef GRASPITDBG
  else std::cout << "PL_OUT: not a valid primitive. " << std::endl;
#endif
  
  /* genereate return list  */
  for (gd_it=gd_list.begin(); gd_it!=gd_list.end(); gd_it++){
    
    /* instantiate new grasp with computed direction
       and put it into return list */
    graspList.push_back(new plannedGrasp(*gd_it));
    delete (*gd_it);
  }
  
  return graspList;
  
}


/*!
  Given a path to a cylinder primitive, this will create a list of grasp
  directions using the heuristics described in the planning paper.  The
  number of grasp directions created depends on the value of the sampling
  parameters.
*/
std::list <GraspDirection*>
grasp_planner::getCylinderGraspDirections(SoPath* sop)
{
  /* allocate return list */
  std::list <GraspDirection*> gd_list;
  std::list <GraspDirection*>::iterator gd_it;
  
  /* get new cartesian grasp direction. All grasp dirs 
     are converted into cartesian, so that return list
     only has cartesian spaces. */
  GraspDirection* cgd;
  
  cylindrical_coordinates ccpoint;
  cylindrical_coordinates ccdir;
  double radius = ((SoCylinder*)sop->getTail())->radius.getValue();
  double height = ((SoCylinder*)sop->getTail())->height.getValue();
  double plane_height;
  
  /* top direction  in (R, phi, Z) */
  cgd = new cartesianGraspDirection;
  ccpoint[0] = 0.0;   ccpoint[1] = 0.0;   ccpoint[2] = height/2 + STARTING_DISTANCE;
  ccdir[0]   = 0.0;   ccdir[1]   = 0.0;   ccdir[2]   = -1.0;
  /* fill structure */
  cgd->set_point(ccpoint.get_pos_cartesian());
  cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
  cgd->set_gdType(GDT_CYL_TOP_BOTTOM);
  /* add it to the list */
  gd_list.push_back(cgd);
  
  /* bottom direction */
  cgd = new cartesianGraspDirection;
  ccpoint[0] = 0.0;   ccpoint[1] = 0.0;   ccpoint[2] = (-1) * (height/2 + STARTING_DISTANCE);
  ccdir[0]   = 0.0;   ccdir[1]   = 0.0;   ccdir[2]   = 1.0;
  /* fill structure */
  cgd->set_point(ccpoint.get_pos_cartesian());
  cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
  cgd->set_gdType(GDT_CYL_TOP_BOTTOM);
  /* add it to the list */
  gd_list.push_back(cgd);
  
  /* radial direction (one DoF left) */
  for (int curr_plane=1; curr_plane<=nr_of_parallel_planes_height; curr_plane++){
    plane_height = -height/2 + curr_plane*(height/(nr_of_parallel_planes_height + 1));
    for (double step=0.0; step <= 2*M_PI-DELTA_360_DEG_ERROR; step+=(2*M_PI)/nr_of_360_deg_steps){
      cgd = new cartesianGraspDirection;
      ccpoint[0] = radius + STARTING_DISTANCE; ccpoint[1] = step; ccpoint[2] = plane_height;
      ccdir[0]   = -1.0;                       ccdir[1]   = 0.0;  ccdir[2]   = 0.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CYL_SIDES);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  
  /* Cylindrical is a bit special, because inventor 
     cylinders are symmetric to the y axis, not to
     the z axis as the coordinate space expects. Thats 
     why theres a transformation done after generating
     the directions. Changing the coordinate space
     would have been too confusing... */
  
  for (gd_it=gd_list.begin(); gd_it!=gd_list.end(); gd_it++){
    /* swap y and z */
    cartesian_coordinates cctmp((*gd_it)->get_point());
    double ytmp = cctmp[1];
    cctmp[1] = cctmp[2];
    cctmp[2] = ytmp;
    (*gd_it)->set_point(cctmp);
    
    cctmp = (*gd_it)->get_dir();
    ytmp = cctmp[1];
    cctmp[1] = cctmp[2];
    cctmp[2] = ytmp;
    (*gd_it)->set_dir(cctmp);
  }
  return gd_list;
}

/*!
  Given a path to a cube primitive, this will create a list of grasp
  directions using the heuristics described in the planning paper.  The
  number of grasp directions created depends on the value of the sampling
  parameters.
*/
std::list <GraspDirection*>
grasp_planner::getCubeGraspDirections(SoPath* sop)
{
  /* allocate return list */
  std::list <GraspDirection*> gd_list;
  GraspDirection* cgd;
  
  /* get new cartesian grasp direction. All grasp dirs 
     are converted into cartesian, so that return list
     only has cartesian spaces. */
  
  cartesian_coordinates ccpoint;
  cartesian_coordinates ccdir;
  double width = ((SoCube*)sop->getTail())->width.getValue();
  double height = ((SoCube*)sop->getTail())->height.getValue();
  double depth = ((SoCube*)sop->getTail())->depth.getValue();
  double plane_heightA;
  double plane_heightB;
  int curr_planeA,curr_planeB; 
  
  /* top direction  in (x,y,z) */
  for (curr_planeA = 1; curr_planeA <= nr_of_parallel_planes_width; curr_planeA++){
    for (curr_planeB = 1; curr_planeB <= nr_of_parallel_planes_depth; curr_planeB++){
      plane_heightA = - width/2 + curr_planeA * (width/(nr_of_parallel_planes_width + 1));
      plane_heightB = - depth/2 + curr_planeB * (depth/(nr_of_parallel_planes_depth + 1));
      
      cgd = new cartesianGraspDirection;
      ccpoint[0] = plane_heightA;   ccpoint[1] = height + STARTING_DISTANCE;  ccpoint[2] = plane_heightB;
      ccdir[0]   = 0.0;             ccdir[1]   = -1.0;                        ccdir[2]   = 0.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CUBE_HEIGHT);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  /* bottom direction */
  for (curr_planeA = 1; curr_planeA <= nr_of_parallel_planes_width; curr_planeA++){
    for (curr_planeB = 1; curr_planeB <= nr_of_parallel_planes_depth; curr_planeB++){
      plane_heightA = - width/2 + curr_planeA * (width/(nr_of_parallel_planes_width + 1));
      plane_heightB = - depth/2 + curr_planeB * (depth/(nr_of_parallel_planes_depth + 1));
      
      cgd = new cartesianGraspDirection;
      ccpoint[0] = plane_heightA;   ccpoint[1] = -1 * (height + STARTING_DISTANCE);  ccpoint[2] = plane_heightB;
      ccdir[0]   = 0.0;             ccdir[1]   = 1.0;                                ccdir[2]   = 0.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CUBE_HEIGHT);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  /* left direction  in (x,y,z) */
  for (curr_planeA = 1; curr_planeA <= nr_of_parallel_planes_height; curr_planeA++){
    for (curr_planeB = 1; curr_planeB <= nr_of_parallel_planes_depth; curr_planeB++){
      plane_heightA = - height/2 + curr_planeA * (height/(nr_of_parallel_planes_height + 1));
      plane_heightB = - depth/2 + curr_planeB * (depth/(nr_of_parallel_planes_depth + 1));
      
      cgd = new cartesianGraspDirection;
      ccpoint[0] = width + STARTING_DISTANCE;   ccpoint[1] = plane_heightA;   ccpoint[2] = plane_heightB;
      ccdir[0]   = -1.0;                        ccdir[1]   = 0.0;   ccdir[2]   = 0.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CUBE_WIDTH);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  /* right direction */
  for (curr_planeA = 1; curr_planeA <= nr_of_parallel_planes_height; curr_planeA++){
    for (curr_planeB = 1; curr_planeB <= nr_of_parallel_planes_depth; curr_planeB++){
      plane_heightA = - height/2 + curr_planeA * (height/(nr_of_parallel_planes_height + 1));
      plane_heightB = - depth/2 + curr_planeB * (depth/(nr_of_parallel_planes_depth + 1));
      
      cgd = new cartesianGraspDirection;
      ccpoint[0] = -1 * (width + STARTING_DISTANCE);   ccpoint[1] = plane_heightA;   ccpoint[2] = plane_heightB;
      ccdir[0]   =  1.0;                               ccdir[1]   = 0.0;   ccdir[2]   = 0.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CUBE_WIDTH);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  /* front direction in (x,y,z) */
  for (curr_planeA = 1; curr_planeA <= nr_of_parallel_planes_width; curr_planeA++){
    for (curr_planeB = 1; curr_planeB <= nr_of_parallel_planes_height; curr_planeB++){
      plane_heightA = - width/2 + curr_planeA * (width/(nr_of_parallel_planes_width + 1));
      plane_heightB = - height/2 + curr_planeB * (height/(nr_of_parallel_planes_height + 1));
      
      cgd = new cartesianGraspDirection;
      ccpoint[0] = plane_heightA;   ccpoint[1] = plane_heightB;   ccpoint[2] = depth + STARTING_DISTANCE;
      ccdir[0]   = 0.0;     ccdir[1]   = 0.0;      ccdir[2]   = -1.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CUBE_DEPTH);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  /* back direction */
  for (curr_planeA = 1; curr_planeA <= nr_of_parallel_planes_width; curr_planeA++){
    for (curr_planeB = 1; curr_planeB <= nr_of_parallel_planes_height; curr_planeB++){
      plane_heightA = - width/2 + curr_planeA * (width/(nr_of_parallel_planes_width + 1));
      plane_heightB = - height/2 + curr_planeB * (height/(nr_of_parallel_planes_height + 1));
      
      cgd = new cartesianGraspDirection;
      ccpoint[0] = plane_heightA;   ccpoint[1] = plane_heightB;   ccpoint[2] = -1 * (depth + STARTING_DISTANCE);
      ccdir[0]   = 0.0;     ccdir[1]   = 0.0;      ccdir[2]   = 1.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CUBE_DEPTH);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  
  return gd_list;
}

/*!
  Given a path to a sphere primitive, this will create a list of grasp
  directions using the heuristics described in the planning paper.  The
  number of grasp directions created depends on the value of the sampling
  parameters.
*/
std::list <GraspDirection*>
grasp_planner::getSphereGraspDirections(SoPath* sop)
{
  /* allocate return list */
  std::list <GraspDirection*> gd_list;
  
  /* get new cartesian grasp direction. All grasp dirs 
     are converted into cartesian, so that return list
     only has cartesian spaces. */
  GraspDirection* cgd;
  
  spherical_coordinates ccpoint;
  spherical_coordinates ccdir;
  double radius = ((SoSphere*)sop->getTail())->radius.getValue();
  double teta, phi;
  for (teta=0.0; teta<=M_PI; teta+=M_PI/(nr_of_360_deg_steps/2)){
    if (teta == 0.0 || teta >= M_PI){
      cgd = new cartesianGraspDirection;
      ccpoint[0] = radius + STARTING_DISTANCE;  ccpoint[1] = teta;   ccpoint[2] = 0.0;
      ccdir[0]   = -1.0;                        ccdir[1]   = 0.0;   ccdir[2]   = 0.0;
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_SPH);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
    else{   
      for (phi=0.0; phi <= 2*M_PI; phi+=(2*M_PI)/((int)(nr_of_360_deg_steps * sin(teta))+1)){
	
	cgd = new cartesianGraspDirection;
	ccpoint[0] = radius + STARTING_DISTANCE;  ccpoint[1] = teta;   ccpoint[2] = phi;
	ccdir[0]   = -1.0;                        ccdir[1]   = 0.0;   ccdir[2]   = 0.0;
	/* fill structure */
	cgd->set_point(ccpoint.get_pos_cartesian());
	cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
	cgd->set_gdType(GDT_SPH);
	/* add it to the list */
	gd_list.push_back(cgd);
      }
    }
  }
  return gd_list;
}

/*!
  Given a path to a cone primitive, this will create a list of grasp
  directions using the heuristics described in the planning paper.  The
  number of grasp directions created depends on the value of the sampling
  parameters.
*/
std::list <GraspDirection*>
grasp_planner::getConeGraspDirections(SoPath* sop)
{
  /* Try to use cylindrical space for cones; lets see what happens... */
  
  /* allocate return list */
  std::list <GraspDirection*> gd_list;
  std::list <GraspDirection*>::iterator gd_it;
  
  /* get new cartesian grasp direction. All grasp dirs 
     are converted into cartesian, so that return list
     only has cartesian spaces. */
  GraspDirection* cgd;
  
  cylindrical_coordinates ccpoint;
  cylindrical_coordinates ccdir;
  double bottomRadius = ((SoCone*)sop->getTail())->bottomRadius.getValue();
  double height       = ((SoCone*)sop->getTail())->height.getValue();
  double side_height  = sqrt(bottomRadius * bottomRadius + height * height);
  double bottomAngle  = atan2(height, bottomRadius);
  double plane_height;
  
  /* top direction  in (R, phi, Z) */
  cgd = new cartesianGraspDirection;
  ccpoint[0] = 0.0;   ccpoint[1] = 0.0;   ccpoint[2] = height/2 + STARTING_DISTANCE;
  ccdir[0]   = 0.0;   ccdir[1]   = 0.0;   ccdir[2]   = -1.0;
  /* fill structure */
  cgd->set_point(ccpoint.get_pos_cartesian());
  cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
  cgd->set_gdType(GDT_CONE_TOP);
  /* add it to the list */
  gd_list.push_back(cgd);
  
  /* bottom direction */
  cgd = new cartesianGraspDirection;
  ccpoint[0] = 0.0;   ccpoint[1] = 0.0;   ccpoint[2] = (-1) * (height/2 + STARTING_DISTANCE);
  ccdir[0]   = 0.0;   ccdir[1]   = 0.0;   ccdir[2]   = 1.0;
  /* fill structure */
  cgd->set_point(ccpoint.get_pos_cartesian());
  cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
  cgd->set_gdType(GDT_CONE_BOTTOM);
  /* add it to the list */
  gd_list.push_back(cgd);
  
  /* radial direction (one DoF left) perpendicular to side plane */
  for (int curr_plane = 1; curr_plane <= nr_of_parallel_planes_height; curr_plane++){
    plane_height = curr_plane * (side_height/(nr_of_parallel_planes_height + 1));
    
    for (double step=0.0; step <= 2*M_PI-DELTA_360_DEG_ERROR; step+=(2*M_PI)/nr_of_360_deg_steps){
      cgd = new cartesianGraspDirection;
      ccpoint[0] = bottomRadius - plane_height * cos(bottomAngle) + STARTING_DISTANCE * sin(bottomAngle);   
      ccpoint[1] = step;   
      ccpoint[2] = -height/2 + plane_height * sin(bottomAngle) + STARTING_DISTANCE * cos(bottomAngle);
      
      ccdir[0]   = (-1) * sin(bottomAngle);                         
      ccdir[1]   = 0.0;    
      ccdir[2]   = (-1) * cos(bottomAngle);
      /* fill structure */
      cgd->set_point(ccpoint.get_pos_cartesian());
      cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
      cgd->set_gdType(GDT_CONE_SIDE_PLANE);
      /* add it to the list */
      gd_list.push_back(cgd);
    }
  }
  
  /* radial direction (one DoF left) towards bottom edge */
  for (double step=0.0; step <= 2*M_PI-DELTA_360_DEG_ERROR; step+=(2*M_PI)/nr_of_360_deg_steps){
    cgd = new cartesianGraspDirection;
    ccpoint[0] = bottomRadius + STARTING_DISTANCE * cos(bottomAngle/2);
    ccpoint[1] = step;   
    ccpoint[2] = -height/2 - STARTING_DISTANCE * sin(bottomAngle/2);
    
    ccdir[0]   = (-1) * cos(bottomAngle/2);                         
    ccdir[1]   = 0.0;    
    ccdir[2]   = sin(bottomAngle/2);
    /* fill structure */
    cgd->set_point(ccpoint.get_pos_cartesian());
    cgd->set_dir(ccdir.get_vec_cartesian(ccpoint));
    cgd->set_gdType(GDT_CONE_EDGE);
    /* add it to the list */
    gd_list.push_back(cgd);
  }
  

  /* Cylindrical space is a bit special, because inventor 
     cylinders are symmetric to the y axis, not to
     the z axis as the coordinate space expects. Thats 
     why theres a transformation done after generating
     the directions. Changing the coordinate space
     would have been too confusing... */
  
  for (gd_it=gd_list.begin(); gd_it!=gd_list.end(); gd_it++){
    /* swap y and z */
    cartesian_coordinates cctmp((*gd_it)->get_point());
    double ytmp = cctmp[1];
    cctmp[1] = cctmp[2];
    cctmp[2] = ytmp;
    (*gd_it)->set_point(cctmp);
    
    cctmp = (*gd_it)->get_dir();
    ytmp = cctmp[1];
    cctmp[1] = cctmp[2];
    cctmp[2] = ytmp;
    (*gd_it)->set_dir(cctmp);
  }
  return gd_list;
}

/*!
  Given a path to a shape primitive, this determines the type of the primitive,
  and calls the appropriate routine to generate grasp preshapes for each grasp
  in the \a graspList.  The graspList that is returned is a new list that may
  have multiple candidate grasps at each of the original grasp directions.
*/
void
grasp_planner::computeGraspPreshapes(std::list<plannedGrasp*>& graspList, SoPath* sop)
{
  std::list<plannedGrasp*> retList;
  
  if (sop->getTail()->isOfType(SoCylinder::getClassTypeId())){
    /*
     * This is a cylinder! Generate cylinder grasping preshapes
     */
    computeCylinderGraspPreshapes(graspList, sop);
  }
  else if (sop->getTail()->isOfType(SoCube::getClassTypeId())){
    /*
     * This is a cube! Generate cylinder grasping preshapes
     */
    computeCubeGraspPreshapes(graspList, sop);
  }
  else if (sop->getTail()->isOfType(SoCone::getClassTypeId())){
    /*
     * This is a cone! Generate cylinder grasping preshapes
     */
    computeConeGraspPreshapes(graspList, sop);
  }
  else if (sop->getTail()->isOfType(SoSphere::getClassTypeId())){
    /*
     * This is a sphere! Generate cylinder grasping preshapes
     */
    computeSphereGraspPreshapes(graspList, sop);
  }
#ifdef GRASPITDBG
  else std::cout << "PL_OUT: not a valid primitive. " << std::endl;
#endif
  
  return;
}

/*!
  Given a path to a cube primitive, and a list of planned grasp directions,
  this will create one or more grasps for each grasp position using different
  thumb directions and a cylindrical grasp preshape.  The number of grasps
  created for a given grasp direction depends on the sampling parameters and
  the dimensions of the primitive.
*/
void
grasp_planner::computeCubeGraspPreshapes(std::list<plannedGrasp*>& graspList, SoPath* sop)
{
  preshape comp_preshape;
  /* ffd: fixed finger direction */
  cartesian_coordinates help;
  cartesian_coordinates ffd;
  plannedGrasp *pg;
  std::list<plannedGrasp*> retList;
  std::list<plannedGrasp*>::iterator it;
  double width =  ((SoCube*)sop->getTail())->width.getValue();
  double height = ((SoCube*)sop->getTail())->height.getValue();
  double depth =  ((SoCube*)sop->getTail())->depth.getValue();
  
  for (it=graspList.begin(); it!=graspList.end(); it++){
    
    /* check which direction the vector points; we need this to compute the 
       cross product in order to obtain the thumb direction */
    if ((*it)->get_graspDirection().get_dir().x() != 0.0)
      help = cartesian_coordinates(0.0, 1.0, 0.0);
    else if ((*it)->get_graspDirection().get_dir().y() != 0.0)
      help = cartesian_coordinates(1.0, 0.0, 0.0);
    else if ((*it)->get_graspDirection().get_dir().z() != 0.0)
      help = cartesian_coordinates(1.0, 0.0, 0.0);
    
    /* For every direction, theres four grasps: Same preshape, 90deg turned angle */
    pg = new plannedGrasp((*it)->get_graspDirection());
    comp_preshape.set_preshapeType(PRESHAPE_CUBE_GRASP);
    pg->set_preshape(comp_preshape);
    ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * help);
    pg->set_fixedFingerDirection(ffd);
    /* check if object isnt too big for this grasp */
    if((ffd[0]!=0.0 && width  <= GP_MAX_CUBE_SIZE && width  >= GP_MIN_CUBE_SIZE) ||
       (ffd[1]!=0.0 && height <= GP_MAX_CUBE_SIZE && height >= GP_MIN_CUBE_SIZE) ||
       (ffd[2]!=0.0 && depth  <= GP_MAX_CUBE_SIZE && depth  >= GP_MIN_CUBE_SIZE)){
      retList.push_back(pg);
    }
    else delete pg;
    
    ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
    pg = new plannedGrasp((*it)->get_graspDirection());
    comp_preshape.set_preshapeType(PRESHAPE_CUBE_GRASP);
    pg->set_preshape(comp_preshape);
    pg->set_fixedFingerDirection(ffd);
    /* check if object isnt too big for this grasp */
    if((ffd[0]!=0.0 && width  <= GP_MAX_CUBE_SIZE && width  >= GP_MIN_CUBE_SIZE) ||
       (ffd[1]!=0.0 && height <= GP_MAX_CUBE_SIZE && height >= GP_MIN_CUBE_SIZE) ||
       (ffd[2]!=0.0 && depth  <= GP_MAX_CUBE_SIZE && depth  >= GP_MIN_CUBE_SIZE)){
      retList.push_back(pg);
    }
    else delete pg;
    
    ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
    if (nr_of_180_deg_grasps == 2){
      pg = new plannedGrasp((*it)->get_graspDirection());
      comp_preshape.set_preshapeType(PRESHAPE_CUBE_GRASP);
      pg->set_preshape(comp_preshape);
      pg->set_fixedFingerDirection(ffd);
      /* check if object isnt too big for this grasp */
      if((ffd[0]!=0.0 && width  <= GP_MAX_CUBE_SIZE && width  >= GP_MIN_CUBE_SIZE) ||
	 (ffd[1]!=0.0 && height <= GP_MAX_CUBE_SIZE && height >= GP_MIN_CUBE_SIZE) ||
	 (ffd[2]!=0.0 && depth  <= GP_MAX_CUBE_SIZE && depth  >= GP_MIN_CUBE_SIZE)){
	retList.push_back(pg);
      }
      else delete pg;
    }
    
    ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
    if (nr_of_180_deg_grasps == 2){
      pg = new plannedGrasp((*it)->get_graspDirection());
      comp_preshape.set_preshapeType(PRESHAPE_CUBE_GRASP);
      pg->set_preshape(comp_preshape);
      pg->set_fixedFingerDirection(ffd);
      /* check if object isnt too big for this grasp */
      if((ffd[0]!=0.0 && width  <= GP_MAX_CUBE_SIZE && width  >= GP_MIN_CUBE_SIZE) ||
	 (ffd[1]!=0.0 && height <= GP_MAX_CUBE_SIZE && height >= GP_MIN_CUBE_SIZE) ||
	 (ffd[2]!=0.0 && depth  <= GP_MAX_CUBE_SIZE && depth  >= GP_MIN_CUBE_SIZE)){
	retList.push_back(pg);
      }
      else delete pg;
    }
  }
  /* delete old graspList */
  for (it=graspList.begin(); it!=graspList.end(); it++){
    delete (*it);
  }
  graspList.clear();
  
  graspList = retList;
  
  return;
}

/*!
  Given a path to a cylinder primitive, and a list of planned grasp directions,
  this will create one or more grasps for each grasp position using different
  thumb directions and either a cylindrical grasp preshape (for side grasps)
  or a spherical grasp preshape (for end grasps).  The number of grasps
  created for a given grasp direction depends on the sampling parameters and
  the dimensions of the primitive.
*/
void
grasp_planner::computeCylinderGraspPreshapes(std::list<plannedGrasp*>& graspList, SoPath* sop)
{
  cartesian_coordinates help1;
  cartesian_coordinates help2;
  cartesian_coordinates ffd;
  preshape comp_preshape;
  plannedGrasp *pg;
  std::list<plannedGrasp*> retList;
  std::list<plannedGrasp*>::iterator it;
  double radius = ((SoCylinder*)sop->getTail())->radius.getValue();
  double height = ((SoCylinder*)sop->getTail())->height.getValue();
  double step;
  
  for (it=graspList.begin(); it!=graspList.end(); it++){
    if ((*it)->get_graspDirection().get_gdType() == GDT_CYL_TOP_BOTTOM){
      if((radius <= GP_MAX_CYL_TBGRASP_DIAMETER) &&
	 (radius >= GP_MIN_CYL_TBGRASP_DIAMETER)){
	
	comp_preshape.set_preshapeType(PRESHAPE_CYLINDER_TOP_BOTTOM_GRASP);
	
	help1 = cartesian_coordinates(1.0, 0.0, 0.0);
	help2 = (*it)->get_graspDirection().get_dir() * help1;
	
	for (int i=0; i< nr_of_grasp_rotations; i++){
	  step = (double)i*2.0*M_PI/(double)nr_of_grasp_rotations;
	  pg = new plannedGrasp((*it)->get_graspDirection());
	  pg->set_preshape(comp_preshape);
	  ffd = cartesian_coordinates(cos(step) * help1 + sin(step) * help2);
	  pg->set_fixedFingerDirection(ffd);
	  retList.push_back(pg);
	}
      }
    }
    else if ((*it)->get_graspDirection().get_gdType() == GDT_CYL_SIDES){
      
      comp_preshape.set_preshapeType(PRESHAPE_CYLINDER_SIDE_GRASP);
      
      /* radial grasp */
      pg = new plannedGrasp((*it)->get_graspDirection());
      pg->set_preshape(comp_preshape);
      ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * cartesian_coordinates(0.0, 1.0, 0.0));
      pg->set_fixedFingerDirection(ffd);
      if((radius <= GP_MAX_CYL_SIDEGRASP_DIAMETER) &&
	 (radius >= GP_MIN_CYL_SIDEGRASP_DIAMETER) &&
	 (height >= GP_MIN_CYL_SIDEGRASP_HEIGHT))
	retList.push_back(pg);
      else delete pg;
      
      /* thumb parallel to height axis */
      pg = new plannedGrasp((*it)->get_graspDirection());
      pg->set_preshape(comp_preshape);
      ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
      pg->set_fixedFingerDirection(ffd);
      if (height <= GP_MAX_CYL_SIDEGRASP_HEIGHT)
	retList.push_back(pg);
      else delete pg;
      
      if (nr_of_180_deg_grasps == 2){
	/* radial grasp */
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
	pg->set_fixedFingerDirection(ffd);
	if((radius <= GP_MAX_CYL_SIDEGRASP_DIAMETER) &&
	   (radius >= GP_MIN_CYL_SIDEGRASP_DIAMETER) &&
	   (height >= GP_MIN_CYL_SIDEGRASP_HEIGHT))
	  retList.push_back(pg);
	else delete pg;
	
	/* thumb parallel to height axis */
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
	pg->set_fixedFingerDirection(ffd);
	if (height <= GP_MAX_CYL_SIDEGRASP_HEIGHT)
	  retList.push_back(pg);
	else delete pg;
      }
    }
  }
  /* delete old graspList */
  for (it=graspList.begin(); it!=graspList.end(); it++){
    delete (*it);
  }
  graspList.clear();
  
  graspList = retList;
  
  return;
}

/*!
  Given a path to a sphere primitive, and a list of planned grasp directions,
  this will create one or more grasps for each grasp position using different
  thumb directions and a spherical grasp preshape.  The number of grasps
  created for a given grasp direction depends on the sampling parameters and
  the dimensions of the primitive.
*/
void
grasp_planner::computeSphereGraspPreshapes(std::list<plannedGrasp*>& graspList, SoPath* sop)
{
  cartesian_coordinates help1;
  cartesian_coordinates help2;
  cartesian_coordinates help3;
  cartesian_coordinates help4;
  cartesian_coordinates ffd;
  preshape comp_preshape;
  plannedGrasp *pg;
  std::list<plannedGrasp*> retList;
  std::list<plannedGrasp*>::iterator it;
  double radius = ((SoSphere*)sop->getTail())->radius.getValue();
  double step;
  
  if((radius <= GP_MAX_SPH_DIAMETER) &&
     (radius >= GP_MIN_SPH_DIAMETER)){
    
    comp_preshape.set_preshapeType(PRESHAPE_SPHERE_GRASP);
    
    for (it=graspList.begin(); it!=graspList.end(); it++){
      
      /* help1 has to be perpendicular to actual graspDirection */
      /* sorry for all these meaningful names */
      help3 = cartesian_coordinates(1., 0., 0.);
      help4 = cartesian_coordinates(0., 1., 1.);
      if ((*it)->get_graspDirection().get_dir() % help3 > (*it)->get_graspDirection().get_dir() % help4)
	help3 = help4;
      /* now help3 is NOT parallel to direction, we can do a cross product */
      help1 = cartesian_coordinates((*it)->get_graspDirection().get_dir() * help3);
      /* now help1 is perpendicular to grasp direction */
      help2 = (*it)->get_graspDirection().get_dir() * help1;
      /* now help1 and help2 are perpendicular to grasp direction, with help1 perp to help2 */
      
      for (int i=0; i< nr_of_grasp_rotations; i++){
	step = (double)i*2.0*M_PI/(double)nr_of_grasp_rotations;
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates(cos(step) * help1 + sin(step) * help2);
	pg->set_fixedFingerDirection(ffd);
	retList.push_back(pg);
      }
    }
  }
  /* delete old graspList */
  for (it=graspList.begin(); it!=graspList.end(); it++){
    delete (*it);
  }
  graspList.clear();
  
  graspList = retList;
  
  return;  
}

/*!
  Given a path to a cone primitive, and a list of planned grasp directions,
  this will create one or more grasps for each grasp position using different
  thumb directions and either a cylindrical grasp preshape (for side and edge
  grasps) or a spherical grasp preshape (for top and bottom grasps).  The
  number of grasps created for a given grasp direction depends on the sampling
  parameters and the dimensions of the primitive.
*/
void
grasp_planner::computeConeGraspPreshapes(std::list<plannedGrasp*>& graspList, SoPath* sop)
{
  cartesian_coordinates help1;
  cartesian_coordinates help2;
  cartesian_coordinates ffd;
  preshape comp_preshape;
  plannedGrasp *pg;
  std::list<plannedGrasp*> retList;
  std::list<plannedGrasp*>::iterator it;
  double bottomRadius = ((SoCone*)sop->getTail())->bottomRadius.getValue();
  double height = ((SoCone*)sop->getTail())->height.getValue();
  double step;
  
  for (it=graspList.begin(); it!=graspList.end(); it++){
    /* Top direction */
    if ((*it)->get_graspDirection().get_gdType() == GDT_CONE_TOP){
      if((bottomRadius <= GP_MAX_CONE_TGRASP_DIAMETER) &&
	 (bottomRadius >= GP_MIN_CONE_TGRASP_DIAMETER) &&
	 (height       >= GP_MIN_CONE_TGRASP_HEIGHT)){
	
	comp_preshape.set_preshapeType(PRESHAPE_CONE_TOP_BOTTOM_GRASP);
	
	help1 = cartesian_coordinates(1.0, 0.0, 0.0);
	help2 = (*it)->get_graspDirection().get_dir() * help1;
	
	for (int i=0; i< nr_of_grasp_rotations; i++){
	  step = (double)i*2.0*M_PI/(double)nr_of_grasp_rotations;
	  pg = new plannedGrasp((*it)->get_graspDirection());
	  pg->set_preshape(comp_preshape);
	  ffd = cartesian_coordinates(cos(step) * help1 + sin(step) * help2);
	  pg->set_fixedFingerDirection(ffd);
	  retList.push_back(pg);
	}
      }
    }
    /* Bottom direction */
    else if ((*it)->get_graspDirection().get_gdType() == GDT_CONE_BOTTOM){
      if((bottomRadius <= GP_MAX_CONE_BGRASP_DIAMETER) &&
	 (bottomRadius >= GP_MIN_CONE_BGRASP_DIAMETER) &&
	 (height       >= GP_MIN_CONE_BGRASP_HEIGHT)){
	
	comp_preshape.set_preshapeType(PRESHAPE_CONE_TOP_BOTTOM_GRASP);
	
	help1 = cartesian_coordinates(1.0, 0.0, 0.0);
	help2 = (*it)->get_graspDirection().get_dir() * help1;
	
	for (int i=0; i< nr_of_grasp_rotations; i++){
	  step = (double)i*2.0*M_PI/(double)nr_of_grasp_rotations;
	  pg = new plannedGrasp((*it)->get_graspDirection());
	  pg->set_preshape(comp_preshape);
	  ffd = cartesian_coordinates(cos(step) * help1 + sin(step) * help2);
	  pg->set_fixedFingerDirection(ffd);
	  retList.push_back(pg);
	}
      }
    }
    
    /* Side grasp */
    else if ((*it)->get_graspDirection().get_gdType() == GDT_CONE_SIDE_PLANE){
      
      comp_preshape.set_preshapeType(PRESHAPE_CONE_SIDE_GRASP);
      
      /* radial grasp */
      pg = new plannedGrasp((*it)->get_graspDirection());
      pg->set_preshape(comp_preshape);
      if ((*it)->get_graspDirection().get_dir().x() != 0.0 ||
	  (*it)->get_graspDirection().get_dir().y() != 1.0 ||
	  (*it)->get_graspDirection().get_dir().z() != 0.0)
	ffd = cartesian_coordinates(normalise((*it)->get_graspDirection().get_dir() * cartesian_coordinates(0.0, 1.0, 0.0)));
      else 
	ffd = cartesian_coordinates(normalise((*it)->get_graspDirection().get_dir() * cartesian_coordinates(1.0, 0.0, 0.0)));
      
      pg->set_fixedFingerDirection(ffd);
      if((bottomRadius <= GP_MAX_CONE_SIDEGRASP_DIAMETER) &&
	 (bottomRadius >= GP_MIN_CONE_SIDEGRASP_DIAMETER) &&
	 (height       >= GP_MIN_CONE_SIDEGRASP_HEIGHT))
	retList.push_back(pg);
      else delete pg;
      
      /* thumb parallel to height axis */
      pg = new plannedGrasp((*it)->get_graspDirection());
      pg->set_preshape(comp_preshape);
      ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
      pg->set_fixedFingerDirection(ffd);
      if (height <= GP_MAX_CONE_SIDEGRASP_HEIGHT)
	retList.push_back(pg);
      else delete pg;
      
      if (nr_of_180_deg_grasps == 2){
	/* radial grasp */
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
	pg->set_fixedFingerDirection(ffd);
	if((bottomRadius <= GP_MAX_CONE_SIDEGRASP_DIAMETER) &&
	   (bottomRadius >= GP_MIN_CONE_SIDEGRASP_DIAMETER) &&
	   (height       >= GP_MIN_CONE_SIDEGRASP_HEIGHT))
	  retList.push_back(pg);
	else delete pg;
	
	/* thumb parallel to height axis */
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
	pg->set_fixedFingerDirection(ffd);
	if (height <= GP_MAX_CONE_SIDEGRASP_HEIGHT)
	  retList.push_back(pg);
	else delete pg;
      }
    }
    
    /* Edge grasp */
    else if ((*it)->get_graspDirection().get_gdType() == GDT_CONE_EDGE){
      
      comp_preshape.set_preshapeType(PRESHAPE_CONE_EDGE_GRASP);
      
      /* radial grasp */
      pg = new plannedGrasp((*it)->get_graspDirection());
      pg->set_preshape(comp_preshape);
      if ((*it)->get_graspDirection().get_dir().x() != 0.0 ||
	  (*it)->get_graspDirection().get_dir().y() != 1.0 ||
	  (*it)->get_graspDirection().get_dir().z() != 0.0)
	ffd = cartesian_coordinates(normalise((*it)->get_graspDirection().get_dir() * cartesian_coordinates(0.0, 1.0, 0.0)));
      else 
	ffd = cartesian_coordinates(normalise((*it)->get_graspDirection().get_dir() * cartesian_coordinates(1.0, 0.0, 0.0)));
      
      pg->set_fixedFingerDirection(ffd);
      if((bottomRadius <= GP_MAX_CONE_EDGEGRASP_DIAMETER) &&
	 (bottomRadius >= GP_MIN_CONE_EDGEGRASP_DIAMETER) &&
	 (height       >= GP_MIN_CONE_EDGEGRASP_HEIGHT))
	retList.push_back(pg);
      else delete pg;
      
      /* thumb parallel to height axis */
      pg = new plannedGrasp((*it)->get_graspDirection());
      pg->set_preshape(comp_preshape);
      ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
      pg->set_fixedFingerDirection(ffd);
      if (height <= GP_MAX_CONE_EDGEGRASP_HEIGHT)
	retList.push_back(pg);
      else delete pg;
      
      if (nr_of_180_deg_grasps == 2){
	/* radial grasp */
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
	pg->set_fixedFingerDirection(ffd);
	if((bottomRadius <= GP_MAX_CONE_EDGEGRASP_DIAMETER) &&
	   (bottomRadius >= GP_MIN_CONE_EDGEGRASP_DIAMETER) &&
	   (height       >= GP_MIN_CONE_EDGEGRASP_HEIGHT))
	  retList.push_back(pg);
	else delete pg;
	
	/* thumb parallel to height axis */
	pg = new plannedGrasp((*it)->get_graspDirection());
	pg->set_preshape(comp_preshape);
	ffd = cartesian_coordinates((*it)->get_graspDirection().get_dir() * ffd);
	pg->set_fixedFingerDirection(ffd);
	if (height <= GP_MAX_CONE_EDGEGRASP_HEIGHT)
	  retList.push_back(pg);
	else delete pg;
      }
    }
    
    
  }
  
  /* delete old graspList */
  for (it=graspList.begin(); it!=graspList.end(); it++){
    delete (*it);
  }
  graspList.clear();
  
  graspList = retList;
  
  return;  
}

/*!
  Returns the current value of the parameterMode.
*/
int  
grasp_planner::get_parameterMode(){
    return parameterMode;
}

/*!
  Sets the parameterMode to \a in.  Returns TRUE if the set was successful.
*/
bool
grasp_planner::set_parameterMode(int in){
    if (in >= 0){
	parameterMode = in;
	return true;
    }
    return false;
}

