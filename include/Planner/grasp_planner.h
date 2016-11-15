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
// $Id: grasp_planner.h,v 1.2 2009/03/25 22:10:24 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_planner.h                                          */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the grasp_planner class (part of grasp planner)
*/

#ifndef __GRASP_PLANNER_H__
#define __GRASP_PLANNER_H__

/* Starting distance is the (double) distance, in which
   the hand is placed acc to the object before grasping. 
   The distance is measured from the surface of the object. */
#define STARTING_DISTANCE 80.0

/* When grasping an axially symmetric object, the 360degrees
   are approximated with NR_OF_360_DEG_STEPS steps. 
   This must be an even number, because the 180 degrees of
   teta (spherical coordinate space) are approximated
   with NR_OF_360_DEG_STEPS/2. */
#define NR_OF_360_DEG_STEPS 2

/* For a cylinder side grasp or a cube grasp, grasps may be 
   generated in different planes. If this number is even,
   the middle plane won't be used (so you probably should 
   give an odd number) */
#define NR_OF_PARALLEL_PLANES 1

/* When you grasp a cylinder or a cube from one side, you have
   2 possibilities, turned 180 deg to each other. If you want to
   try both of them (they are very similar), set this define to
   2, else to 1. */
#define NR_OF_180_DEG_GRASPS 1

/* Cylinder from the top or sphere grasp: Try NR_OF_GRASP_ROTATIONS
   different palm z rotations */
#define NR_OF_GRASP_ROTATIONS 1

/* Parameter mode is described within the class definition */
#define DEFAULT_PARAMETER_MODE 2

/* Maximum and minimum object sizes for grasping primitives. 
   All values in mm. */

/* Prim: Cube
   Presh: PRESHAPE_CUBE_GRASP */
#define GP_MAX_CUBE_SIZE 200.0
#define GP_MIN_CUBE_SIZE 0.0

/* Prim: Cylinder
   Presh: PRESHAPE_CYLINDER_SIDE_GRASP
   Thumb dir: phi */
#define GP_MAX_CYL_SIDEGRASP_DIAMETER 180.0
#define GP_MIN_CYL_SIDEGRASP_DIAMETER 0.0
#define GP_MIN_CYL_SIDEGRASP_HEIGHT 28.0
/* Prim: Cylinder
   Presh: PRESHAPE_CYLINDER_SIDE_GRASP
   Thumb dir: z */
#define GP_MAX_CYL_SIDEGRASP_HEIGHT 200.0
   
/* Prim: Cylinder
   Presh: PRESHAPE_CYLINDER_TOP_BOTTOM_GRASP
   Thumb dir: R */
#define GP_MAX_CYL_TBGRASP_DIAMETER 180.0
#define GP_MIN_CYL_TBGRASP_DIAMETER 5.0
#define GP_MIN_CYL_TBGRASP_HEIGHT 40.0

/* Prim: Sphere
   Presh: PRESHAPE_SPHERE_GRASP
   Thumb dir: perp. to r */
#define GP_MAX_SPH_DIAMETER 180.0
#define GP_MIN_SPH_DIAMETER   0.0

/* Prim: Cone
   Presh: PRESHAPE_CONE_TOP_BOTTOM_GRASP; from TOP
   Thumb dir: R */
#define GP_MAX_CONE_TGRASP_DIAMETER 100000.0
#define GP_MIN_CONE_TGRASP_DIAMETER 5.0
#define GP_MIN_CONE_TGRASP_HEIGHT   0.0
/* Prim: Cone
   Presh: PRESHAPE_CONE_TOP_BOTTOM_GRASP; from BOTTOM
   Thumb dir: R */
#define GP_MAX_CONE_BGRASP_DIAMETER 180.0
#define GP_MIN_CONE_BGRASP_DIAMETER 5.0
#define GP_MIN_CONE_BGRASP_HEIGHT   0.0
/* Prim: Cone
   Presh: PRESHAPE_CONE_SIDE_GRASP
   Thumb dir: phi */
#define GP_MAX_CONE_SIDEGRASP_DIAMETER 100000.0
#define GP_MIN_CONE_SIDEGRASP_DIAMETER 0.0
#define GP_MIN_CONE_SIDEGRASP_HEIGHT 0.0
/* Prim: Cone
   Presh: PRESHAPE_CONE_SIDE_GRASP
   Thumb dir: z */
#define GP_MAX_CONE_SIDEGRASP_HEIGHT 100000.0
/* Prim: Cone
   Presh: PRESHAPE_CONE_EDGE_GRASP
   Thumb dir: phi */
#define GP_MAX_CONE_EDGEGRASP_DIAMETER 180.0
#define GP_MIN_CONE_EDGEGRASP_DIAMETER 0.0
#define GP_MIN_CONE_EDGEGRASP_HEIGHT 28.0
/* Prim: Cone
   Presh: PRESHAPE_CONE_EDGE_GRASP
   Thumb dir: z */
#define GP_MAX_CONE_EDGEGRASP_HEIGHT 100000.0



/* Grasp preshapes for existing primitives */
#define PRESHAPE_CUBE_GRASP                PR_two_opp_one 
#define PRESHAPE_CYLINDER_TOP_BOTTOM_GRASP PR_circle
#define PRESHAPE_CYLINDER_SIDE_GRASP       PR_two_opp_one
#define PRESHAPE_SPHERE_GRASP              PR_circle
#define PRESHAPE_CONE_TOP_BOTTOM_GRASP     PR_circle
#define PRESHAPE_CONE_SIDE_GRASP           PR_two_opp_one
#define PRESHAPE_CONE_EDGE_GRASP           PR_two_opp_one

/* When adding k times 2*pi/k, you won't end up
   with 2*pi but with 2*pi-delta. This defines 
   a threshold for delta. */
#define DELTA_360_DEG_ERROR  0.0001

class Body;
class SoQtExaminerViewer;
class GraspableBody;
class IVmgr;
class plannedGrasp;
class GraspDirection;
class SoPath;
class SoSeparator;

#include "matvec3D.h"
#include <Inventor/SoLists.h>
#include <vector>
typedef std::pair<Body *,Body *> BodyPair;

//! This class is used to automatically generate a set of candidate grasps given a set of shape primitives for an object.
/*!
  The grasps are chosen based on heuristics defined for each shape primitive
  (see grasp planning paper "Automatic grasp planning using shape primitives").
  The number of grasps produced depends two factors: the dimensions of
  each primitive, and the values of the sampling parameters (also explained
  in the paper).  The values of these sampling parameters can be selected
  manually by the user for maximum control, or automatic sampling can be used
  and the density of the samples depends on the value of
  \a parameterMode:
   - 0 The parameters are taken from the user interface 
        or if unchanged from the default value defines
   - 1 The parameters are determined automatically 
        with a low resolution
   - 2 Automatically, but bigger numbers are chosen,
        which results in better grasps as well as in
        longer computing time
   - 3 Automatically, very high resolution. 

   At the completion of the planning process, a list of candidate grasps is
   returned to the grasp manager.
 */
class grasp_planner
{
private:

  //! a pointer to the main window viewer
  SoQtExaminerViewer *myViewer;

  //! a pointer to the grasped object
  GraspableBody      *my_body;

  //! a pointer to the inventor manager
  IVmgr                *ivmgr;

  //! a pointer to an Inventor scene graph containing the shape primitives 
  SoGroup *IVGeomPrimitives;
    
  //! The parameter mode sets the way the parameters for the planning step are determined.
  int parameterMode;

/* The meaning of these variables is equivalent to the 
   corresponding definitions above. The values of these are 
   initially taken from the #defines, but may be changed
   by the user via the motif interface. */

  //! When grasping a cylinder, cone, or sphere, this sets how many samples of 360 degrees are used.  Should be an even number
  int nr_of_360_deg_steps;

  //! Number of parallel planes of grasping positions along the width of a cube
  int nr_of_parallel_planes_width;
  
  //! Number of parallel planes of grasping positions along the height of a cube
  int nr_of_parallel_planes_height;

  //! Number of parallel planes of grasping positions along the depth of a cube
  int nr_of_parallel_planes_depth;

  //! Number of hand rotations about the approach vector for boxes and side grasps of cylinders, either 1 or 2. 
  int nr_of_180_deg_grasps;

  //! Number of hand rotations about the approach vector for spheres and end grasps of cylinders. 
  int nr_of_grasp_rotations;

/*
 * private methods
 */

  SoPathList searchPrimitives(GraspableBody*);

    /* Takes prasp list with local cartesian coordinates
       and changes these to global. */
    void localToGlobalCoordinates(std::list<plannedGrasp*>&,
				  SoPath *,const transf &);
  
    /* Returns the global path
       of a given node */
    SoPath *getGlobalPath(SoNode *node);

    /* Compute all starting points and directions 
       from which the given primitive can be grasped.
       Does no collision detection. */
    std::list <plannedGrasp*>    getPlannedGraspDirections(SoPath*);
    /* Calls one of: */
    std::list <GraspDirection*> getCubeGraspDirections    (SoPath* sop);
    std::list <GraspDirection*> getCylinderGraspDirections(SoPath* sop);
    std::list <GraspDirection*> getSphereGraspDirections  (SoPath* sop);
    std::list <GraspDirection*> getConeGraspDirections    (SoPath* sop);

    /* Compute preshapes for all grasps in list that have
       implemented heuristics; return grasp list */
    void computeGraspPreshapes        (std::list<plannedGrasp*>&, SoPath*);
    /* Calls one or more of: */
    void computeCubeGraspPreshapes    (std::list<plannedGrasp*>&, SoPath*);
    void computeCylinderGraspPreshapes(std::list<plannedGrasp*>&, SoPath*);
    void computeSphereGraspPreshapes  (std::list<plannedGrasp*>&, SoPath*);
    void computeConeGraspPreshapes    (std::list<plannedGrasp*>&, SoPath*);

    /* helper for eliminating obsolete grasps which may occur if
       obj consists of several primitives */
    bool existsInList(plannedGrasp, std::list<plannedGrasp*>);

    /* this returns the number of grasps that will be planned. 
       If spheres are involved, this is only an estimate :-) */
    int determineNumberOfGrasps(SoPathList);

    /* If the planning parameters are set automatically, this method
       does it. The parameters depend on the parameterMode, the size
       and the shape of the primitive. */
    bool set_planningParametersFromPrimitive(SoPath*);

public:

/*
 * constructor, destructor 
 */
    grasp_planner();
    ~grasp_planner();

    /* this method calls all planning stuff */
    std::list<plannedGrasp*> planIt(GraspableBody*,SoGroup *);

    /* used by the motif interface */
    bool set_planningParameters(int nr_of_360_deg_steps_in, 
				int nr_of_parallel_planes_in,
				int nr_of_180_deg_grasps_in,
				int nr_of_grasp_rotations_in);

    void get_planningParameters(int& nr_of_360_deg_steps_in, 
				int& nr_of_parallel_planes_in,
				int& nr_of_180_deg_grasps_in,
				int& nr_of_grasp_rotations_in);


    int  get_parameterMode();
    bool set_parameterMode(int);
};

#endif /* __GRASP_PLANNER_H__ */




/******************
   Local Variables:
   mode:c++
   End:
******************/
