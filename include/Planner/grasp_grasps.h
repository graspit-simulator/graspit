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
// $Id: grasp_grasps.h,v 1.2 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/***************************************************************************/
/*      FILE:     grasp_grasps.h                                           */
/*      AUTHOR:   Steffen Knoop                                            */
/*      DATE:     03-08-2002                                               */
/***************************************************************************/

/*! \file
 \brief Defines the classes finalGraspPosition and plannedGrasp (used in the grasp planner)
*/

#ifndef __GRASP_GRASPS_H__
#define __GRASP_GRASPS_H__

#define DIST_DIR_WEIGHT    5.0
#define DIST_FFD_WEIGHT    2.0
#define DIST_PRESH_WEIGHT  1.0

#include <list>
#include <vector>

class grasp_representation;
class GraspableBody;

//! The final pose of the hand after a grasp candidate has been tested.
/*!
  This class records the hand's final position and orientation as well as its
  DOF values after the tester has perfomed its tests with a particular grasp
  candidate.
*/
class finalGraspPosition
{
private:
  //! Final pose of the hand
  transf finalTran;

  //! Final DOF values
    std::list<double> dof;

public:
    finalGraspPosition();
    finalGraspPosition(const finalGraspPosition&);
    ~finalGraspPosition();

    transf get_finalTran()const;
    void   set_finalTran(transf);
    
    std::list<double> get_dof()      const;
    void              add_dof(double);
    bool              change_dof(unsigned int, double);
};

/**********************/

//! This class represents a single candidate grasp for the grasp planner
/*!
  It holds a grasp direction, consisting of an initial world position and palm
  approach vector, a thumb direction, and a grasp preshape.  After the
  tester moves the hand along this approach vector, closes the fingers and
  evaluates the grasp, the quality and final grasp position are recorded
  in this class as well.  This class also has a pointer to the
  associated grasp_representation 
*/
class plannedGrasp
{
 private:
  //! Grasp direction includes a point and a direction
  cartesianGraspDirection myGraspDir;
  
  //! Direction of the thumb
  cartesian_coordinates fixedFingerDirection;
  
  //! Stores the final grasp position (after testing)
  finalGraspPosition finalGPos;
  
  //      /* reachability of GraspDirection for the platform, arm and hand */
  //      bool isReachable;
  
  //! This grasp's quality as evaluated by the tester
  double myQuality;
  
  //! The grasp preshape
  preshape myPreshape;
  
  //! Pointer to the grasped object
  GraspableBody *myGraspableBody;
  
  //! Pointer to the associated grasp visualization
  grasp_representation *myCP;
  
 public:

    /* constructor, destructor */
    plannedGrasp();
    plannedGrasp(const plannedGrasp&);
    plannedGrasp(cartesianGraspDirection);
    ~plannedGrasp();

    /* return value: 0 <= val <= 1.0 */
    double distanceTo(plannedGrasp) const;

    /* access methods */
    cartesianGraspDirection get_graspDirection() const;
    void                    set_graspDirection(cartesianGraspDirection);
    
    cartesian_coordinates   get_fixedFingerDirection() const;
    void                    set_fixedFingerDirection(cartesian_coordinates);

    finalGraspPosition      get_finalGraspPosition() const;
    void                    set_finalGraspPosition(finalGraspPosition);

//      bool           get_isReachable()         const;
//      void           set_isReachable(bool);

    double         get_quality()                 const;
    void           set_quality(double);

    preshape       get_preshape()                const;
    void           set_preshape(preshape in);

    GraspableBody* get_graspableBody()           const;
    void           set_graspableBody(GraspableBody*);

    grasp_representation* get_graspRepresentation()  const;
    void                  set_graspRepresentation(grasp_representation*);
    void                  remove_graspRepresentation();
};


/*
  Two different versions are used because of the differences in the
  STL implementations
*/

#ifdef WIN32

//! An operator to compare two planned grasps
struct compareGraspQM : public std::greater<plannedGrasp *>
{
  bool operator()(plannedGrasp *&, plannedGrasp *&) const;
};

#else

//! An operator to compare two planned grasps
struct compareGraspQM : public std::less<plannedGrasp *>
{
  bool operator()(plannedGrasp *&, plannedGrasp *&) const;
};

#endif

/* This function is needed by the STL for sorting the grasp list 
   according to the qm */
//bool compareGraspQM(plannedGrasp*, plannedGrasp*);

#endif


/******************
   Local Variables:
   mode:c++
   End:
******************/























