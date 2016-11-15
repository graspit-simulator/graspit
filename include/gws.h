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
// $Id: gws.h,v 1.9 2009/06/16 22:53:24 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the abstract Grasp Wrench Space class and specific GWS classes
 */
#ifndef GWS_H

#include <vector>
#include <set>
#include "matvec3D.h"

class Grasp;
struct coordT;

//! An abstract base class for 6D grasp wrench spaces
/*!
  Specific grasp wrench spaces should be defined as subclasses of this
  class.
*/
class GWS {
public:
  //! Default option for building a GWS on all 6 dimensions
  static const std::vector<int> ALL_DIMENSIONS;
protected:
	//! Consider the effects of gravity on the object
	bool useGravity;

	//! The direction and magnitude of gravity along each GWS axis
	vec3 gravDirection;

	//! Calls QHull to build from an array of pre-specified wrenches
	int buildHyperplanesFromWrenches(void *wr, int numWrenches, 
									 std::vector<int> useDimensions);

	//! After the hyperplanes are computed we can look at other metrics
	void computeHyperplaneMetrics();

	//! Deletes the hyperplanes of the currently computed GWS
	void clearGWS();

 public:

  //! Tells is wether to use gravity, and if yes it's direction and magnitude
  void setGravity(bool use, vec3 gd=vec3(0,0,0));

  //! A pointer to the associated grasp
  Grasp *grasp;
  
  //! number of 6D hyperplanes bounding this GWS
  int numHyperPlanes;

  //! 7 x numHyperPlanes matrix of plane coefficients
  double **hyperPlanes;

  //! Keeps track of the dimensions used to build this gws
  std::vector<int> mUsedDimensions;
  
  //! Surface area of 6D hull (returned by qhull)
  double hullArea;
  
  //! Volume of 6D hull (returned by qhull)
  double hullVolume;

  //! If GWS contains the origin, the grasp has force closure
  bool forceClosure;

  //! Number of reference to this GWS.  When it is 0, GWS may be deleted
  int refCount;

  //! Initializes grasp to \a g.  Zeros hyperplanes and refCount.
  GWS(Grasp *g) : useGravity(false), grasp(g),numHyperPlanes(0),hyperPlanes(NULL),refCount(0) {}

  virtual ~GWS();

  /*! Returns a pointer to the grasp associated with this GWS */
  Grasp *getGrasp() const {return grasp;}

  /*! Adds one reference to this GWS */
  void ref() {refCount++;}

  /*! Removes one reference to this GWS */
  void unref() {refCount--;}

  /*! Returns the current number of references to this GWS */
  int getRefCount() {return refCount;}

  /*! Returns whether this grasp has force closure. */
  bool isForceClosure() {return forceClosure;}

  /*! Returns whether the 6D GWS has non-zero volume. */
  bool hasPositiveVolume(){ if (hullVolume>0) return true; return false;}

  /*! Returns a string that is the name of the type of GWS. */
  virtual const char *getType() =0;

  /*! Builds the GWS from the contact wrenches. */
  virtual int build(std::vector<int> useDimensions = ALL_DIMENSIONS) = 0;

  //! Creates a 3D projection of the 6D wrench space
  virtual int projectTo3D(double *projCoords,std::set<int> fixedCoordSet,
			  std::vector<position> &hullCoords,
			  std::vector<int> &hullIndices);

  //! Array of strings of possible GWS types
  static const char *TYPE_LIST[];

  static GWS *createInstance(const char *type,Grasp *g);
};

//! A class to build an L1 GWS
/*!
  A unit grasp wrench space constructed using the L1 norm of the
  grasp vector (i.e. The sum magnitude of all contact normal forces is 1).
*/
class L1GWS : public GWS {

  //! Name of this type of GWS
  static const char *type;

 public:

  /*! Stub */
  L1GWS(Grasp *g) : GWS(g) {}

  //! Build the GWS using individual contact wrenches
  int build(std::vector<int> useDimensions = ALL_DIMENSIONS);

  /*! Returns the name of this type of GWS */
  static const char *getClassType() {return type;}

  /*! Returns the name of this type of GWS */
  virtual const char *getType() {return type;}
};

//! A class to build an L-Infinity GWS
/*!
  A unit grasp wrench space constructed using the L-Infinity norm of the
  grasp vector (i.e. The maximum magnitude of any contact normal force is 1).
*/
class LInfGWS : public GWS {

  //! Name of this type of GWS
  static const char *type;

 public:

  /*! Stub */
  LInfGWS(Grasp *g) : GWS(g) {}
  //  virtual ~LInfGWS() {}

  //! Build the GWS using the Minkowski sum of all contact wrenches
  int build(std::vector<int> useDimensions = ALL_DIMENSIONS);

  /*! Returns the name of this type of GWS */
  static const char *getClassType() {return type;}

  /*! Returns the name of this type of GWS */
  virtual const char *getType() {return type;}

};

#define GWS_H
#endif
