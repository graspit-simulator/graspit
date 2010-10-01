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
// Author(s):  Andrew T. Miller 
//
// $Id: gwsprojection.h,v 1.5 2009/06/16 22:53:24 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the grasp wrench space projection class.
 */

#include <set>
class SoQtRenderArea;
class SoQtExaminerViewer;
class SoCoordinate3;
class SoIndexedFaceSet;
class SoSeparator;
class QWidget;
class GWS;

//! Produces an Inventor window with a 3-D projection of a GWS.  
/*!
  The projection window uses the same camera as the mainViewer.  The GWS
  uses the 3 fixed coordinate values to project each of its 6D
  hyperplanes into 3D. These are then intersected with qhull, and the
  hull boundary is sent back to this class, where it is used to create
  an indexed face set.  This geometry, along with axes is then viewable
  in the new window. 
*/ 
class GWSprojection {

  //! A pointer to the GWS that this projection comes from
  GWS *gws;

  //! Coordinate of the projection (only 3 values are used)
  double projCoords[6];

  //! A set of 3 indices that are fixed (3 coordinates)
  std::set<int> fixedCoordIndex;

  //! A pointer to the QT widget holding the projection window.
  QWidget *projWin;

  //! A pointer to the viewer for this window
  SoQtRenderArea *projectionViewer;

  //! Root of the convex hull geometry
  SoSeparator *hullSep;

  //! 3D hull vertices
  SoCoordinate3 *hullCoords;

  //! Inventor geometry node for the hull
  SoIndexedFaceSet *hullIFS;

  //! Inventor node for the projection
  SoSeparator *sg;

  void setWinTitle();

public:

  GWSprojection(SoQtExaminerViewer* mainViewer,GWS *g,double *c, std::set<int> whichFixed);
  ~GWSprojection();

  /*! Returns a pointer to the main QT widget for the projection window. */
  QWidget *getProjWin() const {return projWin;}

  /*! Returns a pointer to the gws. */
  GWS *getGWS() const {return gws;}

  /*! Returns a pointer to the root of the GWS*/
  SoSeparator * getGWSRoot() { return sg; }

  void update();
  void deleteHull();
};
