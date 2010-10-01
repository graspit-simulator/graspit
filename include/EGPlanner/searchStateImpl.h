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
// $Id: searchStateImpl.h,v 1.4 2009/05/07 19:57:46 cmatei Exp $
//
//######################################################################

#ifndef _HandObjectStateimpl_h_
#define _HandObjectStateimpl_h_

#include "searchState.h"

/*! \file These are implementations of the various interfaces 
	for storing hand poature and hand position. 

	Ways of storing hand posture:
	<ul>
	<li> in eigenspace (store eigengrasp amplitudes)
	<li> in DOF space (store exact DoF's)
	</ul>
	Ways of storing hand position:
	<ul>
	<li> complete (4 entries of a quaternion + translation)
	<li> axis-angle (3 entries for rotation + translation)
	<li> elipsoid (a point on a pre-defined ellipsoid)
	<li> approach (relative to a reference hand position and 
		 pre-defined appraoch direction)
	</ul>
	Right now, it is only possible to transition from any other
	parameterization into the "complete" parameterization, without
	losing the saved position.
*/

//! Saves hand posture in eigengrasp space
class PostureStateEigen : public PostureState
{
protected:
	void createVariables();
public:
	PostureStateEigen(const Hand *h) : PostureState(h){createVariables();}
	StateType getType() const {return POSE_EIGEN;}
	void getHandDOF(double *dof) const;
	void storeHandDOF(const double *dof);
};

//! Saves hand posture in dof space
class PostureStateDOF : public PostureState
{
protected:
	void createVariables();
public:
	PostureStateDOF(const Hand *h) : PostureState(h){createVariables();}
	StateType getType() const {return POSE_DOF;}
	void getHandDOF(double *dof) const;
	void storeHandDOF(const double *dof);
};

//! Saves complete hand position as quaternion + translation; 7 variables
/*! This parameterization is redundant (as it is not necessary to save
	all 4 entries in a quaternion) and therefore is not used for actually
	searching, but for saving and storing positions.
*/
class PositionStateComplete : public PositionState
{
protected:
	void createVariables();
public:
	PositionStateComplete(const Hand *h) : PositionState(h){createVariables();}
	StateType getType() const {return SPACE_COMPLETE;}
	transf getCoreTran() const;
	void setTran(const transf &t);
};

//! Saves hand position as any rotation and translation; 6 variables
/*! Rotation is saved using only 3 variables in an axis-angle way: the
	first two variables encode an axis of rotation and the third one is
	the rotation angle around that axis.
*/
class PositionStateAA : public PositionState
{
protected:
	void createVariables();
public:
	PositionStateAA(const Hand *h) : PositionState(h){createVariables();}
	StateType getType() const {return SPACE_AXIS_ANGLE;}
	transf getCoreTran() const;
	void setTran(const transf &t);
};

//! Saves a hand position on a predefined ellipsoid; 4 variables and 3 parameters
/*! The shape of the ellipsoid is determined by the parameters a, b and c.
	Uses 4 variables: beta, gama and tau determine a point on the ellipsoid,
	with the hand placed so that its pre-defined approach direction is normal
	to the ellipsoid. The fourth variable, dist, then moves the hand back and
	forth along the approach direction.
*/
class PositionStateEllipsoid : public PositionState
{
protected:
	void createVariables();
public:
	PositionStateEllipsoid(const Hand *h) : PositionState(h){createVariables();}
	StateType getType() const {return SPACE_ELLIPSOID;}
	transf getCoreTran() const;
	void setTran(const transf &t);
};

//! Allows the hand to move back and forth in a cone around its pre-defined approach direction; 3 variables
/*! One variable specifies how much the hand moves back and forth along
	its approach direction. The other two variables, which are angular,
	define a cone around the approach direction that the hand can move
	in. This is used with the on-line planner, where a reference hand
	position is specified externally.
*/
class PositionStateApproach : public PositionState
{
protected:
	void createVariables();
public:
	PositionStateApproach(const Hand *h) : PositionState(h){createVariables();}
	StateType getType() const {return SPACE_APPROACH;}
	transf getCoreTran() const;
	void setTran(const transf &t);
};

#endif
