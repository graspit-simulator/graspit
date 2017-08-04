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
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: graspit_db_grasp.cpp,v 1.14 2010/09/01 23:56:09 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the special %GraspitDBGrasp class
 */

#include "graspit/DBase/graspit_db_grasp.h"
#include <QSqlQuery>
#include <QtSql>
#include <list>

#include "graspit/EGPlanner/searchState.h"
#include "graspit/matvec3D.h"
#include "graspit/body.h"
#include "graspit/robot.h"

#include "graspit/debug.h"

//! Create a HandObjectState from data
bool initializeHandObjectState(const std::vector<double> &joints, const std::vector<double> &position,
                               GraspPlanningState *state)
{
  if (joints.size() == 0) { return false; }
  state->setPostureType(POSE_DOF);
  state->getPosture()->readFromArray(const_cast<std::vector<double>&>(joints));
  state->setPositionType(SPACE_COMPLETE);
  state->getPosition()->readFromArray(const_cast<std::vector<double>&>(position));
  return true;
}

GraspitDBGrasp::~GraspitDBGrasp() {
  delete mPreGrasp;
  delete mFinalGrasp;
}

//! copy another grasp
GraspitDBGrasp::GraspitDBGrasp(const GraspitDBGrasp &grasp2) : db_planner::Grasp(grasp2), mHand(grasp2.mHand) {
  mPreGrasp = new GraspPlanningState(grasp2.mPreGrasp->getHand());
  mPreGrasp->copyFrom(grasp2.mPreGrasp);
  mFinalGrasp = new GraspPlanningState(grasp2.mFinalGrasp->getHand());
  mFinalGrasp->copyFrom(grasp2.mFinalGrasp);
  mTestScores = grasp2.mTestScores;
}

//! give the required data, fill in the necessary information of a grasp
bool GraspitDBGrasp::SetGraspParameters(const std::vector<double> &prejoint,
                                        const std::vector<double> &prepos,
                                        const std::vector<double> &finjoint,
                                        const std::vector<double> &finpos) {

  mPreGrasp = new GraspPlanningState(mHand);
  initializeHandObjectState(prejoint, prepos, mPreGrasp);

  mFinalGrasp = new GraspPlanningState(mHand);
  initializeHandObjectState(finjoint, finpos, mFinalGrasp);
  return true;
}

//! transform this grasp by the transformation defined by array
bool GraspitDBGrasp::Transform(const float array[16]) {
  // synthesize the transformation matrix
  mat3 m;
  m(0) = array[0];
  m(1) = array[1];
  m(2) = array[2];
  m(3) = array[4];
  m(4) = array[5];
  m(5) = array[6];
  m(6) = array[8];
  m(7) = array[9];
  m(8) = array[10];
  vec3 v;
  v(0) = array[3];
  v(1) = array[7];
  v(2) = array[11];
  transf transform;
  transform.set(m, v);

  std::vector<double> position;

  PositionState *ps = mPreGrasp->getPosition();
  ps->setTran(transform % ps->getCoreTran());
  for (int i = 0; i < ps->getNumVariables(); ++i) {
    position.push_back(ps->getVariable(i)->getValue());
  }
  SetPregraspPosition(position);

  position.clear();
  ps = mFinalGrasp->getPosition();
  ps->setTran(transform % ps->getCoreTran());
  for (int i = 0; i < ps->getNumVariables(); ++i) {
    position.push_back(ps->getVariable(i)->getValue());
  }
  SetFinalgraspPosition(position);

  return true;
}

//! get the average test scores stored in this grasp
double GraspitDBGrasp::getTestAverageScore() {
  double sum = 0;
  for (int i = 0; i < (int)mTestScores.size(); i ++) {
    sum += mTestScores[i];
  }
  if ((int)mTestScores.size() > 0) {
    return sum / (double)mTestScores.size();
  }
  else {
    return 0;
  }
}

void GraspitDBGrasp::setPreGraspPlanningState(GraspPlanningState *p) {
  delete mPreGrasp;
  mPreGrasp = p;

  const PositionState *positionS = p->readPosition();
  const PostureState *postureS = p->readPosture();

  std::vector<double> position, joints;
  for (int i = 0; i < positionS->getNumVariables(); ++i) {
    position.push_back(positionS->getVariable(i)->getValue());
  }
  for (int i = 0; i < postureS->getNumVariables(); ++i) {
    joints.push_back(postureS->getVariable(i)->getValue());
  }
  this->SetPregraspJoints(joints);
  this->SetPregraspPosition(position);
}

void GraspitDBGrasp::setFinalGraspPlanningState(GraspPlanningState *p) {
  delete mFinalGrasp;
  mFinalGrasp = p;

  const PositionState *positionS = p->readPosition();
  const PostureState *postureS = p->readPosture();

  std::vector<double> pos, joints, contacts;
  for (int i = 0; i < positionS->getNumVariables(); ++i) {
    pos.push_back(positionS->getVariable(i)->getValue());
  }
  for (int i = 0; i < postureS->getNumVariables(); ++i) {
    joints.push_back(postureS->getVariable(i)->getValue());
  }
  std::list<position> *tmpContacts;
  tmpContacts = p->getContacts();
  for (std::list<position>::iterator it = tmpContacts->begin(); it != tmpContacts->end(); ++it) {
    contacts.push_back((*it).x());
    contacts.push_back((*it).y());
    contacts.push_back((*it).z());
  }
  this->SetFinalgraspJoints(joints);
  this->SetFinalgraspPosition(pos);
  this->SetContacts(contacts);
}

