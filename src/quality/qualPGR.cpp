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
// Author(s): Maximilian Haas-Heger
//
//######################################################################

#include "graspit/quality/qualPCR.h"
#include "graspit/quality/qualPGR.h"

#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>

#include <pthread.h>
#include <unistd.h>

#include "graspit/grasp.h"
#include "graspit/math/matrix.h"
#include "graspit/robot.h"
#include "graspit/debug.h"

const char *QualPGR::type = "PGR";

//! Structure holding pointers to UI items specific to this quality measure
struct QualPGRParamT {
  QLineEdit *wrenchInput;
  QLineEdit *FxInput;
  QLineEdit *FyInput;
  QLineEdit *FzInput;
  QLineEdit *MxInput;
  QLineEdit *MyInput;
  QLineEdit *MzInput;

  QLineEdit *maxForceInput;

  QLineEdit *maxContactsInput;
};

/*!
  Adds the GWS specified by the combo box in the parameter area of the
  dialog box to the grasp associated with this qm.  The grasp will take care
  of creating it if it doesn't already exist.
*/
QualPGR::QualPGR(qmDlgDataT *data) : QualityMeasure(data)
{
  QualPGRParamT *params = (QualPGRParamT *)data->paramPtr;

  mWrenchMultiplier = params->wrenchInput->text().toDouble();

  mWrench.resize(6, 0.0);
  mWrench[0] = params->FxInput->text().toDouble();
  mWrench[1] = params->FyInput->text().toDouble();
  mWrench[2] = params->FzInput->text().toDouble();
  mWrench[3] = params->MxInput->text().toDouble();
  mWrench[4] = params->MyInput->text().toDouble();
  mWrench[5] = params->MzInput->text().toDouble();

  mMaxForce = params->maxForceInput->text().toDouble();

  mMaxContacts = params->maxContactsInput->text().toInt();
}

QualPGR::~QualPGR()
{
#ifdef GRASPITDBG
   std::cout << "deleting QualPGR" << std::endl;
#endif
}

double
QualPGR::evaluate()
{
  Matrix dw(&mWrench[0], 6, 1, true);
  double wrenchMultiplier = mWrenchMultiplier;
  if (mWrenchMultiplier) dw.multiply(wrenchMultiplier);

  return evaluatePGR(dw, mMaxForce, mMaxContacts);
}

double
QualPGR::evaluate3D()
{
  return 0;
}

/*! Evaluate PGR quality metric as described in 'Contact and Grasp 
    Robustness Measures: Analysis and Experiment' (1997) by 
    Prattichizzo et al.
*/
double
QualPGR::evaluatePGR(Matrix &wrench, double maxForce, int maxContacts)
{
  //use the pre-set list of contacts. This includes contacts on the palm, but
  //not contacts with other objects or obstacles
  int numContacts = grasp->getNumContacts();
  std::list<Contact*> contacts;
  for (int i=0; i<numContacts; i++) contacts.push_back(grasp->getContact(i));
  //if there are no contacts we are done
  if (contacts.empty()) {
    DBGA("No contacts");
    return -1;
  }
  //if there are too many contacts, the computational cost is too high
  if (contacts.size() > maxContacts) {
    DBGA("Too many contacts");
    return -1;
  }

  //Array of threads for parallelization of PGR computation
  int numThreads = sysconf(_SC_NPROCESSORS_ONLN);
  pthread_t threads[numThreads];
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_mutex_t statesMutex;
  pthread_mutex_t resultMutex;
  pthread_mutex_init(&statesMutex, NULL);
  pthread_mutex_init(&resultMutex, NULL);

  //Arguments for threads
  threadArgs args;
  args.g = grasp;
  args.wrench = &wrench;
  args.maxForce = maxForce;
  args.states = std::vector<int>(contacts.size(), 0);
  args.maxPCR = -1;
  args.doneFlag = false;
  args.statesMutex_pt = &statesMutex;
  args.resultMutex_pt = &resultMutex;

  for (int i=0; i<numThreads; i++) {
    if (pthread_create(&threads[i], &attr, evaluatePCRThread, (void*)&args)) {
      DBGA("Failed to create thread");
      exit(1);
    }
  }
  for (int i=0; i<numThreads; i++) {
    void *status = NULL;
    pthread_join(threads[i], &status);
    if (status) {
      DBGA("Failed to join tread;");
      exit(1);
    }
  }

  //Clean-up
  pthread_attr_destroy(&attr);
  pthread_mutex_destroy(&statesMutex);
  pthread_mutex_destroy(&resultMutex);

  if (args.maxPCR == -1) {
    DBGA("No stable grasp found");
    return -1;
  }
  double PGR = QualPCR::evaluatePCR(grasp, wrench, maxForce, args.finalStates);
  assert(PGR - args.maxPCR < Matrix::EPS);
  std::cerr << "optimal contact states: ";
  for (std::vector<int>::iterator it=args.finalStates.begin(); it!=args.finalStates.end(); it++)
    std::cerr << *it;
  std::cerr << std::endl;

  return PGR;
}

// Allows to evaluate PCR for different contact states on a 
// thread. This makes computation of PGR much faster. The
// argv array of pointers contains pointers to:
// 0. Grasp *g;
// 1. Matrix *wrench;
// 2. double *maxForce;
// 3. std::vector<int> *states;
// 4. std::vector<int> *final_states;
// 5. double *maxPCR;
// 6. bool *doneFlag;
// 7. states vector mutex
// 8. result mutex
void*
QualPGR::evaluatePCRThread(void *thread_args)
{
  threadArgs *args = (threadArgs*) thread_args;
  while(true) {
    //Lock states mutex
    pthread_mutex_lock(args->statesMutex_pt);
    
    //Check if all jobs are done 
    if (args->doneFlag) {
      pthread_mutex_unlock(args->statesMutex_pt);
      pthread_exit(NULL);
    }

    //Set states for next job and increment
    std::vector<int> states(args->states);
    int counter;
    do {
      counter = 0;
      (args->states)[0]++;
      for (int j=0; j<args->states.size(); j++) {
        if ((args->states)[j] == 3) {
          (args->states)[j] = 0;
          (args->states)[j+1] += 1;
        }
        if ((args->states)[j] == 0) counter += 3;
        if ((args->states)[j] == 1) counter++;
      }
      //Check if we have exhausted the queue
      args->doneFlag = true;
      std::vector<int>::iterator it;
      for (it=args->states.begin(); it!=args->states.end(); it++) {
        if ((*it) != 2) {
          args->doneFlag = false;
          break;
        }
      }
      if (args->doneFlag) {
        pthread_mutex_unlock(args->statesMutex_pt);
        pthread_exit(NULL);
      }
    } while (counter < 6);
    pthread_mutex_unlock(args->statesMutex_pt);

    //Display contact states
    std::cerr << "Contact states: ";
    for (std::vector<int>::iterator it=states.begin(); it!=states.end(); it++)
      std::cerr << *it;
    std::cerr << std::endl;

    double PCR = QualPCR::evaluatePCR(args->g, *(args->wrench), args->maxForce, states, false);

    //Lock result mutex and check if we have improved on PGR
    pthread_mutex_lock(args->resultMutex_pt);
    if (PCR > args->maxPCR) {
      args->maxPCR = PCR;
      args->finalStates = states;
    } 
    pthread_mutex_unlock(args->resultMutex_pt);
  }
}

/*!
  Builds a PCR qm parameter area within the quality measure dialog
  box such that the user can input the applied wrench and the 
  desired maximum contact force
*/
void
QualPGR::buildParamArea(qmDlgDataT *qmData)
{
  static QualPGRParamT params;
  QualPGR *currQM = (QualPGR*)qmData->currQM;

#ifdef GRASPITDBG
  std::cout << "building QualPGR" << std::endl;
#endif

  std::vector<double> wrench(6, 0.0);
  double wrenchMultiplier = 1.0;
  double maxForce = 1.0;
  int maxContacts = 8;
  if (currQM) {
    wrench = currQM->mWrench;
    wrenchMultiplier = currQM->mWrenchMultiplier;
    maxForce = currQM->mMaxForce;
    maxContacts = currQM->mMaxContacts;
  } else if (qmData->grasp->isGravitySet()) {
    wrench = QualPCR::getGravityWrench(qmData->grasp);
    wrenchMultiplier = qmData->grasp->getObject()->getMass() * 1.0e-3 * 9.80665 * 1.0e6;
  } 

  QGridLayout *wl = new QGridLayout(qmData->settingsArea,8,2);

  wl->addWidget(new QLabel(QString("Multiplier:")));
  params.wrenchInput = new QLineEdit();
  params.wrenchInput->setText(QString::number(wrenchMultiplier));
  wl->addWidget(params.wrenchInput);

  wl->addWidget(new QLabel(QString("Force X:")));
  params.FxInput = new QLineEdit();
  params.FxInput->setText(QString::number(wrench[0]));
  wl->addWidget(params.FxInput);

  wl->addWidget(new QLabel(QString("Force Y:")));
  params.FyInput = new QLineEdit();
  params.FyInput->setText(QString::number(wrench[1]));
  wl->addWidget(params.FyInput);

  wl->addWidget(new QLabel(QString("Force Z:")));
  params.FzInput = new QLineEdit();
  params.FzInput->setText(QString::number(wrench[2]));
  wl->addWidget(params.FzInput);

  wl->addWidget(new QLabel(QString("Torque X:")));
  params.MxInput = new QLineEdit();
  params.MxInput->setText(QString::number(wrench[3]));
  wl->addWidget(params.MxInput);

  wl->addWidget(new QLabel(QString("Torque Y:")));
  params.MyInput = new QLineEdit();
  params.MyInput->setText(QString::number(wrench[4]));
  wl->addWidget(params.MyInput);

  wl->addWidget(new QLabel(QString("Torque Z:")));
  params.MzInput = new QLineEdit();
  params.MzInput->setText(QString::number(wrench[5]));
  wl->addWidget(params.MzInput);

  wl->addWidget(new QLabel(QString("Max. Force:")));
  params.maxForceInput = new QLineEdit();
  params.maxForceInput->setText(QString::number(maxForce));
  wl->addWidget(params.maxForceInput);

  wl->addWidget(new QLabel(QString("Max. no. of contacts:")));
  params.maxContactsInput = new QLineEdit();
  params.maxContactsInput->setText(QString::number(maxContacts));
  wl->addWidget(params.maxContactsInput);

  qmData->paramPtr = &params;
}
