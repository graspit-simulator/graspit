#include <gtest/gtest.h>
#include <iostream>
#include <QString>
#include <pthread.h>
#include <QMutex>
#include <Inventor/Qt/SoQt.h>

#include "graspitCore.h"
#include <graspitApp.h>
#include "mainWindow.h"
#include "ivmgr.h"
#include "world.h"
#include "plannerdlg.h"
#include "quality.h"
#include "robot.h"
#include "qmDlg.h"
#include "grasp.h"

QMutex mtx_;

void *exit_graspit( void *ptr );
void *run_test(void *ptr );

void *exit_graspit(void *ptr )
{
  //sleep a moment to ensure that start is called first
  sleep(3);

  //hang here until tests are finished and mtx is released.
  QMutexLocker guard(&mtx_);

  graspitCore->exitMainLoop();

  return NULL;
}


void *run_test(void *ptr )
{
  QMutexLocker guard(&mtx_);

  PlannerDlg *dlg;
  dlg = (PlannerDlg *) ptr;

  dlg->testGrasps();
  while(!dlg->ShowButton->isEnabled())
  {
      sleep(3);
  }

  Grasp *grasp=graspitCore->getWorld()->getCurrentHand()->getGrasp();

  QString fileName = QString("models/objects/mug.xml");
  EXPECT_STREQ(grasp->getObject()->getFilename().toStdString().c_str(), fileName.toStdString().c_str());

  std::cout << "Grasp has " << grasp->getNumContacts() << " contacts" << std::endl;
  EXPECT_GT(grasp->getNumContacts(), 0);

  std::cout << "Grasp has quality measure " << grasp->getQM(0)->evaluate() << std::endl;
  EXPECT_GT(grasp->getQM(0)->evaluate(), 0);

  dlg->close();
  sleep(1);

  return NULL;
}



TEST(TEST_PLANNER, TEST_VALID_GRASP_FOUND) {
  int argc = 0;
  GraspItApp app(argc, NULL);
  GraspitCore core(argc, NULL);

  app.setMainWidget(core.getMainWindow()->mWindow);

  QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

  graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
  graspitCore->getWorld()->load(fileName);
  graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

  int num_bodies = graspitCore->getWorld()->getNumBodies();
  EXPECT_EQ(num_bodies, 10);

  QMDlg *d = new QMDlg(graspitCore->getMainWindow()->mWindow);
  d->show();
  d->addEditQM();
  d->accept();

  PlannerDlg *dlg = new PlannerDlg(graspitCore->getMainWindow()->mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();

  dlg->generateGrasps();
  dlg->visualizeBox->setChecked(true);

  pthread_t thread1;
  pthread_t thread2;
  int iret2 = pthread_create( &thread1, NULL, run_test, (PlannerDlg*) dlg);
  int iret1 = pthread_create( &thread2, NULL, exit_graspit, (GraspitCore*) &graspitCore);

  graspitCore->startMainLoop();

  pthread_join( thread1, NULL);
  pthread_join( thread2, NULL);
  Q_UNUSED(iret1);
  Q_UNUSED(iret2);

}


int main(int argc, char **argv)
{
  GraspItApp app(argc, NULL);
  GraspitCore core(argc, NULL);

  app.setMainWidget(core.getMainWindow()->mWindow);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
