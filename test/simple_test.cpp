#include <gtest/gtest.h>
#include <iostream>       // std::cout

#include <QString>

#include "graspitGUI.h"
#include <graspitApp.h>
#include "mainWindow.h"
#include "ivmgr.h"
#include "world.h"
#include "plannerdlg.h"
#include "quality.h"
#include "robot.h"
#include "qmDlg.h"
#include <pthread.h>

#include <Inventor/Qt/SoQt.h>


void *foo( void *ptr );

void *foo(void *ptr )
{

    GraspItGUI *gui;
  gui = (GraspItGUI *) ptr;

  std::cout << "about to exit0" << std::endl;
  sleep(10);

  std::cout << "about to exit1" << std::endl;

  gui->exitMainLoop();
  //SoQt::done();

}


TEST(SIMPLE_TEST, EXAMPLE_TEST) {
  int argc = 0;
  GraspItApp app(argc, NULL);
  GraspItGUI gui(0, NULL);

  app.setMainWidget(gui.getMainWindow()->mWindow);

  QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

  gui.getMainWindow()->mUI->worldBox->setTitle(fileName);
  graspItGUI->getIVmgr()->emptyWorld();
  graspItGUI->getIVmgr()->getWorld()->load(fileName);
  gui.getMainWindow()->setMainWorld(gui.getIVmgr()->getWorld());

  int num_bodies = graspItGUI->getIVmgr()->getWorld()->getNumBodies();
  EXPECT_EQ(num_bodies, 10);


  //Grasp *g = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getGrasp();

  QMDlg *d = new QMDlg(gui.getMainWindow()->mWindow);
  d->show();
  d->addEditQM();
  d->accept();

  PlannerDlg *dlg = new PlannerDlg(gui.getMainWindow()->mWindow);
  dlg->setAttribute(Qt::WA_ShowModal, false);
  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
  dlg->show();

  dlg->generateGrasps();
  dlg->visualizeBox->setChecked(true);
  dlg->testGrasps();


  pthread_t thread1;
  int iret1 = pthread_create( &thread1, NULL, foo, (GraspItGUI*) &gui);

  gui.startMainLoop();

  dlg->showGrasp();
  pthread_join( thread1, NULL);

//  PlannerDlg *dlg = new PlannerDlg(gui.getMainWindow()->mWindow);
//  dlg->setAttribute(Qt::WA_ShowModal, false);
//  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
//  dlg->show();


  //QualEpsilon *qm = new QualEpsilon(g, QString("epsilon_quality_measure"), QualityMeasure::TYPE_LIST[0]);


//  qmDlgDataT *qmDlgData = new qmDlgDataT();
//  qmDlgData->grasp = g;
//  qmDlgData->qmType ="Epsilon";

//  QualityMeasure::createInstance(qmDlgData);




}




int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
