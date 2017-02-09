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
#include "collisionInterface.h"
#include "collisionStructures.h"
#include "collisionModel.h"
#include "Graspit/collisionAlgorithms.h"
#include "bBox.h"

#define SLOP 0.001

TEST(TEST_COLLISION, TEST_BODY_TO_BODY_DISTANCE) {

    QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

    graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
    graspitCore->getWorld()->load(fileName);
    graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

    vec3 x = vec3(1,0,0);
    transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
    transf translate = transf::TRANSLATION(vec3(1,2,3));


    graspitCore->getWorld()->getGB(0)->setTran(translate % rot);
    Body *mug = graspitCore->getWorld()->getGB(0);
    Body *palm = graspitCore->getWorld()->getRobot(0)->getBase();
    position p1, p2;
    double dist = graspitCore->getWorld()->getCollisionInterface()->bodyToBodyDistance(mug,palm, p1, p2);

    ASSERT_NEAR(dist, -1, SLOP);

    ASSERT_NEAR(p1.x(), -1, SLOP);
    ASSERT_NEAR(p1.y(), 3.31021, SLOP);
    ASSERT_NEAR(p1.z(), 1.42917, SLOP);

    ASSERT_NEAR(p2.x(), 0.0, SLOP);
    ASSERT_NEAR(p2.y(), 0.0, SLOP);
    ASSERT_NEAR(p2.z(), 0.0, SLOP);

}


TEST(TEST_COLLISION, TEST_POINT_TO_BODY_DISTANCE) {

    QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

    graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
    graspitCore->getWorld()->load(fileName);
    graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

    vec3 x = vec3(1,0,0);
    transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
    transf translate = transf::TRANSLATION(vec3(1,2,3));

    graspitCore->getWorld()->getGB(0)->setTran(translate % rot);
    Body *mug = graspitCore->getWorld()->getGB(0);
    Body *palm = graspitCore->getWorld()->getRobot(0)->getBase();
    position p1(10,11,12);
    position clostestPoint;
    vec3 closestNormal;
    double dist = graspitCore->getWorld()->getCollisionInterface()->pointToBodyDistance(mug,p1, clostestPoint, closestNormal);

    std::cout << "dist: " << dist << std::endl;
    std::cout << "closestNormal:" << closestNormal << std::endl;
    std::cout << "clostestPoint:" << clostestPoint << std::endl;

    ASSERT_NEAR(dist, 33.8332, SLOP);

    ASSERT_NEAR(closestNormal.x(), 0.951057, SLOP);
    ASSERT_NEAR(closestNormal.y(), -0.168109, SLOP);
    ASSERT_NEAR(closestNormal.z(), 0.259287, SLOP);

    ASSERT_NEAR(clostestPoint.x(), 42.1773, SLOP);
    ASSERT_NEAR(clostestPoint.y(), 5.31235, SLOP);
    ASSERT_NEAR(clostestPoint.z(), 20.7725, SLOP);
}

TEST(TEST_COLLISION, TEST_GET_BOUNDING_VOLUMES) {

  if (graspitCore->getWorld()->getNumGB() != 0){
      graspitCore->emptyWorld();
  }

  QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

  graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
  graspitCore->getWorld()->load(fileName);
  graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

  Body *mug = graspitCore->getWorld()->getGB(0);

  std::vector<BoundingBox> mug_bvs;
  graspitCore->getWorld()->getCollisionInterface()->getBoundingVolumes(mug,0, &mug_bvs);
  ASSERT_EQ(mug_bvs.size(), 1);

  transf mugBboxTran = mug_bvs.at(0).getTran() ;

  ASSERT_NEAR(mugBboxTran.translation().x(), -11.662, SLOP);
  ASSERT_NEAR(mugBboxTran.translation().y(), -8.98983, SLOP);
  ASSERT_NEAR(mugBboxTran.translation().z(), 0.000190189, SLOP);

  EXPECT_NEAR(mugBboxTran.rotation().w(), 0.999999, SLOP);
  EXPECT_NEAR(mugBboxTran.rotation().x(), -9.15765e-06, SLOP);
  EXPECT_NEAR(mugBboxTran.rotation().y(), -1.47701e-06, SLOP);
  EXPECT_NEAR(mugBboxTran.rotation().z(), 0.00157698, SLOP);


  Body *palm = graspitCore->getWorld()->getRobot(0)->getBase();
  std::vector<BoundingBox> palm_bvs;
  graspitCore->getWorld()->getCollisionInterface()->getBoundingVolumes(palm,0, &palm_bvs);
  ASSERT_EQ(palm_bvs.size(), 1);

  transf palmBboxTran = palm_bvs.at(0).getTran() ;

  ASSERT_NEAR(palmBboxTran.translation().x(), 0.013166, SLOP);
  ASSERT_NEAR(palmBboxTran.translation().y(), -9.91786, SLOP);
  ASSERT_NEAR(palmBboxTran.translation().z(), -38.5818, SLOP);

  ASSERT_NEAR(palmBboxTran.rotation().w(), 0.6664499, SLOP);
  ASSERT_NEAR(palmBboxTran.rotation().x(), 0.236398, SLOP);
  ASSERT_NEAR(palmBboxTran.rotation().y(), -0.66585779, SLOP);
  ASSERT_NEAR(palmBboxTran.rotation().z(), -0.23789390, SLOP);

}

int main(int argc, char **argv)
{
  GraspItApp app(argc, NULL);
  GraspitCore core(argc, NULL);

  app.setMainWidget(core.getMainWindow()->mWindow);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
