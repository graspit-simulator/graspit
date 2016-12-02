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

QMutex mtx_;

#define SLOP 0.001

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



//TEST(TEST_PLANNER, TEST_VALID_GRASP_FOUND) {
//  int argc = 0;
//  GraspItApp app(argc, NULL);
//  GraspitCore core(argc, NULL);

//  app.setMainWidget(core.getMainWindow()->mWindow);

//  QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

//  graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
//  graspitCore->getWorld()->load(fileName);
//  graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

//  int num_bodies = graspitCore->getWorld()->getNumBodies();
//  EXPECT_EQ(num_bodies, 10);

//  QMDlg *d = new QMDlg(graspitCore->getMainWindow()->mWindow);
//  d->show();
//  d->addEditQM();
//  d->accept();

//  PlannerDlg *dlg = new PlannerDlg(graspitCore->getMainWindow()->mWindow);
//  dlg->setAttribute(Qt::WA_ShowModal, false);
//  dlg->setAttribute(Qt::WA_DeleteOnClose, true);
//  dlg->show();

//  dlg->generateGrasps();
//  dlg->visualizeBox->setChecked(true);

//  pthread_t thread1;
//  pthread_t thread2;
//  int iret2 = pthread_create( &thread1, NULL, run_test, (PlannerDlg*) dlg);
//  int iret1 = pthread_create( &thread2, NULL, exit_graspit, (GraspitCore*) &graspitCore);

//  graspitCore->startMainLoop();

//  pthread_join( thread1, NULL);
//  pthread_join( thread2, NULL);
//  Q_UNUSED(iret1);
//  Q_UNUSED(iret2);

//}

//TEST(TEST_TRANSFORMS, TEST_TRANSFORM_OPERATIONS) {

//  vec3 x = vec3(1,0,0);
//  transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
//  transf translate = transf::TRANSLATION(vec3(1,2,3));
//  position point = position(0.1, 0.2, 0.3);
//  vec3 norm = vec3(0.1, 0.2, 0.3).normalized();

//  norm = (rot % translate).affine() * (norm);
//  point = (rot % translate) * (point);

//  vec3 xcrossnorm = x.cross(norm);
//  double xdotnorm = x.dot(norm);

//  ASSERT_NEAR(norm.x(), 0.267261, SLOP);
//  ASSERT_NEAR(norm.y(), -0.0123153, SLOP);
//  ASSERT_NEAR(norm.z(), -0.963545, SLOP);

//  ASSERT_NEAR(point.x(), 1.1, SLOP);
//  ASSERT_NEAR(point.y(), -0.0506877, SLOP);
//  ASSERT_NEAR(point.z(), -3.96578, SLOP);

//  ASSERT_NEAR(xcrossnorm.x(), 0.0, SLOP);
//  ASSERT_NEAR(xcrossnorm.y(), 0.963545, SLOP);
//  ASSERT_NEAR(xcrossnorm.z(), -0.0123153, SLOP);

//  ASSERT_NEAR(xdotnorm, 0.267261, SLOP);

//}

//TEST(TEST_TRANSFORMS, TEST_TRANSFORM_OPERATIONS2) {
//  //making sure transform class works as expected.
//  //from peter's notes pg 5
//  //http://www.cs.columbia.edu/~allen/F15/NOTES/forwardspong.pdf

//  double d01 = 0.1;
//  double d12 = 0.2;
//  double d23 = 0.3;

//  transf A_01 = transf(mat3::Identity(), vec3(0,0,d01));
//  mat3 r_12;
//  r_12.col(0) = vec3(1,0,0);
//  r_12.col(1) = vec3(0,0,-1);
//  r_12.col(2) = vec3(0,1,0);

//  transf A_12 = transf(r_12, vec3(0,0,d12));
//  transf A_23 = transf(mat3::Identity(), vec3(0,0,d23));

//  transf A_03 = A_01 % (A_12 % A_23);

//  ASSERT_NEAR(A_03.translation().x(), 0.0, SLOP);
//  ASSERT_NEAR(A_03.translation().y(), 0.3, SLOP);
//  ASSERT_NEAR(A_03.translation().z(), 0.3, SLOP);

//  ASSERT_NEAR(A_03.rotation().w(), 0.707107, SLOP);
//  ASSERT_NEAR(A_03.rotation().x(), -0.707107, SLOP);
//  ASSERT_NEAR(A_03.rotation().y(), 0.0, SLOP);
//  ASSERT_NEAR(A_03.rotation().z(), 0.0, SLOP);

//  position p_0 = position (0.1, 0.2, 0.3);
//  position p_3 = A_03 * (p_0);

//  ASSERT_NEAR(p_3.x(), 0.1, SLOP);
//  ASSERT_NEAR(p_3.y(), 0.6, SLOP);
//  ASSERT_NEAR(p_3.z(), 0.1, SLOP);

//}


//TEST(TEST_COLLISION, TEST_BODY_TO_BODY_DISTANCE) {

//    QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

//    graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
//    graspitCore->getWorld()->load(fileName);
//    graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

//    vec3 x = vec3(1,0,0);
//    transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
//    transf translate = transf::TRANSLATION(vec3(1,2,3));


//    graspitCore->getWorld()->getGB(0)->setTran(translate % rot);
//    Body *mug = graspitCore->getWorld()->getGB(0);
//    Body *palm = graspitCore->getWorld()->getRobot(0)->getBase();
//    position p1, p2;
//    double dist = graspitCore->getWorld()->getCollisionInterface()->bodyToBodyDistance(mug,palm, p1, p2);

//    ASSERT_NEAR(dist, -1, SLOP);

//    ASSERT_NEAR(p1.x(), -1, SLOP);
//    ASSERT_NEAR(p1.y(), 3.31021, SLOP);
//    ASSERT_NEAR(p1.z(), 1.42917, SLOP);

//    ASSERT_NEAR(p2.x(), 0.0, SLOP);
//    ASSERT_NEAR(p2.y(), 0.0, SLOP);
//    ASSERT_NEAR(p2.z(), 0.0, SLOP);

//}


//TEST(TEST_COLLISION, TEST_POINT_TO_BODY_DISTANCE) {

//    QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

//    graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
//    graspitCore->getWorld()->load(fileName);
//    graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

//    vec3 x = vec3(1,0,0);
//    transf rot = transf::AXIS_ANGLE_ROTATION(10, x);
//    transf translate = transf::TRANSLATION(vec3(1,2,3));

//    graspitCore->getWorld()->getGB(0)->setTran(translate % rot);
//    Body *mug = graspitCore->getWorld()->getGB(0);
//    Body *palm = graspitCore->getWorld()->getRobot(0)->getBase();
//    position p1(10,11,12);
//    position clostestPoint;
//    vec3 closestNormal;
//    double dist = graspitCore->getWorld()->getCollisionInterface()->pointToBodyDistance(mug,p1, clostestPoint, closestNormal);

//    std::cout << "dist: " << dist << std::endl;
//    std::cout << "closestNormal:" << closestNormal << std::endl;
//    std::cout << "clostestPoint:" << clostestPoint << std::endl;

//    ASSERT_NEAR(dist, 33.8332, SLOP);

//    ASSERT_NEAR(closestNormal.x(), 0.951057, SLOP);
//    ASSERT_NEAR(closestNormal.y(), -0.168109, SLOP);
//    ASSERT_NEAR(closestNormal.z(), 0.259287, SLOP);

//    ASSERT_NEAR(clostestPoint.x(), 42.1773, SLOP);
//    ASSERT_NEAR(clostestPoint.y(), 5.31235, SLOP);
//    ASSERT_NEAR(clostestPoint.z(), 20.7725, SLOP);
//}

//TEST(TEST_COLLISION, TEST_GET_BOUNDING_VOLUMES) {

//  if (graspitCore->getWorld()->getNumGB() != 0){
//      graspitCore->emptyWorld();
//  }

//  QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));

//  graspitCore->getMainWindow()->mUI->worldBox->setTitle(fileName);
//  graspitCore->getWorld()->load(fileName);
//  graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());

//  Body *mug = graspitCore->getWorld()->getGB(0);

//  std::vector<BoundingBox> mug_bvs;
//  graspitCore->getWorld()->getCollisionInterface()->getBoundingVolumes(mug,0, &mug_bvs);
//  ASSERT_EQ(mug_bvs.size(), 1);

//  transf mugBboxTran = mug_bvs.at(0).getTran() ;

//  ASSERT_NEAR(mugBboxTran.translation().x(), -11.662, SLOP);
//  ASSERT_NEAR(mugBboxTran.translation().y(), -8.98983, SLOP);
//  ASSERT_NEAR(mugBboxTran.translation().z(), 0.000190189, SLOP);

//  EXPECT_NEAR(mugBboxTran.rotation().w(), 0.999999, SLOP);
//  EXPECT_NEAR(mugBboxTran.rotation().x(), -9.15765e-06, SLOP);
//  EXPECT_NEAR(mugBboxTran.rotation().y(), -1.47701e-06, SLOP);
//  EXPECT_NEAR(mugBboxTran.rotation().z(), 0.00157698, SLOP);


//  Body *palm = graspitCore->getWorld()->getRobot(0)->getBase();
//  std::vector<BoundingBox> palm_bvs;
//  graspitCore->getWorld()->getCollisionInterface()->getBoundingVolumes(palm,0, &palm_bvs);
//  ASSERT_EQ(palm_bvs.size(), 1);

//  transf palmBboxTran = palm_bvs.at(0).getTran() ;

//  ASSERT_NEAR(palmBboxTran.translation().x(), 0.013166, SLOP);
//  ASSERT_NEAR(palmBboxTran.translation().y(), -9.91786, SLOP);
//  ASSERT_NEAR(palmBboxTran.translation().z(), -38.5818, SLOP);

//  ASSERT_NEAR(palmBboxTran.rotation().w(), 0.6664499, SLOP);
//  ASSERT_NEAR(palmBboxTran.rotation().x(), 0.236398, SLOP);
//  ASSERT_NEAR(palmBboxTran.rotation().y(), -0.66585779, SLOP);
//  ASSERT_NEAR(palmBboxTran.rotation().z(), -0.23789390, SLOP);

//}


//TEST(TEST_QUATERNION, TEST_QUATERNION) {
//  Eigen::AngleAxisd aa = Eigen::AngleAxisd(10, vec3(1,1,1).normalized());
//  Quaternion q(aa);

//  mat3 R = q.toRotationMatrix();
//  Quaternion q2 = Quaternion(R);
//  std::cout << "q.w()" <<q.w() << std::endl;
//  std::cout << "q.x()" <<q.x() << std::endl;
//  std::cout << "q.y()" <<q.y() << std::endl;
//  std::cout << "q.z()" <<q.z() << std::endl;
//  std::cout << R << std::endl;
//  std::cout << R(0) << std::endl;
//  std::cout << R(1) << std::endl;
//  std::cout << R(2) << std::endl;
//  std::cout << R(3) << std::endl;
//  std::cout << R(4) << std::endl;
//  std::cout << R(5) << std::endl;
//  std::cout << R(6) << std::endl;
//  std::cout << R(7) << std::endl;
//  std::cout << R(8) << std::endl;
//  std::cout << "q2.w()" <<q2.w() << std::endl;
//  std::cout << "q2.x()" <<q2.x() << std::endl;
//  std::cout << "q2.y()" <<q2.y() << std::endl;
//  std::cout << "q2.z()" <<q2.z() << std::endl;

//  std::cout << "R * vec3(0,0,1) " <<R * vec3(0,0,1) << std::endl;
//  std::cout << "q * vec3(0,0,1) " << q * vec3(0,0,1) << std::endl;

//  ASSERT_NEAR(q.w(), -q2.w(), SLOP);
//  ASSERT_NEAR(q.x(), -q2.x(), SLOP);
//  ASSERT_NEAR(q.y(), -q2.y(), SLOP);
//  ASSERT_NEAR(q.z(), -q2.z(), SLOP);
//}

TEST(TEST_LEAF, TEST_LEAF) {

  position v1= position(0,0,1);
  position v2= position(1,0,1);
  position v3= position(0,2,1);
  Triangle * t0 = new Triangle(v1,v2,v3);

  transf t = transf::AXIS_ANGLE_ROTATION(10, vec3(1,1,1).normalized());
  t0->applyTransform(t);

  Eigen::AngleAxisd aa = Eigen::AngleAxisd(10, vec3(1,1,1).normalized());
  Quaternion q(aa);

  position centroid  = t0->centroid();
  vec3 normal  = t0->normal();

  ASSERT_NEAR(centroid.x(), 0.84166, SLOP);
  ASSERT_NEAR(centroid.y(), 0.87606, SLOP);
  ASSERT_NEAR(centroid.z(), 0.282279, SLOP);

  ASSERT_NEAR(normal.x(), 0.298933, SLOP);
  ASSERT_NEAR(normal.y(), 0.927115, SLOP);
  ASSERT_NEAR(normal.z(), -0.226048, SLOP);

  Collision::Leaf *leaf = new Collision::Leaf();
  leaf->addTriangle(*t0);
  leaf->computeBbox();
  BoundingBox b = leaf->getBox();
  transf bboxTran = b.getTran();

  ASSERT_NEAR(bboxTran.translation().x(), 1.09786, SLOP);
  ASSERT_NEAR(bboxTran.translation().y(), 0.786607, SLOP);
  ASSERT_NEAR(bboxTran.translation().z(), 0.254205, SLOP);

  ASSERT_NEAR(bboxTran.rotation().w(), 0.774406, SLOP);
  ASSERT_NEAR(bboxTran.rotation().x(), 0.613771, SLOP);
  ASSERT_NEAR(bboxTran.rotation().y(), -0.102141, SLOP);
  ASSERT_NEAR(bboxTran.rotation().z(), -0.114663, SLOP);

  position closest_point = closestPtBbox(b, position(4,5,6));
  double dist  = pointBoxDistanceSq(b, position(4,5,6));

  ASSERT_NEAR(closest_point.x(), 2.18595 , SLOP);
  ASSERT_NEAR(closest_point.y(), 0.566747, SLOP);
  ASSERT_NEAR(closest_point.z(), 0.74717, SLOP);
  ASSERT_NEAR(dist, 50.536730, SLOP);

  position v4 = position(10,0,1);
  position v5 = position(1,10,1);
  position v6 = position(0,2,11);
  Triangle * t1 = new Triangle(v1,v2,v3);
  Collision::Leaf *leaf2 = new Collision::Leaf();
  BoundingBox b2 = leaf2->getBox();
  leaf2->addTriangle(*t1);

  //transf mTran2To1 = b2.getTran().inverse() %  b.getTran();
  transf mTran2To1 = b.getTran().inverse() % b2.getTran() ;
  double bboxDistanceSqDist = bboxDistanceSq(b, b2, mTran2To1 );

  std::cout << "bboxDistanceSqDist" << bboxDistanceSqDist << std::endl;
  ASSERT_NEAR(bboxDistanceSqDist, 3.71454, SLOP);

}

//TEST(TEST_APPROACH_CONTACTS, TEST_APPROACH_CONTACTS) {

//    if (graspitCore->getWorld()->getNumGB() != 0){
//        graspitCore->emptyWorld();
//    }

//    QString fileName = QString(QString(getenv("GRASPIT"))+QString("/worlds/") + QString("plannerMug.xml"));
//    graspitCore->getWorld()->load(fileName);

//    Hand *h = (Hand*)graspitCore->getWorld()->getRobot(0);
//    h->autoGrasp(false);

//    std::list<Contact *> cs = h->getContacts();

////    for (std::list<Contact *>::iterator it=cs.begin(); it != cs.end(); ++it)
////      {
////         Contact *c = (*it);
////         std::cout << c->getBody1()->name() << std::endl;
////         std::cout << c->getFrame() << std::endl;
////      }

//    EXPECT_EQ(graspitCore->getWorld()->getNumGB(), 1);
//    EXPECT_EQ(cs.size(), 3);

//}



int main(int argc, char **argv)
{
  GraspItApp app(argc, NULL);
  GraspitCore core(argc, NULL);

  app.setMainWidget(core.getMainWindow()->mWindow);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
