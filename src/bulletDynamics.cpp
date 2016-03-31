//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2015  Columbia University in the City of New York.
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
// $Id:$
//
//######################################################################

#include "bulletDynamics.h"

#include <btBulletDynamicsCommon.h>
#include <btBvhTriangleMeshShape.h>
#include <btTriangleMesh.h>
#include <btGImpactShape.h>
#include <btGImpactCollisionAlgorithm.h>
#include <btHingeConstraint.h>
#include <btGearConstraint.h>

#include "body.h"
#include "dynJoint.h"
#include "robot.h"
#include "triangle.h"
#include "world.h"

#include "debug.h"
#include "dynamics.h"
#include "humanHand.h"
BulletDynamics::BulletDynamics(World *world)
{

    mWorld = world;

    // collision configuration contains default setup for memory, collision setup.
    btDefaultCollisionConfiguration* collisionConfiguration;
    collisionConfiguration= new btDefaultCollisionConfiguration();

    // use the default collision dispatcher.
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

    // btDbvtBroadphase is a good general purpose broadphase. .
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    //the default constraint solver.
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    mBtDynamicsWorld =
            new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

    mBtDynamicsWorld->setGravity(btVector3(0,0, 0));
}

BulletDynamics::~BulletDynamics()
{
    delete mBtDynamicsWorld;
}

void BulletDynamics::addBody(Body *newBody)
{
    // Creation of CollisionShape
    btTriangleMesh* triMesh = new btTriangleMesh(true,true);//true,true);

    // Get the geometry data form the Graspit object
    std::vector<Triangle> triangles;

    newBody->getGeometryTriangles(&triangles);
    int numTriangles = triangles.size();
    Triangle tritemp = triangles.at(0);

    for(int i = 0; i < numTriangles-1; i=i+1)
    {
        tritemp = triangles.at(i);
        btScalar v01(tritemp.v1[0]);
        btScalar v02(tritemp.v1[1]);
        btScalar v03(tritemp.v1[2]);
        btScalar v11(tritemp.v2[0]);
        btScalar v12(tritemp.v2[1]);
        btScalar v13(tritemp.v2[2]);
        btScalar v21(tritemp.v3[0]);
        btScalar v22(tritemp.v3[1]);
        btScalar v23(tritemp.v3[2]);

        btVector3 v0(v01,v02,v03);
        btVector3 v1(v11,v12,v13);
        btVector3 v2(v21,v22,v23);
        triMesh->btTriangleMesh::addTriangle(v0,v1,v2,true);
    }

    btCollisionShape* triMeshShape;
    btScalar mass(0.);
    btVector3 localInertia(0,0,0);

    if (newBody->isDynamic())
    {
        mass = ((DynamicBody*)newBody)->getMass(); //mass g
        mass=mass/1000; //kg
        triMeshShape = new btGImpactMeshShape(triMesh);
        ((btGImpactMeshShape*)triMeshShape)->updateBound();
        triMeshShape->calculateLocalInertia(mass,localInertia);
        printf("localInertia: %lf %lf %lf  kg.m^2\n", localInertia.x()/1e6,localInertia.y()/1e6,localInertia.z()/1e6);
    }
    else
    {
        triMeshShape = new btBvhTriangleMeshShape(triMesh, true,true);
    }

    //using motionstate is recommended, it provides interpolation capabilities,
    //and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0 , 0)));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,triMeshShape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    // avoid deactive
    body->setSleepingThresholds(0,0);

    //add the body to the dynamics world
    mBtDynamicsWorld->addRigidBody(body);

    mBtLinks.push_back(body);
    //add the newlink and body to the map
    btBodyMap.insert(btBodyPair(newBody,body));
}

void BulletDynamics::addRobot(Robot *robot)
{
    btRigidBody* btbase=NULL;

    if (robot->getBase()) {
        btScalar mass(0.);
        btVector3 localInertia(0, 0, 0);
        if ((btbase=btBodyMap.find(robot->getBase())->second) == NULL) {
            printf("error, base is not in the btBodyMap\n");
        }
        btbase->setMassProps(mass , localInertia);
    }
    printf("### of m_links: %d \n ", mBtLinks.size());

    for (int f=0; f <robot->getNumChains(); f++) {

        int numberLinks = robot->getChain(f)->getNumLinks();

        // get the transfrom from the origin of the palm to the base of this chain
        transf chaintransf = robot->getChain(f)->getTran();

        vec3 chaintranslation = chaintransf.translation();
        printf("chain %d TRANSLATION %f,%f,%f \n", f,
               chaintranslation.x(), chaintranslation.y(), chaintranslation.z());
        btVector3 chainpos(chaintranslation.x(), chaintranslation.y(), chaintranslation.z());
        // calculate chain rotation, and to get base z,x,y in the frame of world frame
        //(palm origin frame)
        Quaternion rotq = chaintransf.rotation();
        vec3 zbaseinOrigin = rotq*vec3(0, 0, 1);
        vec3 xbaseinOrigin = rotq*vec3(1, 0, 0);
        vec3 ybaseinOrigin = rotq*vec3(0, 1, 0);
        btVector3 baseaxis(zbaseinOrigin.x(), zbaseinOrigin.y(), zbaseinOrigin.z());
        btVector3 xbaseaxis(xbaseinOrigin.x(), xbaseinOrigin.y(), xbaseinOrigin.z());
        btVector3 ybaseaxis(ybaseinOrigin.x(), ybaseinOrigin.y(), ybaseinOrigin.z());


        int jointind = 0;  // keep track current joint index
        for (int l=0; l <numberLinks; l++) {

            btRigidBody* btcurrentlink = btBodyMap.find(robot->getChain(f)->getLink(l))->second;
            btRigidBody* btprevlink;
            if (l > 0) {
                btprevlink = btBodyMap.find(robot->getChain(f)->getLink(l-1))->second;
            } else if (l == 0) {
                btprevlink = btbase;
            }

            printf("link#: %d \n ", l);
            // get the rotation axis in the frame of next joint
            vec3 proxjointaxis = robot->getChain(f)->getLink(l)->getProximalJointAxis();
            btVector3 currentProximalRotationAxis(proxjointaxis.x() , proxjointaxis.y() , proxjointaxis.z());
            printf("The link PROXIMAL JOINT AXIS:%f,%f,%f \n",
                   proxjointaxis.x() , proxjointaxis.y(), proxjointaxis.z());
            // get the proximal joint location in the frame of next joint
            position prolocation = robot->getChain(f)->getLink(l)->getProximalJointLocation();
            printf("proximalJoint localtion: %f, %f, %f \n",
                   prolocation.x(), prolocation.y(), prolocation.z());
            btVector3 currentProximalJointLocation(prolocation.x(), prolocation.y(), prolocation.z());

            DynJoint::DynamicJointT djtype = robot->getChain(f)->getLink(l)->getDynJoint()->getType();

            if (djtype == DynJoint::REVOLUTE) {
                printf("~~~~~~chain: %d link: %d  type: REVOLUTE \n", f, l);
                printf("jointind: %d \n ", jointind);
                Joint* j = robot->getChain(f)->getJoint(jointind);
                jointind++;

                btHingeConstraint* newjoint;
                btRigidBody* rbA ;
                btRigidBody* rbB ;
                btVector3 pivotInA;
                btVector3 pivotInB;
                btVector3 axisInA;
                btVector3 axisInB;
                bool useReferenceFrameA;

                btTransform frameInA;
                btTransform frameInB;

                position dislocation = robot->getChain(f)->getLink(l-1)->getDistalJointLocation();
                printf("DistalJointLocation location: %f, %f, %f  \n",
                       dislocation.x(), dislocation.y(), dislocation.z());
                btVector3 previousDistalJointLocation(dislocation.x(), dislocation.y(), dislocation.z());
                btVector3 previousDistalRotationAxis(0, 0, 1);

                //if this is the first link, then we need to set it with regard to the base
                if (l == 0) {
                    rbA = btbase ;
                    rbB = btcurrentlink;
                    pivotInA = chainpos;
                    pivotInB = currentProximalJointLocation;
                    axisInA= baseaxis;
                    axisInB = currentProximalRotationAxis;
                    useReferenceFrameA = false;

                    frameInA.setIdentity();
                    frameInB.setIdentity();

                    frameInA.setOrigin(chainpos);
                    frameInB.setOrigin(currentProximalJointLocation);
                }
                //set with regard to the previous link
                else {


                    rbA = btprevlink;
                    rbB = btcurrentlink;
                    pivotInA = previousDistalJointLocation;
                    pivotInB = currentProximalJointLocation;
                    axisInA = previousDistalRotationAxis;
                    axisInB = currentProximalRotationAxis;
                    useReferenceFrameA = false;

                    frameInA.setIdentity();
                    frameInB.setIdentity();

                    frameInA.setOrigin(btVector3(0, 0, 0));
                    frameInA.getBasis().setValue(1, 0, 0,
                                                 0, 1, 0,
                                                 0, 0, 1);
                    //frameInA.setOrigin(previousDistalJointLocation);
                    //frameInB.setOrigin(currentProximalJointLocation);

                }
                newjoint = new btHingeConstraint(*rbA ,
                                                 *rbB ,
                                                 pivotInA,
                                                 pivotInB,
                                                 axisInA,
                                                 axisInB,
                                                 useReferenceFrameA);


                //btJointMap.insert(btJointPair(j,newjoint));

                        double max = j->getMax();
                        double min = j->getMin();

//                        if (max < -M_PI)
//                        {
//                            max += (2*M_PI);
//                        }
//                        if (max > M_PI)
//                        {
//                            max -= (2*M_PI);
//                        }

//                        if (min < -M_PI)
//                        {
//                            min += (2*M_PI);
//                        }
//                        if (min > M_PI)
//                        {
//                            min -= (2*M_PI);
//                        }




                        //newjoint->setFrames(frameInA, frameInB);
                        //newjoint->setLimit(min,max);
                        std::cout << "Joint Limits: "<< min << " : " << max << " current: " << j->getVal()<< "current2: " << newjoint->getHingeAngle()<<  std::endl;


                bool disable_collision_between_rbA_rbB = true;
                mBtDynamicsWorld->addConstraint(newjoint , disable_collision_between_rbA_rbB);

            } else if (djtype == DynJoint::UNIVERSAL) {
                printf("~~~~~~chain: %d link: %d  type: UNIVERSAL \n", f, l);
                Joint* joint0 = robot->getChain(f)->getJoint(jointind);
                Joint* joint1 = robot->getChain(f)->getJoint(jointind+1);
                jointind+=2;  //universal: use two joints

                transf T1 = joint0->getTran();
                transf T2 = joint1->getTran();
                Quaternion rotqj01 = T1.rotation();
                Quaternion rotqj12 = T2.rotation();
                Quaternion rot02 = rotqj01*rotqj12;
                Quaternion rot20 = rot02.inverse();
                Quaternion rot21 = rotqj12.inverse();
                //get the joint0 x,y,z in new frame(after two transformation T1,T2)
                vec3 zjoint0new = rot20*vec3(0, 0, 1);
                vec3 xjoint0new = rot20*vec3(1, 0, 0);
                vec3 yjoint0new = rot20*vec3(0, 1, 0);
                btVector3 btzjoint0new(zjoint0new.x(), zjoint0new.y(), zjoint0new.z());
                btVector3 btxjoint0new(xjoint0new.x(), xjoint0new.y(), xjoint0new.z());
                btVector3 btyjoint0new(yjoint0new.x(), yjoint0new.y(), yjoint0new.z());
                printf("chain %d link: %d joint0 z in frame 2 %f,%f,%f \n",
                       f, l, zjoint0new.x(), zjoint0new.y(), zjoint0new.z());
                //get the joint1 x,y,z in new frame(after 1 transformation T2)
                vec3 zjoint1new = rot21*vec3(0, 0, 1);
                vec3 xjoint1new = rot21*vec3(1, 0, 0);
                vec3 yjoint1new = rot21*vec3(0, 1, 0);
                btVector3 btzjoint1new(zjoint1new.x(), zjoint1new.y(), zjoint1new.z());
                btVector3 btxjoint1new(xjoint1new.x(), xjoint1new.y(), xjoint1new.z());
                btVector3 btyjoint1new(yjoint1new.x(), yjoint1new.y(), yjoint1new.z());
                printf("chain %d link: %d joint1 z in frame2 %f,%f,%f \n",
                       f, l, zjoint1new.x(), zjoint1new.y(), zjoint1new.z());
                // joint1 z axis in the frame of joint0
                vec3 zjoint1infram0 = rotqj01*vec3(0, 0, 1);


                // universal constraint
                btTransform frameInA;
                btTransform frameInB;
                frameInA.setIdentity();
                frameInB.setIdentity();
                //set the constraint frameB: which is the joint0 x,y,z coordinates in new frame
                //(after 2 transformation T1,T2)
                frameInB.getBasis().setValue(btxjoint0new.x(), btyjoint0new.x(), btzjoint0new.x(),
                                             btxjoint0new.y(), btyjoint0new.y(), btzjoint0new.y(),
                                             btxjoint0new.z(), btyjoint0new.z(), btzjoint0new.z());
                frameInB.setOrigin(currentProximalJointLocation);
                btGeneric6DofConstraint* newjoint;

                if (l == 0) {  // connect with palm ,frameinA: palm
                    frameInA.getBasis().setValue(xbaseaxis.x(), ybaseaxis.x(), baseaxis.x(),
                                                 xbaseaxis.y(), ybaseaxis.y(), baseaxis.y(),
                                                 xbaseaxis.z(), ybaseaxis.z(), baseaxis.z());

                    frameInA.setOrigin(chainpos);
                    newjoint = new btGeneric6DofConstraint(*(btbase) , *(btcurrentlink),
                                                           frameInA, frameInB, false);
                } else {  // privous one is not palm, frameinA: previous link
                    frameInA.setOrigin(btVector3(0, 0, 0));
                    frameInA.getBasis().setValue(1, 0, 0,
                                                 0, 1, 0,
                                                 0, 0, 1);
                    newjoint = new btGeneric6DofConstraint(*(btprevlink), *(btcurrentlink),
                                                           frameInA, frameInB, false);
                }
                //set translation and rotation limits, they are based on joint0 frame
                //set x,y,z translation limit=0,0; 0,1,2-> x,y,z translation, 3,4,5-> x,y,z rotation.
                newjoint->setLimit(0, 0, 0);
                newjoint->setLimit(1, 0, 0);
                newjoint->setLimit(2, 0, 0);
                printf("chain %d joint1 z in joint0  frame %f,%f,%f \n",
                       f, zjoint1infram0.x(), zjoint1infram0.y(), zjoint1infram0.z());
                //universal constraints: two rotation axises, one is z axis of joint0,
                //the other one is the joint1 z axis in the frame of joint0 (x or y),
                //set the left one limit=0,0
                if (zjoint1infram0.x() != 0) {
                    printf("zjoint1 in joint0 frame is x, limit the y rotation \n ");
                    newjoint->setLimit(4, 0, 0);  // limit the y rotation;
                } else if (zjoint1infram0.y() != 0) {
                    printf("zjoint1 in joint0 frame is y, limit the x rotation \n ");
                    newjoint->setLimit(3, 0, 0);  // limit the x rotation;
                }

                mBtDynamicsWorld->addConstraint(newjoint, true);
            } else if (djtype == DynJoint::BALL) {
                printf("~~~~~~chain: %d link: %d  type: BALL \n", f, l);
                jointind+=3;
            } else if (djtype == DynJoint::PRISMATIC) {
                printf("~~~~~~chain: %d link: %d  type: PRISMATIC \n", f, l);
                jointind++;
            } else if (djtype == DynJoint::FIXED) {
                printf("~~~~~~chain: %d link: %d  type: FIXED \n", f, l);
                jointind++;
            }
        }  // for link
    }  // for chain

     //couple dof joints
    for (int i = 0; i < mWorld->getNumRobots(); i++) {
        Robot* robot=mWorld->getRobot(i);
        int numDOF=robot->getNumDOF();
        //for each dof, couple joints.
        for (int d=0;d<numDOF;d++) {

            DOF * dof=robot->getDOF(d);

            Joint* baseJoint = dof->getJointList().at(0);
            Link* baseLink = dynamic_cast<Link*>(baseJoint->getDynJoint()->getNextLink());
            std::cout << "baseLink->getName(): " << baseLink->getName().toStdString().c_str() << std::endl;
            btRigidBody* btbaseLink = btBodyMap.find(baseLink)->second;

            vec3 baseLinkaxis = baseLink->getProximalJointAxis();
            btVector3 btcbaseLinkaxis(baseLinkaxis.x() , baseLinkaxis.y() , baseLinkaxis.z());


            for (unsigned int j_count = 0; j_count < dof->getJointList().size(); j_count ++)
            {

                Joint *joint = dof->getJointList().at(j_count);

                Link* currentLink = dynamic_cast<Link*>(joint->getDynJoint()->getNextLink());

                btRigidBody* btcurrentLink = btBodyMap.find(currentLink)->second;
                vec3 currentLinkaxis = currentLink->getProximalJointAxis();
                btVector3 btcurrentLinkaxis(currentLinkaxis.x() , currentLinkaxis.y() , currentLinkaxis.z());

                    if(btcurrentLink != btbaseLink)
                    {
                        btGearConstraint* newGear = new btGearConstraint(*btbaseLink,
                                                                         *btcurrentLink,
                                                                         btcbaseLinkaxis,
                                                                         btcurrentLinkaxis,
                                                                         -joint->getCouplingRatio()*2.0);
                                                                         //joint->getCouplingRatio());
                        mBtDynamicsWorld->addConstraint(newGear);
                    }

            }
        }
    }
}

void BulletDynamics::turnOnDynamics()
{
    // Update BULLET TRANSFORM whenever Dynamics are turned on
    for (int j=mBtDynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--) {
        Body* tempbody = mWorld->getBody(j);
        transf temptrans = tempbody->getTran();

        btScalar wrot(temptrans.rotation().w);
        btScalar xrot(temptrans.rotation().x);
        btScalar yrot(temptrans.rotation().y);
        btScalar zrot(temptrans.rotation().z);

        btScalar xtrans(temptrans.translation().x());
        btScalar ytrans(temptrans.translation().y());
        btScalar ztrans(temptrans.translation().z());
        
        btCollisionObject* obj = mBtDynamicsWorld->getCollisionObjectArray()[j];
        btRigidBody* body = btRigidBody::upcast(obj);
        body->setCenterOfMassTransform(btTransform(btQuaternion( xrot , yrot , zrot , wrot),
                                                   btVector3(xtrans, ytrans, ztrans)));
        body->setLinearVelocity(btVector3(0 , 0 , 0));
        body->setAngularVelocity(btVector3(0 , 0 , 0));
    }
}

void BulletDynamics::turnOffDynamics()
{
}


void BulletDynamics::btApplyInternalWrench (Joint * activeJoint, double magnitude, std::map<Body*, btRigidBody*> btBodyMap) {
        //currently if magnitude is fixed, the hand moves fine no matter how it is oriented.
        //so I believe this function is fine, it is the magnitude that is incorrect.
//        magnitude = -0.1;
//        std::cout << "magnitude: " << magnitude <<std::endl;
        //assert(false);

        vec3 worldAxis=activeJoint->getWorldAxis();

        DynamicBody *db_prev = activeJoint->getDynJoint()->getPrevLink();
        btRigidBody* btPrevLink = btBodyMap.find(db_prev)->second;

        btVector3 torquePrev(-magnitude*worldAxis.x(),-magnitude*worldAxis.y(), -magnitude*worldAxis.z());
        btPrevLink->applyTorqueImpulse(torquePrev);

        DynamicBody *db_next = activeJoint->getDynJoint()->getNextLink();
        btRigidBody* btNextLink = btBodyMap.find(db_next)->second;

        btVector3 torqueNext(magnitude*worldAxis.x(),magnitude*worldAxis.y(), magnitude*worldAxis.z());
        btNextLink->applyTorqueImpulse(torqueNext);
}

int BulletDynamics::stepDynamics()
{
    double timeStep=1.0f/60.f;

    mBtDynamicsWorld->stepSimulation(timeStep,10);
    for (int j=mBtDynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
    {
        btCollisionObject* obj = mBtDynamicsWorld->getCollisionObjectArray()[j];
        btRigidBody* body = btRigidBody::upcast(obj);

        btTransform feedbacktransform = body->getCenterOfMassTransform();
        btQuaternion btrotation = feedbacktransform.getRotation();
        btVector3 bttranslation = feedbacktransform.getOrigin();

        Quaternion* rot = new Quaternion(btrotation.getAngle() ,
                                         vec3(btrotation.getAxis()[0] ,
                                         btrotation.getAxis()[1] ,
                btrotation.getAxis()[2]) );
        vec3* transl = new vec3(bttranslation.getX() , bttranslation.getY() , bttranslation.getZ());
        Quaternion rotfix = *rot;
        vec3 translfix = *transl;
        transf* temptrans2 = new transf(rotfix , translfix) ;

        Body* tempbody = mWorld->getBody(j);
        tempbody->setTran(*temptrans2);

        //update the bullet velocity to graspit velocity in order to calculate friction
        btVector3 btAngularVelocity=body->getAngularVelocity ();
        btVector3 btLinearVelocity=body->getLinearVelocity () ;

        /*! Sets the current 6x1 velocity vector [vx vy vz vrx vry vrz] */
        double newvelocity[6]={btLinearVelocity.x(), btLinearVelocity.y(),btLinearVelocity.z(),btAngularVelocity.x(), btAngularVelocity.y(),btAngularVelocity.z()};
        if(tempbody->isDynamic()){
            DynamicBody * tempdynbody=(DynamicBody *)tempbody;
            tempdynbody->setVelocity(newvelocity);
        }
    }
    // --------------------------add torque--------------------------------------------------

    mWorld->resetDynamicWrenches();

    for (int i = 0; i < mWorld->getNumRobots(); i++) {

        Robot* robot=mWorld->getRobot(i);

        //need to update the joint world axis, values, and velocities...
        //this currently recomputes the joint values even though bullet has already done this for us..
        robot->updateJointValuesFromDynamics();

        //apply torque to dofs
        for (int d=0; d<robot->getNumDOF(); d++) {

            DOF * dof=robot->getDOF(d);
            dof->updateSetPoint();
            dof->callController(timeStep);

            double magnitude=dof->getForce()/10e6;
            if (magnitude !=0)
            {
                 btApplyInternalWrench(dof->getJointList().at(0),  magnitude,  btBodyMap);
            }

            printf("DOF: %d getForce:%lf ,desired:%lf  \n",d, dof->getForce(), dof->getDesiredForce());
            printf("DOF: %d, Joint %d, apply torque: %lf N.mm  \n",d, 0, magnitude);
        }

        // --------------------------add friction--------------------------------------------------
        for (int c = 0; c < robot->getNumChains(); c++) {
            for (int j = 0; j < robot->getChain(c)->getNumJoints(); j++) {
                Joint *tempjoint=robot->getChain(c)->getJoint(j);

                std::cout << " current: j->getVal()" << tempjoint->getVal()<< std::endl;

                double frictionForce = tempjoint->getFriction()/10e6;
                btApplyInternalWrench(tempjoint,frictionForce,btBodyMap);

                double springForce = -tempjoint->getSpringForce();
                btApplyInternalWrench(tempjoint, springForce,btBodyMap);
            }
        }
    }

    return 0;
}



/*! One of the two main functions of the dynamics time step. This function is 
called to move the bodies according to the velocities and accelerations found
in the previous step. The move is attempted for the duration of the time step 
given in \a timeStep.

After all the bodies are moved, then collisions are checked. If any collisions
are found, the move is traced back and the value of the times step is 
interpolated until the exact moment of contact is found. The actual value
of the time step until contact is made is returned. If interpolation fails,
a negative actual time step is returned. All the resulting contacts are added
to the bodies that are in contact to be used for subsequent computations.

The same procedure is carried out if, by executing a full time step, a joint
ends up outside of its legal range.
*/
double BulletDynamics::moveDynamicBodies(double timeStep) {
    Q_UNUSED(timeStep);
    return 0;
}


/*! Asks the dynamics engine to compute the velocities of all bodies at
the current time step. These will be used in the next time step when
the bodies are moved by World::moveDynamicBodies.

The bodies are separated into "islands" of bodies connected by contacts 
or joints.  Two dynamic bodies are connected if they share a contact or
a joint.  Then for each island, this calls the iterate dynamics routine
to build and solve the LCP to find the velocities of all the bodies
in the next iteration.
*/  
int BulletDynamics::computeNewVelocities(double timeStep) {
    Q_UNUSED(timeStep)
    return 0;
}
