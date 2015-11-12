/*
* handAdjust.cpp
*
*  Created on: Jun 20, 2011
*      Author: dang
*/

#include "handAdjust.h"

#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "matvec3D.h"
#include "SensorInterface.h"
#include "DBPlanner/sql_database_manager.h"
#include "debug.h"
#include "searchState.h"
#include "grasp.h"
#include "stdlib.h"
#include "qualityEstimator.h"
#include "handControlAPI.h"
//#define PROF_ENABLED
#include "profiling.h"

//#include <windows.h>
//#include <tchar.h>
#include <string>

#include "debug.h"

PROF_DECLARE(HAND_ADJUSTMENT);
PROF_DECLARE(AUTO_GRASP);
PROF_DECLARE(QUICK_OPEN);
PROF_DECLARE(ENTIRE_HAND_ADJUSTMENT);

/*
This function only searches based on the densly sampled experience database
*/
transf HandAdjust::getHandAdjustment2(GraspExperienceEntry nn, double radius, double epsilon)
{
	GraspExperienceEntry gee = mGeb->getNNBasedOnPose(nn, radius, epsilon);
	//std::cout << "==========" << std::endl;
	//std::cout << "go from: ";
	//nn.printMe(false);
	//std::cout << "to the neighbor: " << std::endl;
	//gee.printMe(false);
	//std::cout << gee.getPose() * nn.getPose().inverse() << std::endl;
	return gee.getPose() * nn.getPose().inverse();
}

transf HandAdjust::getHandAdjustmentONLINE(double *spread, char *writeTo, bool inSampleMode)
{

//how shall we load in the environment: CGDB should be using USECGDB, worldfile should be use USEWORLDFILE
#define USECGDB
//#define USEWORLDFILE

//hard code it to be 1 if we use this function to generate a TED
#define NUM_NEIGHBORS 5

/*	BASED_ON_CONTACT: adjustment is represented as roll pitch and yaw, where is the center?
	here we are computing the cente rof the contacts
	BASED_ON_WORLD_COORDINATE: we assume a generic pose error model based on world coordinate system
*/
#define BASED_ON_CONTACT
//#define BASED_ON_WORLD_COORDINATE

//AURO experiment use follows:
/*
#define USEWORLDFILE
#define BASED_ON_WORLD_COORDINATE
#define NUM_NEIGHBORS 1
*/
//-------------------------------------

	PROF_START_TIMER(ENTIRE_HAND_ADJUSTMENT);
	PROF_RESET(ENTIRE_HAND_ADJUSTMENT);
	//save the world
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	transf handTran = h->getTran();
	/*
	if we are in sample mode, we do not want to save the object
	simply because we do not need to restore it later
	*/
	clearWorld(!inSampleMode);
	//the current world should be empty now, only the hand is in it

	//the data to be stored while iterating through all the objects
	double minimum_distance = mGraspNeighbors[0].getDistance();
	transf adjustTF = transf::IDENTITY;
	double new_spread = getCurrentHandSpread();

	mAdjustmentInfo.reset();
	transf adjustmentPerNN;
	double spreadAdjustmentPerNN;
	int targetNN = 0;
	std::vector<double> originalQualityPerNN;
	originalQualityPerNN.resize(2);
	originalQualityPerNN[0] = mGraspNeighbors[0].getEpsilonQuality();
	originalQualityPerNN[1] = mGraspNeighbors[0].getVolumeQuality();

	for(size_t i = 0; i < NUM_NEIGHBORS; ++i) //test 5 neighbors
	{
		adjustmentPerNN = transf::IDENTITY;
		//load the ideal one on the NN model

#ifdef USECGDB
		//load the NN/ideal grasp in the database
		GraspitDBGrasp* gr = loadNNGraspFromCGDB(i);
		//////get the current grasp
		////GraspPlanningState pregraspState(gr->getPreGraspPlanningState()), finalgraspState(gr->getFinalGraspPlanningState());

		////pregraspState.setObject(mTestbody);
		//////shape the grasp to the pre-grasp
		////pregraspState.execute();
#endif
#ifdef USEPOSELIST
		loadNNFromPoseList(i);
#endif
#ifdef USEWORLDFILE
		loadNNFromWorldFile(i);
		w = graspItGUI->getIVmgr()->getWorld();
		h = w->getCurrentHand();
#endif

		//store where the hand was when the ideal grasp is applied
		transf idealTran = w->getCurrentHand()->getTran();
		//std::cout << "ideal tran is: " << idealTran << std::endl;

		//generate perturbation
		std::vector<transf> perturbationList;
#ifdef BASED_ON_CONTACT
		Hand * h2 = w->getCurrentHand();
		position diff(0,0,0);
		std::list<Contact *> contactList = w->getGB(0)->getContacts();
		for(std::list<Contact *>::iterator cp = contactList.begin(); cp != contactList.end(); ++cp)
		{
			Contact* ct = *(cp);

			//compute the contact within hand coordinate system
			diff = diff + ct->getPosition() * w->getGB(0)->getTran() * h2->getTran().inverse();//contactInObject * objectInWorld * worldInHand
			//std::cout << diff[0] << ", " << diff[1] << ", " << diff[2] << std::endl;
		}
		vec3 diff2(diff[0]/contactList.size(), diff[1]/contactList.size(), diff[2]/contactList.size());
		double contactRange = diff2.len();

		perturbationList = getPerturbationListBasedOnContacts(10, 10, 10, 5, contactRange);
		//perturbationList = getPerturbationListBasedOnContacts(5, 5, 5, 10, contactRange);
		//perturbationList = getPerturbationListBasedOnContacts(15, 15, 15, 30, contactRange);
		//perturbationList = getPerturbationListBasedOnContacts(15, 15, 15, 10, contactRange);

#endif

/* sometimes we just want to simulate world error based on world coordinate system, this is used for precomputing TED
*/
#ifdef 	BASED_ON_WORLD_COORDINATE
		/*rock, library_cup, box, snapple, the coordinate system of these objects are the reference coordinate system*/
		//perturbationList = getPertrubationListBasedOnObject(30, 30, 20, vec3::X, vec3::Y, vec3::Z, false, 0);
		/*all bottle and shampoo*/
		//perturbationList = getPertrubationListBasedOnObject(30, 30, 20, vec3::Y, -vec3::X, vec3::Z, false, 0);

		/*shape primitives - box, elipsoid, sphere*/
		perturbationList = getPertrubationListBasedOnObject(30, 30, 20, -vec3::Z, vec3::Y, vec3::X, false, 0);
		/*shape primitives - cylinder*/
		//perturbationList = getPertrubationListBasedOnObject(30, 30, 20, -vec3::Z, -vec3::X, vec3::Y, false, 0);
#endif

		std::cout << "perturbation size is " << perturbationList.size() << std::endl;
		//mGraspNeighbors[0].printMe();
		int combinationalPerturbationIndex = 0;

		for(size_t pInd = 0; pInd < perturbationList.size(); ++pInd)
		{

			//std::cout << "checking perturbation: " << pInd << std::endl;
			transf p = perturbationList[pInd];

			//test for different spread angle as well
			double spread_start, spread_end;
			if(mGraspNeighbors[i].getSpread() > mActualGrasp.getSpread())
			{
				spread_start = mActualGrasp.getSpread();
				spread_end = mGraspNeighbors[i].getSpread();
			}
			else
			{
				spread_start = mGraspNeighbors[i].getSpread();
				spread_end = mActualGrasp.getSpread();
			}
			double spread_interval = (spread_end-spread_start)/3.0;//always test with three intervals in spread angle
			//if the interval is too small < 3 degrees, we make it 3 degrees
			if(spread_interval < 3*M_PI/180.0)
				spread_interval = 3*M_PI/180.0;

			//std::cout << spread_end << " " << spread_start << " " << (spread_end - spread_start) / spread_interval << std::endl;

			//override all the relative info if we are in sample mode to generate
			if(inSampleMode)
			{
				spread_start = mGraspNeighbors[i].getSpread() - 5*M_PI/180.0;
				spread_end = mGraspNeighbors[i].getSpread() + 5*M_PI/180.0;
				spread_interval = 5*M_PI/180.0;
				if(spread_start < 0.00001)
					spread_start = 0.00001;
			}

			for(double tmp_spread = spread_start; tmp_spread < spread_end + 0.5*spread_interval; tmp_spread += spread_interval)
			{
				PROF_START_TIMER(HAND_ADJUSTMENT);
				PROF_RESET(HAND_ADJUSTMENT);

				if(!perturbAndGraspObject(p * idealTran, tmp_spread))
				{
					continue;
				}

				//store perturbation info
				if(writeTo)
				{
					char id[128];
					FILE *fp = fopen(writeTo, "a");
					sprintf(id, "%s_%d", mGraspNeighbors[i].getGraspInfo().getString().c_str(), //NN id
						combinationalPerturbationIndex //perturbation id
						);
					transf p_prime = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran() * idealTran.inverse();
					vec3 d = p_prime.translation() - p.translation();
					if( d.len() > 0.1 )
						std::cout << "caution: not the same! " << combinationalPerturbationIndex << std::endl;
					fprintf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf\n",
						id,
						p_prime.translation().x(),
						p_prime.translation().y(),
						p_prime.translation().z(),
						p_prime.rotation().w,
						p_prime.rotation().x,
						p_prime.rotation().y,
						p_prime.rotation().z,
						graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getDOF(0)->getVal() //new hand spread angle
						);
					fclose(fp);
					
					char index[128];
					sprintf(index,"%d", combinationalPerturbationIndex);
					writeOutAsTactileExperience(mGraspNeighbors[i].getGraspInfo().getString(), std::string(index));
					combinationalPerturbationIndex++;
					continue;
				}

				PROF_PRINT(HAND_ADJUSTMENT);

				//synthesize the current grasp after perturbation
				GraspExperienceEntry gee;
				std::vector<ContactInfo> cl = getCurrentGraspContactInfoList();
				gee.setContactList(cl);
				gee.setSpread(getCurrentHandSpread());

				PROF_STOP_TIMER(HAND_ADJUSTMENT);
				PROF_PRINT(HAND_ADJUSTMENT);

				//compare the tactile distance
				double distance = mActualGrasp.getTactileSimilarityMeasure(gee);
				if(distance > mGraspNeighbors[i].getDistance())
				{
					//std::cout << "distance: " << distance << " > " << mGraspNeighbors[i].getDistance() << std::endl;
					continue;
				}

				if(distance < minimum_distance)
				{
					std::cout << "Candidate found on NN: " << i << ", perturbation id: " << pInd << ", spread: " << tmp_spread <<
						", distance: " << distance << ", minimum distance so far: " << minimum_distance << std::endl;
					minimum_distance = distance;
					transf perturbed = w->getCurrentHand()->getTran();
					adjustTF = idealTran * perturbed.inverse();
					new_spread = tmp_spread;

					//record adjustment information
					adjustmentPerNN = adjustTF;
					spreadAdjustmentPerNN = new_spread;
					targetNN = i;
					//std::cout << "new adjustTF: " << adjustTF << std::endl;
				}
				else
				{
					//std::cout << "distance is: " << distance << std::endl;
				}

			}//one spread is done

		}// one perturbation is done here

		//when every NN is tested, we record some relevant information
		mAdjustmentInfo.adjustmentPerNN.push_back(adjustmentPerNN);
		mAdjustmentInfo.spreadAdjustmentPerNN.push_back(spreadAdjustmentPerNN);
		mAdjustmentInfo.neighborIdPerNN.push_back(mGraspNeighbors[i].getGraspInfo().getString());
		mAdjustmentInfo.distancePerNN.push_back(mGraspNeighbors[i].getDistance());
		mAdjustmentInfo.targetNN = targetNN;
	}//one NN is done

	/*
	if we are in sample mode, we do not need to restore the world
	since everytime it is a new test independent of the previous one
	*/
	if(!inSampleMode)
	{
		//restore the world
		restoreWorld();
	}

	//return the adjustment parameters
	std::cout << "minimum distance: " << minimum_distance << " adjustTF returned: " << adjustTF << std::endl;
	if(spread)
		*spread = new_spread;
	PROF_STOP_TIMER(ENTIRE_HAND_ADJUSTMENT);
	PROF_PRINT(ENTIRE_HAND_ADJUSTMENT);
	return adjustTF;
}

bool HandAdjust::graspObjectNew(transf t, double spread)
{

	PROF_START_TIMER(QUICK_OPEN);
	PROF_RESET(QUICK_OPEN);

	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();

	PROF_PRINT(QUICK_OPEN);
	//perturb the hand at the ideal place with the ideal spread angle
	std::cout << "hand new: " << t << std::endl;
	h->setTran(t);

	PROF_PRINT(QUICK_OPEN);
	//change the spread angle
	h->forceDOFVal(0, spread);

	PROF_PRINT(QUICK_OPEN);

	//move dof out of contact
	if(!h->quickOpen())
	{
		//if cannot open hand, check collision, may have self collision
		if(!w->noCollision())
		{
			std::cout << "collision in pre-grasp position" << std::endl;
			//move back only when necessary
			if(!w->getCurrentHand()->findInitialContact(100))
			{
				std::cout << "cannot resolve collision" << std::endl;
				PROF_STOP_TIMER(QUICK_OPEN);
				PROF_PRINT(QUICK_OPEN);
				return false;
			}
			else //move forward and autograsp
			{
				w->getCurrentHand()->approachToContact(30);
				w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole());
				return true;
			}
		}
	}

	PROF_STOP_TIMER(QUICK_OPEN);
	PROF_PRINT(QUICK_OPEN);

	/*
	//auto grasp, do not render is we are using console				
	//close hand to its first collision
	*/

	w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole()); //320ms
	return true;
}

bool HandAdjust::perturbAndGraspObject(transf t, double spread)
{
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	/*
	Here we could have problem when reshaping the hand
	Since forceDofVal will not work starting from collision
	*/
	//reset the hand pose to zero and clear the breakaway flags
	resetHandPoseToZero(); //200ms
	//printf("%lf, += %lf\n", tmp_spread, spread_interval);

	PROF_PRINT(HAND_ADJUSTMENT);
	//shape the spread angle, be careful if the spread angle is close to 0, it could be self collision
	h->forceDOFVal(0,spread);

	PROF_PRINT(HAND_ADJUSTMENT);
	//perturb the hand at the ideal place with the ideal spread angle
	//std::cout << "hand old: " << t << std::endl;
	h->setTran(t);

	/*Set joint vals to last non-contact positition from previous perturbation
	w->setJointVals(non_contact_joint_vals)
	//open hand until collisions are resolved
	w->resolve_collision
	*/
	PROF_PRINT(HAND_ADJUSTMENT);
	//move until losing the contact with the object if we start in collision
	if(!w->noCollision())
	{
		//std::cout << "collision in pre-grasp position" << std::endl;
		//move back only when necessary
		if(!w->getCurrentHand()->findInitialContact(100))
		{
			std::cout << "cannot resolve collision from pre-grasp position" << std::endl;
			return false;
		}
	}

	PROF_PRINT(HAND_ADJUSTMENT);
	/*
	//auto grasp, do not render is we are using console				
	//close hand to its first collision
	*/
	w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole(), -1);
	w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole()); //320ms
	return true;
}

/*
perturbation in hand coordinate system
*/
std::vector<transf> HandAdjust::getPertrubationListBasedOnHand(double rx, double ty, double tz)
{
	std::vector<transf> transfList;

	double step_t = 5; //5mm
	double step_r = 5; //5degree
	vec3 x = vec3(1.0,0,0);
	vec3 y =  vec3(0,1.0,0);
	vec3 z =  vec3(0,0,1.0);

	transfList.clear();
	for(double i = -rx; i < rx+1; i+=step_r)
	{
		for(double j = -ty; j < ty+1; j+=step_t)
		{
			for(double k = -tz; k < tz+1; k+=step_t)
			{
				transf rot(Quaternion(i*M_PI/180.0, x), vec3::ZERO);
				transf transl(Quaternion::IDENTITY, vec3(0, j, k));
				transfList.push_back(rot * transl);
			}
		}
	}
	return transfList;
}

std::vector<transf> HandAdjust::getPertrubationListBasedOnObject(double tx, double ty, double rz, vec3 x, vec3 y, vec3 z, bool considerRolling, double roll)
{
	std::vector<transf> transfList, result;

	double step_t = 5; //5mm
	double step_r = 5; //5degree
	double step_roll = 5; //5degree

	GraspableBody * gb = graspItGUI->getIVmgr()->getWorld()->getGB(0);
	transf objectInWorld = gb->getTran();
	transf handInWorld = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();

	transfList.clear();
	for(double i = -tx; i < tx+1; i+=step_t)
	{
		for(double j = -ty; j < ty+1; j+=step_t)
		{
			for(double k = -rz; k < rz+1; k+=step_r)
			{
				transf rot(Quaternion(k*M_PI/180.0, z), vec3::ZERO);
				transf transl(Quaternion::IDENTITY, i*x + j*y);
				transf perturbInObject = rot * transl;
				transf objectInWorldAfterPerturbation = perturbInObject * objectInWorld;
				transf handInObject = handInWorld * objectInWorld.inverse();
				transf handInWorldAfterObjectPerturbation = handInObject * objectInWorldAfterPerturbation;
				transf newHandInOldHand = handInWorldAfterObjectPerturbation * handInWorld.inverse();
				transfList.push_back(newHandInOldHand);
			}
		}
	}
	
	if(!considerRolling)
		result = transfList;
	else
	{
		result.clear();
		for(int i = 0; i < transfList.size(); ++i)
		{
			transf tmp = transfList[i];
			for(double j = - roll; j < roll+1; j+=step_roll)
			{
				transf roll(Quaternion(j*M_PI/180.0, vec3::Z), vec3::ZERO);
				result.push_back(roll * tmp);
			}
		}
	}

	return result;
}

std::vector<transf> HandAdjust::getPerturbationListBasedOnContacts(double latitude, double longitude, double rolling, double depth, double centerDistance)
{

	std::vector<transf> transfList;

	World * w = graspItGUI->getIVmgr()->getWorld();
	GraspableBody * body = w->getGB(0);

	double maxRadius = body->getMaxRadius();
	//3 dimensional latitude, latitude, rolling
	//assuming starting from the ideal grasp pose
	transf move_z;

	if(centerDistance < 0)
		move_z = transf(Quaternion::IDENTITY, vec3(0.0, 0.0, maxRadius));
	else
		move_z = transf(Quaternion::IDENTITY, vec3(0.0, 0.0, centerDistance));

	//std::cout << move_z << std::endl;

	vec3 x = vec3(1.0,0,0);
	vec3 y =  vec3(0,1.0,0);
	vec3 z =  vec3(0,0,1.0);

	double step = 5.0;

	for(double i = -latitude; i < latitude+1; i+=step) //every 2.5 degrees
	{
		for(double j = -longitude; j < longitude+1; j+=step) //every 2.5 degrees
		{
			for(double k = -rolling; k < rolling+1; k+=step) //every 2.5 degrees
			{
				for(double d = -depth; d < depth+1; d +=step) //every 2.5 mm along the approaching direction
				{
					transf Rot_x = transf(Quaternion(i * M_PI / 180.0,x),vec3(0,0,0));
					transf Rot_y = transf(Quaternion(j * M_PI / 180.0,y),vec3(0,0,0));
					transf Rot_z = transf(Quaternion(k * M_PI / 180.0,z),vec3(0,0,0));
					transf Transl_d = transf(Quaternion::IDENTITY, vec3(0,0,d));
					transf t = Transl_d * move_z.inverse() * Rot_z * Rot_y * Rot_x * move_z;
					transfList.push_back(t);
				}
			}
		}
	}

	return transfList;
}

void HandAdjust::clearWorld(bool save)
{
	//std::cout << "deleting object ..." << std::endl;
	World * w = graspItGUI->getIVmgr()->getWorld();
	transf handTran = w->getCurrentHand()->getTran();
	if(w->getNumGB() > 1)
	{
		std::cout << "there are more than one graspable body, please delete ...";
	}
	else if(w->getNumGB() == 0)
	{
		std::cout << "there is no graspable body";
		mHandPose = w->getCurrentHand()->getBase()->getTran();
		mHandJoint.resize(w->getCurrentHand()->getNumJoints());
		w->getCurrentHand()->getJointValues(&mHandJoint[0]);
		mBody = NULL;
	}
	else
	{
		mBody = w->getGB(0);
		std::cout << "deleted object is: " << mBody->getName().toStdString() << std::endl;
		w->destroyElement(mBody, !save); //if save = true, we won't destroy the object and we can restore it later
		mHandPose = w->getCurrentHand()->getBase()->getTran();
		mHandJoint.resize(w->getCurrentHand()->getNumJoints());
		w->getCurrentHand()->getJointValues(&mHandJoint[0]);
	}
}

/*
CAUTION:
If save = false when calling clearWorld, we cannot restoreWorld at a later time
*/
void HandAdjust::restoreWorld()
{
	World * w = graspItGUI->getIVmgr()->getWorld();

	//clear the current world
	if(w->getNumGB() > 0)
		w->destroyElement(w->getGB(0), false);

	//restore the object and the hand
	if(mBody)
	{
		mBody->addToIvc();
		//todo: where to dynamic information come from?
		//model->getGraspableBody()->initDynamics();
		//this adds the object to the graspit world so that we can see it
		w->addBody(mBody);
	}
	w->getCurrentHand()->setTran(mHandPose);
	w->getCurrentHand()->setJointValuesAndUpdate(&mHandJoint[0]);
}

void HandAdjust::loadNNFromWorldFile(int i)
{
	std::string filename = mGraspNeighbors[i].getGraspInfo().objectName;
	char worldpath[128];
	sprintf(worldpath, "%s/worlds/simple/%s", getenv("GRASPIT"), filename.c_str());
	
	graspItGUI->getIVmgr()->emptyWorld();
	
	graspItGUI->getIVmgr()->getWorld()->load(worldpath);
	
	mTestbody = graspItGUI->getIVmgr()->getWorld()->getGB(0);
}

void HandAdjust::loadNNModelFromCGDB(int i)
{
	World * w = graspItGUI->getIVmgr()->getWorld();

	if(w->getNumGB() > 0)
		w->destroyElement(w->getGB(0), false);

	//get the neighbor info
	std::string objectName = mGraspNeighbors[i].getGraspInfo().objectName;
	int index = mGraspNeighbors[i].getGraspInfo().graspIndex;

	std::cout << "===>>>examining grasp on object: " << objectName << " and grasp_id is: " << index << std::endl;
	//load the object
	GraspitDBModel* model = getModel(objectName);
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(w) != SUCCESS) {
			DBGA("Model load failed");
		}
	}
	mTestbody= model->getGraspableBody();
	//adds the object to the collision detection system
	mTestbody->addToIvc();
	//todo: where to dynamic information come from?
	//model->getGraspableBody()->initDynamics();
	//this adds the object to the graspit world so that we can see it
	w->addBody(model->getGraspableBody());
}

transf HandAdjust::loadNNFromPoseList(int i)
{
	resetHandPoseToZero();
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran( mGraspNeighbors[i].getPose() );
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->forceDOFVal(0, mGraspNeighbors[i].getSpread());

	loadNNModelFromCGDB(i);
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(!graspItGUI->useConsole());
	return mGraspNeighbors[i].getPose();
}

GraspitDBGrasp* HandAdjust::loadNNGraspFromCGDB(int i)
{
	//load the object
	loadNNModelFromCGDB(i);
	//load the grasp
	std::string objectName = mGraspNeighbors[i].getGraspInfo().objectName;
	int index = mGraspNeighbors[i].getGraspInfo().graspIndex;
	GraspitDBGrasp* gr = getGrasp(objectName, index);
	gr->getPreGraspPlanningState()->execute();
	/*
	//auto grasp, do not render is we are using console				
	//close hand to its first collision
	*/
	World *w = graspItGUI->getIVmgr()->getWorld();
	w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole(), -1);
	w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole()); //320ms
	return gr;
}

GraspitDBModel* HandAdjust::getModel(std::string objectName)
{
	db_planner::Model* dbmodel;
	for(size_t i = 0; i < mModelList.size(); ++i)
	{
		if( !strcmp(mModelList[i]->ModelName().c_str(), objectName.c_str()) ) //find the object
		{
			dbmodel = mModelList[i];
		}
	}

	if(!dbmodel)
	{
		std::cout << "object not found" << std::endl;
		return NULL;
	}
	return static_cast<GraspitDBModel*>(dbmodel);
}

GraspitDBGrasp* HandAdjust::getGrasp(std::string objectName, int index)
{
	db_planner::Model* dbmodel;
	for(size_t i = 0; i < mModelList.size(); ++i)
	{
		if( !strcmp(mModelList[i]->ModelName().c_str(), objectName.c_str()) ) //find the object
		{
			dbmodel = mModelList[i];
		}
	}

	if(!dbmodel)
	{
		std::cout << "object not found" << std::endl;
		return NULL;
	}

	//clear the grasp list
	mGraspList.clear();

	if(!mDBMgr->GetGrasps(*dbmodel,GraspitDBGrasp::getHandDBName(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()).toStdString(), &mGraspList)){
		DBGA("Load grasps failed");
		mGraspList.clear();
		return NULL;
	}
    DBGA("hello");
	std::cout << "grasps retrieved: " << mGraspList.size() << std::endl;
	return static_cast<GraspitDBGrasp*>(mGraspList[index]);
}

void HandAdjust::init(GraspExperienceBase * geb)
{
	if(geb)
	{
		mGeb = geb;
		std::cout << "new geb" << std::endl;
	}

	if(mIsInitialized)
		return;

	if(!mDBMgr)
	{
		mDBMgr = graspItGUI->getIVmgr()->getDBMgr();
		std::cout << "try dbmgr in the world" << std::endl;
	}
	if (mDBMgr)
	{
		std::cout << "Database connected" << std::endl;
	}
	else
	{
		std::cout << "Database not connected" << std::endl;
		mIsInitialized = false;
		return;
	}

	//load the models from database manager
	if(!mDBMgr->ModelList(&mModelList,db_planner::FilterList::NONE)){
		DBGA("Model list retrieval failed");
		mIsInitialized = false;
		return;
	}
	mIsInitialized = true;
}

void HandAdjust::connectToDB(QString dbURL, QString dbName, QString dbPort, QString dbUserName, QString dbPassword)
{
	if(!mDBMgr)
	{
		mDBMgr = new db_planner::SqlDatabaseManager(dbURL.toStdString(),
			dbPort.toInt(),
			dbUserName.toStdString(),
			dbPassword.toStdString(),
			dbName.toStdString(),
			new GraspitDBModelAllocator(),
			new GraspitDBGraspAllocator(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()));
	}

	if (mDBMgr->isConnected()) {
		std::cout << "CGDB connected" << std::endl;
	}
}

transf HandAdjust::getHandAdjustmentUsePrecomputedPerturbation(GraspExperienceEntry queryGrasp, double* spread)
{

	//look around
	int NNNum = 5;

	transf t;

	//synthesize a sub-grasp experience database based on the grasp names in the folder
	std::vector<std::string> nameSubset;
	//get the grasp name list for precomputed grasps
	QString dictionary_name(getenv("GRASPIT"));
	dictionary_name += QString("/experience_precomp");
	//sprintf(directory_name,"%s/experience_precomp",getenv("GRASPIT"));
	char prefix[256];
#ifdef FILTER
	sprintf(prefix,"perturb_%s", graspItGUI->getIVmgr()->getWorld()->getGB(0)->getName().ascii());
#else
	sprintf(prefix,"perturb_");
#endif

	std::cout << "prefix is: " << prefix << std::endl;
	getFileNames(nameSubset, dictionary_name.toStdString().c_str(), prefix);
	//the sub-grasps experience database
	std::vector<GraspExperienceEntry> validExperienceSubset;
	validExperienceSubset.clear();
	for(size_t i = 0; i < nameSubset.size(); ++i)
	{
		validExperienceSubset.push_back(mGeb->getEntry(nameSubset[i].substr(8, nameSubset[i].size() - 12)));
		std::cout << nameSubset[i].c_str() << std::endl;
	}

	//temporary geb for computing level-1 NN
	GraspExperienceBase geb;
	geb.setGraspEntryList(validExperienceSubset);
	std::vector<GraspExperienceEntry> queryGraspNNList;
	queryGraspNNList = geb.getKNNBasedOnTactile(queryGrasp);
	
	//extra test for neighbor distance
//#define EXTRA_CHECK
#ifdef EXTRA_CHECK
	std::cout << "NN dist: " << queryGraspNNList[0].getDistance() << std::endl;
	if(queryGraspNNList[0].getDistance() < 10.0)
		return transf::IDENTITY;

	QualityEstimator* qe;
	qe = new QualityEstimator();
	qe->setTactileReadings(getTactileSensorReadings());
	//setup the svm model trained from libsvm
	qe->setModel("modeli00");
	//setup the path to the codebook and normalizer
	qe->setCodebook("codebooki00.txt");
	//setup the current grasp tactile sensing data
	HandControlAPI hc;
	qe->setTactileLocations(hc.getTactileLocations());

	double e;
	e =  qe->estimateStability();
	if(e > 0.5)
	{
		std::cout << "positive quality" << std::endl;
		return transf::IDENTITY;
	}
#endif

//	for(int i = 0; i < queryGraspNNList.size(); ++i)
//	{
//		std::cout << queryGraspNNList[i].getInfoString() << std::endl;
//	}

	//search within the neighborhood of each NN
	double bestDist = -1;
	GraspExperienceEntry bestLocal;
	transf bestPerturbation;

	for(int i = 0; i < NNNum && i < queryGraspNNList.size(); ++i)
	{
		//temporary geb for computing level-2 NN
		GraspExperienceBase geb2;
		char expath[512], hppath[512];
		sprintf(expath, "%s/experience_precomp/perturbed_experience_cgdb_%s.txt", QString(getenv("GRASPIT")).toStdString().c_str(), queryGraspNNList[i].getInfoString().c_str());
		std::cout << expath << std::endl;
		//handpose is specified as the perturbation files so that the loaded hand pose will be the perturbation we applied to the grasp
		sprintf(hppath, "%s/experience_precomp/perturb_%s.txt", QString(getenv("GRASPIT")).toStdString().c_str(), queryGraspNNList[i].getInfoString().c_str());
		geb2.loadGraspExperienceBase(expath, 1);
		geb2.loadHandPoses(hppath);

		std::cout << "analyzing with " << queryGraspNNList[i].getInfoString() << std::endl;
		//tmpList is a ranked list of the NN's
		std::vector<GraspExperienceEntry> tmpList = geb2.getKNNBasedOnTactile(queryGrasp);

//only considers the distance to the NN as the 
#ifndef COMPLEX_DISTANCE
		double refDistance = tmpList[0].getDistance();
#else
		double refDistance = 0;
		int RefNum = 5;
		for(int n = 0; n < RefNum; ++n)
			refDistance += tmpList[n].getDistance();
#endif

		if( queryGraspNNList[i].getDistance() < tmpList[0].getDistance() )
		{
			std::cout << "no better localization: level 0 distance is smaller than level 1 distance" << std::endl;
		}
		else if( refDistance < bestDist || bestDist == -1 )
		{
			bestDist = refDistance;
			bestLocal = tmpList[0];
			//bestPerturbation = geb2.getPose(bestLocal.getInfoString());
			bestPerturbation = geb2.getWeightedPose(tmpList, 2.0); //consider a combination of potential pose adjustment
			std::cout << ": a better reference distance: " << bestDist << " from ideal grasp " << bestLocal.getInfoString() << " with pert: " << bestPerturbation;
		}
		else
		{
			std::cout << ": a worse reference distance: " << refDistance;
		}
		std::cout << std::endl;
	}
	std::cout << "best dist: " << bestDist << " from ideal grasp " << bestLocal.getInfoString() << std::endl;
	std::cout << "best adjust: " << bestPerturbation.inverse() << std::endl;
	*spread = bestLocal.getSpread();
	return bestPerturbation.inverse();
}
