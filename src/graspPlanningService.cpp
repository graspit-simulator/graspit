
#include "graspPlanningService.h"

#include <set>
#include <string>
#include <algorithm>

#include "searchState.h"
#include "debug.h"

#include "graspitServer.h"
#include "robot.h"
#include "world.h"
#include "DBPlanner/sql_database_manager.h"
#include "graspit_db_model.h"
#include "graspit_db_grasp.h"
#include "searchEnergy.h"
#include "graspPlanningTask.h" 
#include "egPlannerUtils.h"
//#include "Tasks/graspAnalyzingTask.h"
#include "quality.h"
//#include "ui/graspViewerDlg.h"
#include "DBPlanner/grasp.h"

double BACKUP_DIST = 60;

bool compareGraspQM(db_planner::Grasp* g1, db_planner::Grasp* g2)
{
	return g1->EpsilonQuality() > g2->EpsilonQuality();
}

void GraspPlanningService::init()
{
	mDBMgr = new db_planner::SqlDatabaseManager(mdbaseURL.toStdString(),
		mdbasePort,
		mdbaseUserName.toStdString(),
		mdbasePassword.toStdString(),
		mdbaseName.toStdString(),
		new GraspitDBModelAllocator(),
		new GraspitDBGraspAllocator(mHand));

	if (mDBMgr->isConnected()) {
		std::cout << "CGDB connected" << std::endl;
		graspItGUI->getIVmgr()->setDBMgr(mDBMgr);
	}
}

void GraspPlanningService::setParams(QString hand, QString object, QString method_type, ClientSocket* clientSocket, transf t, const vec3 & approach_vector, double approach_tolerance_angle)
{
	//import a new plan parameters
	mHandName = hand;
	mObjectName = object;
	mMethodType = method_type;
	mSocket = clientSocket;
	mObjectPose = t;
	mIsParamSet = true;
	mApproachVector = approach_vector;
	mApproachAngle = approach_tolerance_angle;
	clearGraspList();

}

void GraspPlanningService::clearGraspList()
{
	for(size_t i = 0; i < mGraspList.size(); ++i)
		delete mGraspList[i];
	for(size_t i = 0; i < filteredGraspList.size(); ++i)
		delete filteredGraspList[i];
	mGraspList.clear();
	filteredGraspList.clear();
}


QString GraspPlanningService::report()
{
	//make these params as junk
	mIsParamSet = false;
	if(!strcmp(mMethodType.latin1(), "RETRIEVAL"))
	{
		std::vector<db_planner::Grasp*> tmpList;
		for(size_t i = 0; i < filteredGraspList.size() && i < 10; ++i)
		{
			tmpList.push_back(filteredGraspList[i]);
		}

		mMessage = synthesizeMsg(tmpList);
		return mMessage;
	}
	else if(!strcmp(mMethodType.latin1(), "RETRIEVAL_UNCERTAIN"))
	{
		mMessage = synthesizeMsg(filteredGraspList, mGraspSetCoverageList);
		return mMessage;
	}
	else
	{
		std::cout << "Only RETRIEVAL method is supported so far" << std::endl;
        return QString("NULL");
	}

}

void GraspPlanningService::transform()
{
	transf prePosition, transformedPrePosition, finalPosition, transformedFinalPosition;
	std::vector<double> tmp;

	for(size_t i = 0; i < filteredGraspList.size(); ++i)
	{
		//get the pre-grasp position, x,y,z,qw,qx,qy,qz
		tmp = filteredGraspList[i]->GetPregraspPosition();
		prePosition = transf(Quaternion(tmp[3],tmp[4],tmp[5],tmp[6]), vec3(tmp[0],tmp[1],tmp[2]));

		//get the final-grasp position, x,y,z,qw,qx,qy,qz
		tmp = filteredGraspList[i]->GetFinalgraspPosition();
		finalPosition = transf(Quaternion(tmp[3],tmp[4],tmp[5],tmp[6]), vec3(tmp[0],tmp[1],tmp[2]));

		//calculate the transformed positions
		transformedPrePosition = prePosition * mObjectPose;
		transformedFinalPosition = finalPosition * mObjectPose;

		//store back the transformed position
		tmp.clear();
		tmp.push_back(transformedPrePosition.translation().x());
		tmp.push_back(transformedPrePosition.translation().y());
		tmp.push_back(transformedPrePosition.translation().z());
		tmp.push_back(transformedPrePosition.rotation().w);
		tmp.push_back(transformedPrePosition.rotation().x);
		tmp.push_back(transformedPrePosition.rotation().y);
		tmp.push_back(transformedPrePosition.rotation().z);
		filteredGraspList[i]->SetPregraspPosition(tmp);

		//store back the transformed position
		tmp.clear();
		tmp.push_back(transformedFinalPosition.translation().x());
		tmp.push_back(transformedFinalPosition.translation().y());
		tmp.push_back(transformedFinalPosition.translation().z());
		tmp.push_back(transformedFinalPosition.rotation().w);
		tmp.push_back(transformedFinalPosition.rotation().x);
		tmp.push_back(transformedFinalPosition.rotation().y);
		tmp.push_back(transformedFinalPosition.rotation().z);
		filteredGraspList[i]->SetFinalgraspPosition(tmp);
	}
}

void GraspPlanningService::retrieve()
{	
	DBGA("check hand");
	if (!mHand || GraspitDBGrasp::getHandDBName(mHand) != mHandName){
		if(mHand) 
			delete mHand;
		DBGA("To load hand");
		// if the hand name contains a '_', the name of the hand is the first part, the material is the second part
		if(!(mHand = GraspitDBGrasp::loadHandFromDBName(mHandName)))
			DBGA("GraspPlanningService: Invalid hand name: " + mHandName.toStdString());
	}
	// initialization must be done after setting of mHand if we want to retrieve models through this
	//DBMgr later, because the grasp allocator for the DBMgr must know its hand type
	if (!mDBMgr)
		init();
	DBGA("To retrieve model list");
	//load the models from database manager
	if(!mDBMgr->ModelList(&mModelList,db_planner::FilterList::NONE)){
		DBGA("Model list retrieval failed");
		return;
	} else {
		DBGA("Model list retrieval succeeded");
	}
	//find the model
	DBGA("To find the model");
	for(size_t i = 0; i < mModelList.size(); ++i)
	{
		if( !strcmp(mModelList[i]->ModelName().c_str(), mObjectName.latin1()) ) //find the object
		{
			dbmodel = mModelList[i];
		}
	}

	if(!dbmodel)
	{
		std::cout << "object not found" << std::endl;
		return;
	}

	if(!mDBMgr->GetGrasps(*dbmodel,GraspitDBGrasp::getHandDBName(mHand).toStdString(), &mGraspList)){
		DBGA("Load grasps failed");
		mGraspList.clear();
		return;
	}
	std::cout << "grasps retrieved: " << mGraspList.size() << std::endl;
}

QString GraspPlanningService::synthesizeMsg(std::vector<db_planner::Grasp*> grasps, std::vector< std::vector<size_t> > coverageList)
{
	std::vector<int> graspGroupID;
	graspGroupID.resize(grasps.size(), -1);
	for(size_t i = 0; i < coverageList.size(); ++i)
	{
		std::vector<size_t> coverage = coverageList[i];
		for(size_t j = 0; j < coverage.size(); ++j)
		{
			graspGroupID[coverage[j]] = i;
		}
	}

	QString msg, quality;
	//brace for a whole message
	msg = QString("{");

	for(size_t i = 0; i < grasps.size(); ++i) //grasps.size()
	{
		//bracket for a pre-grasp
		msg += QString("[");
		quality.setNum(grasps[i]->EpsilonQuality());
		msg += quality;
		msg += QString(",");
		quality.setNum(grasps[i]->VolumeQuality());
		msg += quality;
		msg += QString(",");
		msg += extractNumbers(grasps[i]->GetPregraspPosition());
		msg += QString(",");
		msg += extractNumbers(grasps[i]->GetPregraspJoints());
		msg += QString(",");
		//fixme -- set grasp type number from database number, FIXED!
		//if(!grasps[i]->GetGraspTypeName().compare("FINGERTIP"))
		//msg += QString::number(0);
		//else
		//msg += QString::number(1);
		std::string source = grasps[i]->GetSource();
		std::cout << "source: " << source.c_str() << std::endl;
		if(!strcmp(source.c_str(), "EIGENGRASPS"))
			msg += QString::number(db_planner::EIGENGRASPS);
		else if(!strcmp(source.c_str(), "HUMAN"))
			msg += QString::number(db_planner::HUMAN);
		else if(!strcmp(source.c_str(), "HUMAN_REFINED"))
			msg += QString::number(db_planner::HUMAN_REFINED);
		else if(!strcmp(source.c_str(), "OBJECT_RECOGNITION"))
			msg += QString::number(db_planner::OBJECT_RECOGNITION);
		else if(!strcmp(source.c_str(), "EIGENGRASPS_TASK_1"))
			msg += QString::number(db_planner::EIGENGRASPS_TASK_1);
		else if(!strcmp(source.c_str(), "EIGENGRASPS_TASK_2"))
			msg += QString::number(db_planner::EIGENGRASPS_TASK_2);
		else if(!strcmp(source.c_str(), "TABLETOP_ALIGNED"))
			msg += QString::number(db_planner::TABLETOP_ALIGNED);
		else
		{
			std::cout << "WARNING: No source is set or retrieved, set to HUMAN_REFINED as default" << std::endl;
			msg += QString::number(db_planner::HUMAN_REFINED);
		}
		//adding the grasp group to specify the coverage of this grasp
		msg += QString(",");
		msg += QString::number(graspGroupID[i]);

		//adding the grasp type to the message
		msg += ",";
		std::string type = grasps[i]->GetGraspTypeName();
		if(!strcmp(type.c_str(), "UNKNOWN"))
			msg += QString::number(db_planner::UNKNOWN_GRASP_TYPE);
		else if(!strcmp(type.c_str(), "FINGERTIP"))
			msg += QString::number(db_planner::FINGERTIP_GRASP);
		else if(!strcmp(type.c_str(), "POWER"))
			msg += QString::number(db_planner::POWER_GRASP);
		else if(!strcmp(type.c_str(), "TABLECONTACT"))
			msg += QString::number(db_planner::TABLECONTACT_GRASP);
		else
		{
			std::cout << "WARNING: No grasp type is set or retrieved, set to UNKNOWN" << std::endl;
			msg += QString::number(db_planner::UNKNOWN_GRASP_TYPE);
		}
		msg += QString("]");

		//bracket for a fin-grasp
		msg += QString("[");
		quality.setNum(grasps[i]->EpsilonQuality());
		msg += quality;
		msg += QString(",");
		quality.setNum(grasps[i]->VolumeQuality());
		msg += quality;
		msg += QString(",");

		msg += extractNumbers(grasps[i]->GetFinalgraspPosition());
		msg += QString(",");
		msg += extractNumbers(grasps[i]->GetFinalgraspJoints());
		msg += QString("]");
	}
	msg += QString("}");

	return msg;
}

QString GraspPlanningService::extractNumbers(std::vector<double> numArray)
{
	QString msg, tmp;
	//msg = QString("(");
	for(size_t i = 0; i < numArray.size(); ++i)
	{
		tmp.setNum(numArray[i]);
		msg += tmp;
		msg += (i == numArray.size() - 1) ? QString("") : QString(",");
	}
	return msg;
}



class FingertipGraspAnalyzer{
public:
	Body * table;
	FingertipGraspAnalyzer(Body *t):table(t){}

	double operator()(GraspPlanningState * fingertipGrasp){
		vec3 tableNegativeZ = -vec3::Z*table->getTran();
		vec3 handZApproach = vec3::Z*fingertipGrasp->getHand()->getApproachTran()*fingertipGrasp->getTotalTran();
		vec3 handYApproach = (vec3::Y*fingertipGrasp->getHand()->getApproachTran())* fingertipGrasp->getTotalTran();
		double zAngle = fabs(acos(tableNegativeZ%handZApproach));
		double yAngle = fabs(fabs(acos(tableNegativeZ%handYApproach)) - M_PI/2);
		double energy = zAngle+yAngle;
		if(fabs(zAngle) > .29) /*20 degrees*/
			energy += 100;
		if(fabs(yAngle) > .14) /*10 degrees*/
			energy += 1000;
		//std::cout << energy << std::endl;
		fingertipGrasp->setEnergy(energy);
		return energy;
	}
};


bool FingertipGraspComp(GraspPlanningState * s1, GraspPlanningState *s2){
	return s1->getEnergy() < s2->getEnergy();
}


bool FingertipGraspAdjust(GraspPlanningState * Grasp, transf & t){
	Grasp->getHand()->setTran(t);	    
	Grasp->getHand()->findInitialContact(.1);
	Grasp->saveCurrentHandState();
	return true;
}

bool AdjustFingertipGraspToTargetVector(GraspPlanningState * fingertipGrasp, vec3 & targetVector){
	vec3 handZApproach = vec3::Z*fingertipGrasp->getHand()->getApproachTran()*fingertipGrasp->getTotalTran();  
	double zAngle = -acos(targetVector%handZApproach); 
	vec3 rotationAxis = normalise(targetVector*handZApproach);
	transf rotTran = rotate_transf(zAngle, rotationAxis);
	transf t(fingertipGrasp->getTotalTran().affine() * rotTran.affine(), fingertipGrasp->getTotalTran().translation());
	return FingertipGraspAdjust(fingertipGrasp, t);
}


bool AdjustFingertipGraspToTableZAxis(GraspPlanningState * fingertipGrasp, Body * table){
	vec3 tableNegativeZ = -vec3::Z*table->getTran();  
	return AdjustFingertipGraspToTargetVector(fingertipGrasp, tableNegativeZ);
}

bool AdjustFingertipGraspToCenterOfContacts(GraspPlanningState * Grasp, Body * table, Body * mObject){
	graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, table, mObject);
	graspItGUI->getIVmgr()->getWorld()->toggleCollisions(false, Grasp->getHand(), table);
	Grasp->execute();
	std::list<Contact*> clist = Grasp->getHand()->getContacts();
	position contactCenter(0.0, 0.0, 0.0);
	int contactNum = 0;
	for (std::list<Contact *>::iterator citer = clist.begin(); citer != clist.end(); ++citer){
		contactCenter = contactCenter + (*citer)->getMate()->getPosition();
		contactNum ++;
	}
	if(!contactNum)
		return false;
	vec3 handZApproach = vec3::Z*Grasp->getHand()->getApproachTran()*Grasp->getTotalTran();  
	vec3 tableNegativeZ = -vec3::Z*table->getTran();  
	double zAngle = -acos(tableNegativeZ%handZApproach); 
	vec3 rotationAxis = normalise(tableNegativeZ*handZApproach);
	transf rotTran = rotate_transf(zAngle, rotationAxis);
	transf t(Grasp->getTotalTran().affine() * rotTran.affine(), vec3(contactCenter[0],contactCenter[1],contactCenter[2])/contactNum);     
	graspItGUI->getIVmgr()->getWorld()->toggleCollisions(false, table, mObject);
	graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, Grasp->getHand(), table);
	std::cout <<"***GraspPlannningService::FingertipGrasp::No nearby grasps found, new grasp synthesized " << std::endl;
	return FingertipGraspAdjust(Grasp, t);
}







/*
pre-grasp and final-grasp "in our database" should have the following assumptions:
------they only differ in their joint angles------
this does not apply for the pre-grasps output from this service to ros

for every possible symmetric rotation:
test the finalgrasp pose, 
if the hand is close and towards the table,
mark it as a tabletopcontact, 
and put it into fingertip candidate list
generate pregrasp pose from finalgrasp pose by moving back 60mm
if collision,
ignore the grasp
else
add this grasp to a regular list

if regular list is not empty
return it
else
test fingertip candidates
if no candidate survived
adjust the best fingertip candidate
*/
void GraspPlanningService::check(const transf & table_pose){
	filteredGraspList.clear();
	if (mGraspList.size()==0){
		std::cout << "******************************************************" << std::endl;
		std::cout << "GraspPlanningService::FATAL ERROR: 0 grasps retrieved!" <<std::endl;
		return;
	}
	//load table
	Body * table = graspItGUI->getIVmgr()->getWorld()->importBody("Body", QString(getenv("GRASPIT"))+ QString("/models/obstacles/zeroplane.xml"));
	table->setTran(table_pose /** transf(Quaternion(.7071, .7071 , 0, 0), vec3(0,0,0))*/);
	//load the object
	GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(dbmodel);
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
			DBGA("Model load failed");
			return;
		}
	}
	GraspableBody * mObject = model->getGraspableBody();
	graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());
	//adds the object to the collision detection system
	mObject->addToIvc();
	mObject->setTran(mObjectPose);
	graspItGUI->getIVmgr()->getWorld()->toggleCollisions(false, table, model->getGraspableBody());

	int left = filterGraspByCurrentObjectTablePose(table, mObject);
	DBGA("after pose filtering there are: " << left << " grasps");
	//create search grasp analyzer
	//If SCALING OF OBJECT IS DESIRED THIS IS THE RIGHT PLACE
	double symmetryAngleModifier = 1.0;
	double symmetryAngle = 0.0;
	std::vector<GraspPlanningState*> fingertipGraspCandidates;

	symmetryAngle = model->SymmetryAxis()[3];
	do{
		std::cout << "testing with symmetry angle = " << symmetryAngle << std::endl;

		for( unsigned int j =0; j < mGraspList.size(); ++j)
		{

			/*these objects are symmetric around one axis-- rotate the grasp every x degrees and try again, the step is define in CGDB */
			for (double angle = 0; angle < 2.0*M_PI; angle += symmetryAngle)
			{
				GraspPlanningState pregraspState(static_cast<GraspitDBGrasp*>(mGraspList[j])->getPreGraspPlanningState()), finalgraspState(static_cast<GraspitDBGrasp*>(mGraspList[j])->getFinalGraspPlanningState());
				pregraspState.setObject(mObject);
				pregraspState.setRefTran(mObjectPose);
				pregraspState.getPosition()->setTran(pregraspState.getPosition()->getCoreTran() *rotate_transf(angle, vec3 (model->SymmetryAxis()[0],model->SymmetryAxis()[1],model->SymmetryAxis()[2])));

				finalgraspState.setObject(mObject);
				finalgraspState.setRefTran(mObjectPose);
				finalgraspState.getPosition()->setTran(finalgraspState.getPosition()->getCoreTran() *rotate_transf(angle, vec3 (model->SymmetryAxis()[0],model->SymmetryAxis()[1],model->SymmetryAxis()[2])));

				vec3 approachVec = (vec3::Z*mHand->getApproachTran())* finalgraspState.getTotalTran();

				finalgraspState.execute();     
				graspItGUI->getIVmgr()->getWorld()->toggleCollisions(false, mHand, model->getGraspableBody());
				//in this examination, the joints could vary in different grasps
				double distance =  mHand->getApproachDistance(table, 200);
				bool table_close = distance < 140;
				std::cout << "distance from the table: " << distance << std::endl;
				bool approaching_table_from_above = approachVec % (vec3::Z*table->getTran()) < 0;
				std::cout << "angle: " <<  approachVec % (vec3::Z*table->getTran()) << std::endl;
				if(table_close)
				{
					if(!approaching_table_from_above)
					{
						//DBGA("Hand not approaching the table from above although it is close to it");
					}
					else
					{
						//DBGA("Hand too close to table -- consider table top contact ");
					}
					mGraspList[j]->SetGraspTypeName("TABLECONTACT");
				}

				if( (table_close && approaching_table_from_above) )//  || (mGraspConstraints & FINGERTIP_ALIGNED_ONLY) )
				{
					if(table_close && approaching_table_from_above) //used to check the collision with the table, not needed: !graspItGUI->getIVmgr()->getWorld()->noCollision(mHand)
					{
						//DBGA("Adding one fingertip grasp candidate: case 1");
					}
					else
					{
						//DBGA("Adding one fingertip grasp candidate: case 2");
					}

					fingertipGraspCandidates.push_back(new GraspPlanningState(&finalgraspState));
					//set the index for later we retrieve the correspondence after we re-rank the fingertipGraspCandidates list
					//std::cout << "index set: " << j << ", finger1 joint: " << mGraspList[j]->GetPregraspJoints()[1] << std::endl;
					fingertipGraspCandidates[ fingertipGraspCandidates.size() - 1 ]->setIndex(j); // j is the index in mGraspList
					continue;
				}

				//move the pregrasp position back
				double backOffDist = -BACKUP_DIST;
				pregraspState.getPosition()->setTran(translate_transf(vec3(0,0,backOffDist))*finalgraspState.getPosition()->getCoreTran());
				pregraspState.execute();
				//graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, mHand, model->getGraspableBody());
				graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, mHand, NULL);

				std::cout << mHand->getTran().translation().x() << ", " << 
					mHand->getTran().translation().y() << ", " << 
					mHand->getTran().translation().z() << std::endl;
				std::cout << mHand->getTran().rotation().x << ", " << 
					mHand->getTran().rotation().y << ", " << 
					mHand->getTran().rotation().z << std::endl;
				std::cout << table->getTran().translation().x() << ", " <<
					table->getTran().translation().y() << ", " <<
					table->getTran().translation().z() << std::endl;

				if(!graspItGUI->getIVmgr()->getWorld()->noCollision(mHand))
				{
					std::cout << "COL1: collision in pregrasp for grasp: " << j << std::endl;
					continue;
				}
				else
				{
					std::cout << "COL1: no" << std::endl;
				}

				GraspitDBGrasp * ng = new GraspitDBGrasp(*static_cast<GraspitDBGrasp*>(mGraspList[j]));
				ng->setPreGraspPlanningState(new GraspPlanningState(&pregraspState));
				ng->setFinalGraspPlanningState(new GraspPlanningState(&finalgraspState));
				ng->SetGraspTypeName( mGraspList[j]->GetGraspTypeName() );
				std::cout << "A: added a grasp from original grasp: " << j << ", with type: " << ng->GetGraspTypeName() << ", " <<  ng->GetPregraspJoints()[1] << std::endl;

				/*shall we rank them by the closeness to an axis?*/
				double tmp = fabs(approachVec.x()) > fabs(approachVec.y()) ? fabs(approachVec.x()) : fabs(approachVec.y());
				tmp = tmp > fabs(approachVec.z()) ? tmp : fabs(approachVec.z());
				ng->SetEpsilonQuality(tmp);

				filteredGraspList.push_back(ng);	  

			}// loop on all the symmetric angles
		}// loop on all the grasps in grasp list

		std::cout << "before fingertip grasp candidates are tested, there are " << filteredGraspList.size() << " grasps found in regular list" << std::endl;
		std::cout << "there are " << fingertipGraspCandidates.size() << " fingertip grasp candidates to be tested" << std::endl;
		//now try fingertip grasp candidates for this level of symmetry
		DBGA("A");
		//step 1 - rank them by fingertip grasp energy
		std::for_each(fingertipGraspCandidates.begin(), fingertipGraspCandidates.end(),FingertipGraspAnalyzer(table));
		std::sort(fingertipGraspCandidates.begin(),fingertipGraspCandidates.end(),FingertipGraspComp);
		DBGA("B");
		//step 2 - parse successful candidates
		if(fingertipGraspCandidates.size() > 0)
			std::cout << "energy: " << fingertipGraspCandidates[0]->getEnergy() << std::endl;
		for(std::vector<GraspPlanningState*>::iterator fg= fingertipGraspCandidates.begin(); fg != fingertipGraspCandidates.end() && ((*fg)->getEnergy() < 100); ++fg){
			std::cout << "index is: " << fg - fingertipGraspCandidates.begin() << std::endl;

			//modify fg to align to z axis
			if (!AdjustFingertipGraspToTableZAxis((*fg), table))
				continue;

			//Enable all collisions
			graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, mHand, model->getGraspableBody());
			graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, mHand, table);

			//Create Pregrasp state
			GraspPlanningState pregrasp((*fg));
			pregrasp.getPosition()->setTran(translate_transf(vec3(0,0,-BACKUP_DIST))*pregrasp.getPosition()->getCoreTran());
			pregrasp.execute();
			//will open hand -- if pregrasp is in collision at this stage, this grasp is invalid.
			//quickOpen wil return false in this case
			if(!pregrasp.getHand()->quickOpen(1.0))
				continue;
			pregrasp.saveCurrentHandState();
			//Create new Graspit_DB_Grasp
			GraspitDBGrasp * ng = new GraspitDBGrasp((*fg)->getHand());
			ng->SetSource("TABLETOP_ALIGNED");
			ng->SetGraspTypeName("TABLECONTACT");
			ng->SetSource("HUMAN_REFINED");
			ng->setPreGraspPlanningState(new GraspPlanningState(&pregrasp));
			ng->setFinalGraspPlanningState(new GraspPlanningState((*fg)));
			ng->SetPregraspJoints(mGraspList[(*fg)->getIndex()]->GetPregraspJoints());
			ng->SetFinalgraspJoints(mGraspList[(*fg)->getIndex()]->GetFinalgraspJoints());
			std::cout << "B: added new grasp from original index: " << (*fg)->getIndex() << ", " << ng->GetPregraspJoints()[1] << std::endl;

			filteredGraspList.push_back(ng);
		}
		std::cout << "filteredGraspList.size: " << filteredGraspList.size() << " symmetryAngle: " << symmetryAngle*180/M_PI << " degrees" << std::endl;

		if(symmetryAngle < M_PI/60  || symmetryAngle >= M_PI)
		{
			std::cout << "getting out of while loop, symmetryAngle is: " << symmetryAngle << std::endl;
			break;
		}
		else
		{
			symmetryAngleModifier*=2.0;
			symmetryAngle /= symmetryAngleModifier;
		}
	}while(filteredGraspList.size() == 0);

	std::cout << "Now filteredGraspList.size: " << filteredGraspList.size() << std::endl;


	/*Parse the list of failed, potential table aligned grasps and attempt 
	to create a new grasp that aligns with the Z axis of the table and centers the hand
	over the contacts of the planned grasp 
	If there are grasps that can be adjusted to be aligned to table z-axis, this could
	happend above and what's after this point should not be achieved.
	If there are no such grasps, we will go through the following steps
	*/
	if(filteredGraspList.size() == 0){

		//Neither symmetries not adjusting fingertip grasps was able to produce a valid grasp.  Take the best grasp
		//from the fingertip candidate set and force to be perpendicular to the table and around the center of contacts
		//return this grasp
		for(std::vector<GraspPlanningState*>::iterator fg = fingertipGraspCandidates.begin(); fg != fingertipGraspCandidates.end(); ++fg) {
			AdjustFingertipGraspToCenterOfContacts(*fg, table, mObject);
			//Enable all collisions
			graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, mHand, model->getGraspableBody());
			graspItGUI->getIVmgr()->getWorld()->toggleCollisions(true, mHand, table);

			//Create Pregrasp state
			GraspPlanningState pregrasp((*fg));
			pregrasp.getPosition()->setTran(translate_transf(vec3(0,0,-BACKUP_DIST))*pregrasp.getPosition()->getCoreTran());
			//will open hand -- if pregrasp is in collision at this stage, this grasp is invalid.
			//quickOpen wil return false in this case
			pregrasp.execute();
			if(!pregrasp.getHand()->quickOpen(1.0))
				continue;
			pregrasp.saveCurrentHandState();
			//Create new Graspit_DB_Grasp
			GraspitDBGrasp * ng = new GraspitDBGrasp(pregrasp.getHand());
			ng->SetSource("TABLETOP_ALIGNED");
			ng->SetGraspTypeName("TABLECONTACT");
			ng->setPreGraspPlanningState(new GraspPlanningState(&pregrasp));
			ng->setFinalGraspPlanningState(new GraspPlanningState((*fg)));
			ng->SetPregraspJoints(mGraspList[(*fg)->getIndex()]->GetPregraspJoints());
			ng->SetFinalgraspJoints(mGraspList[(*fg)->getIndex()]->GetFinalgraspJoints());
			std::cout << "C: added new grasp from original index: " << (*fg)->getIndex() << ", " <<  ng->GetPregraspJoints()[1] << std::endl;

			filteredGraspList.push_back(ng);
			break;
		}
	}

	graspItGUI->getIVmgr()->getWorld()->destroyElement(table, true);	
	graspItGUI->getIVmgr()->getWorld()->destroyElement(mObject, false);	
	table = NULL;

}

void GraspPlanningService::rank()
{
	std::vector<db_planner::Grasp*>::iterator first, last;
	first = filteredGraspList.begin();
	last = filteredGraspList.end();
	sort(first, last, compareGraspQM);
}

void GraspPlanningService::plan_from_tasks(){
	/*Tasks know their own objects and hands
	In this case, we reuse the object parameter as the task type
	*/
	if (!mDBMgr)
		init();
	if(!mDBMgr->isConnected()){
		//for now this error is fatal, quit.
		DBGA("GraspPlanningService: Fatal error.  Not able to connect to CGDB");
		exit(1);
	}else{
		DBGA("Grasp planning service was able to connect");
	}

	vector <string> taskNameList;
	mDBMgr->TaskTypeList(&taskNameList);
	//find task id - a QStringList would have this function built in.
	int task_type_id = -1;
	for(unsigned int titer = 0; titer < taskNameList.size(); ++titer){
		if(!mObjectName.toStdString().compare(taskNameList[titer])){
			task_type_id	= static_cast<signed int>(titer)+1; //task_type_id is 1 indexed
			break;
		}
	}
	if (task_type_id < 0){
		DBGA("Couldn't find task name");
		exit(1);
		//return;
	}
	TaskFactory * tf(NULL);
	//make the appropriate type of task factory for the task

	switch(task_type_id){
	case 1:{

		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::AlignedYParallelToVEgPlannerFactory, egPlannerUtils::SaveGuidedPlannerTaskToDatabase, 1>(mDBMgr);
		break;
		   }
	case 2:{
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::LooselyAlignedPlannerFactory<egPlannerUtils::AlignedYParallelToVEgPlannerFactory, 80>, egPlannerUtils::SaveGuidedPlannerTaskToDatabase, 2>(mDBMgr);
		break;
		   }
	case 33:
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::SimulatedAnnealingPlannerFactory, egPlannerUtils::SaveSimulatedAnnealingTaskToDatabase, 33>(mDBMgr);
		break;
	case 32:
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::SimulatedAnnealingPlannerFactory, egPlannerUtils::SaveSimulatedAnnealingTaskToDatabase, 32>(mDBMgr);
		break;
	case 31:
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::SimulatedAnnealingPlannerFactory, egPlannerUtils::SaveSimulatedAnnealingTaskToDatabase, 31>(mDBMgr);
		break;
	case 30:
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::SimulatedAnnealingPlannerFactory, egPlannerUtils::SaveSimulatedAnnealingTaskToDatabase, 30>(mDBMgr);
		break;
	case 29:{
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::SimulatedAnnealingPlannerFactory, egPlannerUtils::SaveSimulatedAnnealingTaskToDatabase, 29>(mDBMgr);
		break;
			}
	case 28:{
		tf = new GenericGraspPlanningTaskFactory<egPlannerUtils::GuidedPlannerFactory, egPlannerUtils::SaveGuidedPlannerTaskToDatabase, 28>(mDBMgr);
		break;
			}
	case 37:{
		//for update the tactile contact list for each grasp
		tf = new UpdateTactileContactTaskFactory(mDBMgr);
		break;
			}
	case 38:{
		//for testing TEB grasping
		tf = new BlindGraspingTestTaskFactory(mDBMgr);
		break;
			}
	case 39:{
		tf = new BlindGraspingPrecomputePerturbationTaskFactory(mDBMgr);
		break;
			}
	default:{
		//tf = new GraspAnalyzerFactory();
		DBGA("task id not matched");
		exit(1);
			}
	}

	TaskDispatcher * td = new TaskDispatcher(tf, task_type_id);
	if(td->connect(mdbaseURL.toStdString(),
		mdbasePort,
		mdbaseUserName.toStdString(),
		mdbasePassword.toStdString(),
		mdbaseName.toStdString())){
			//connect returns 0 on success - if this failed, print out that
			//it failed and exit with an exit code
			//FIXME exit codes
			DBGA("GraspPlanningService: Fatal error.  Task Dispatcher not able to connect to CGDB");
			exit(1);
	}
	/*otherwise, acknowledge that we have connected and started*/
	DBGA("Connected to task dispatching database - starting dispatcher");

	//start() returns right away, it is not blocking
	td->start();

	//for now this just leaks and runs until terminating the program

}



/*
This function plans a list of grasps that will together cover all the pose uncertainties
mObjectPose stores the most likely pose we perceived
the six input parameters specify the uncertainties in each of the six transformation domains
how we are combining them is still not determined at current time
*/
void GraspPlanningService::planWithUncertainties(double xUncertainty, double yUncertainty, double zUncertainty, double rxUncertainty, double ryUncertainty, double rzUncertainty)
{
	std::cout << "UNFINISHED" << std::endl;
}

void GraspPlanningService::planWithUncertainties(const std::vector<transf>& poseUncertainty)
{
	std::vector<db_planner::Grasp*> graspList = filteredGraspList;
	if (graspList.size()==0){
		std::cout << "******************************************************" << std::endl;
		std::cout << "GraspPlanningService::FATAL ERROR: 0 grasps retrieved!" <<std::endl;
		return;
	}
	std::cout << "Pose Uncertainty: \n";
	for(size_t i = 0; i < poseUncertainty.size(); ++i)
	{
		std::cout << poseUncertainty[i].translation().x() << " " 
			<< poseUncertainty[i].translation().y() << " " 
			<< poseUncertainty[i].translation().z() << " " 
			<< poseUncertainty[i].rotation().w << " "
			<< poseUncertainty[i].rotation().x << " "
			<< poseUncertainty[i].rotation().y << " "
			<< poseUncertainty[i].rotation().z << std::endl;
	}


	//load table
	//  Body * table = graspItGUI->getIVmgr()->getWorld()->importBody("Body", QString(getenv("GRASPIT"))+ QString("/models/obstacles/zeroplane.xml"));
	//table->setTran(table_pose /** transf(Quaternion(.7071, .7071 , 0, 0), vec3(0,0,0))*/);

	//load the object
	GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(dbmodel);
	//check that this model is already loaded into Graspit, if not, load it
	if (!model->geometryLoaded()) {
		//this loads the actual geometry in the scene graph of the object
		if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
			DBGA("Model load failed");
			return;
		}
	}
	GraspableBody * mObject = model->getGraspableBody();
	graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());
	//adds the object to the collision detection system
	mObject->addToIvc();
	//mObject->setTran(mObjectPose);
	//graspItGUI->getIVmgr()->getWorld()->toggleCollisions(false, table, model->getGraspableBody());

	//generate uncertain pose list based on the input uncertain specifiers
	std::vector<transf> uncertainPoses = poseUncertainty;
	// uncertainPoses.push_back(transf(Quaternion(M_PI/6.0,vec3(1,0,0)),vec3(0,0,0)));
	// uncertainPoses.push_back(transf(Quaternion(-M_PI/6.0,vec3(1,0,0)),vec3(0,0,0)));
	// uncertainPoses.push_back(transf(Quaternion(M_PI/4.0,vec3(1,0,0)),vec3(0,0,0)));
	// uncertainPoses.push_back(transf(Quaternion(-M_PI/4.0,vec3(1,0,0)),vec3(0,0,0)));
	//setup the quality computing facility
	World *w = graspItGUI->getIVmgr()->getWorld();
	QualEpsilon *qualEpsilon = NULL;
	qualEpsilon = new QualEpsilon(w->getCurrentHand()->getGrasp(), QString("Grasp_planner_qm"), "L1 Norm");

	//define the matrix
	std::vector< std::vector<int> > coverCapabilityMatrix;
	//begin to plan
	std::cout << "begin to plan with uncertainties for " << graspList.size() << " grasps on " << uncertainPoses.size() << " poses" << std::endl;

	//run the grasp test for uncertain poses
	for(size_t i = 0; i < graspList.size(); ++i)
	{
		GraspitDBGrasp g(*static_cast<GraspitDBGrasp*>(graspList[i]));
		//initialize the coverage vector
		std::vector<int> m_g;

		for(size_t j = 0; j < uncertainPoses.size(); ++j)
		{
			transf p = uncertainPoses[j];

			//put the object in the right place
			mObject->setTran(p);

			//get the current grasp
			//move the hand back by 60mm to form the pre-grasp
			GraspPlanningState pregraspState(g.getPreGraspPlanningState()), finalgraspState(g.getFinalGraspPlanningState());

			transf backup(Quaternion::IDENTITY, vec3(0.0,0.0,-100.0));
			pregraspState.setObject(mObject);
			pregraspState.setRefTran(p.inverse() * p);
			finalgraspState.setObject(mObject);
			finalgraspState.setRefTran(p.inverse() * p);

			pregraspState.getPosition()->setTran(backup * finalgraspState.getPosition()->getCoreTran());
			//shape the grasp to the pre-grasp
			pregraspState.execute();
			//	  return;

			//move until contact with the object
			if(!w->noCollision())
			{
				std::cout << "collision in prgrasp for grasp: " << j << std::endl;
				continue;
			}
			if(!w->getCurrentHand()->approachToContact(200.0,false))
			{
				std::cout << "failed to approach to contact" << std::endl;
				continue;
			}
			//std::cout << "ready for stability test" << std::endl;

			//auto grasp, do not render is we are using console
			w->getCurrentHand()->autoGrasp(!graspItGUI->useConsole());

			//compute the epsilon quality
			w->updateGrasps();
			w->findAllContacts();
			double qual = qualEpsilon->evaluate();
			if(qualEpsilon->evaluate() > 0.0)
			{
				//store the matrix
				m_g.push_back(j);
				std::cout << "A stable grasp " << i << " at pose " << j << " with quality: " << qual << std::endl;
			}
			else
			{
				std::cout << "An unstable grasp " << i << " at pose " << j << std::endl;
			}
		}
		coverCapabilityMatrix.push_back(m_g);
	}
	//clean up
	delete qualEpsilon;
	if(graspItGUI->useConsole())
		graspItGUI->getIVmgr()->getWorld()->destroyElement(mObject, false);	

	//synthesize the solutionList
	std::vector<size_t> solution, originalIndex;
	std::vector< std::vector<size_t> > solutionList;
	bool success = false;
	for(size_t i = 0; i < coverCapabilityMatrix.size(); ++i)
		originalIndex.push_back(i);

	/* Now we compute *all* the possible solutions
	well, not really all of them, but all in this run
	*/
	while(1)
	{
		bool currentSuccess = minimumCover(coverCapabilityMatrix, uncertainPoses.size()-1, solution);
		success = success || currentSuccess;
		if(!currentSuccess)
		{
			break;
		}
		/* one valid solution is found */

		//synthesize from originalIndex
		//solution is the index in coverCapabilityMatrix, but the matrix is changed every iteration, so, we need to keep track of the original index and put it into the solutionIndex
		std::vector<size_t> solutionIndex;
		for(size_t i = 0; i < solution.size(); ++i)
		{
			solutionIndex.push_back(originalIndex[solution[i]]);
		}
		//push back the current solution to solution list
		solutionList.push_back(solutionIndex);
		//erase the items in the solution from the coverCapabilityMatrix and the originalIndex
		std::vector<size_t> tmpIndex;
		std::vector< std::vector<int> > tmpCoverCapabilityMatrix;
		for(size_t i = 0; i < originalIndex.size(); ++i)
		{
			if( find(solutionIndex.begin(), solutionIndex.end(), originalIndex[i]) == solutionIndex.end() )
			{
				tmpIndex.push_back(originalIndex[i]);
				tmpCoverCapabilityMatrix.push_back(coverCapabilityMatrix[i]);
			}
		}
		originalIndex = tmpIndex;
		coverCapabilityMatrix = tmpCoverCapabilityMatrix;
		solution.clear();
	}

	if(success)
	{
		std::cout << "Found " << solutionList.size() << "solutions" << std::endl;
		mGraspSetCoverageList = solutionList;
		for(size_t i = 0; i < solutionList.size(); ++i)
		{
			std::cout << "solution " << i << ": ";
			solution = solutionList[i];
			for(size_t i = 0; i < solution.size(); ++i)
			{
				std::cout << solution[i] << " ";
			}
			std::cout << std::endl;	  
		}
		if(!graspItGUI->useConsole())
		{
			//GraspViewerDlg *dlg = new GraspViewerDlg(graspList, mObject, uncertainPoses);
			//dlg->setAttribute(Qt::WA_ShowModal, false);
			//dlg->setAttribute(Qt::WA_DeleteOnClose, true);
			//dlg->show();
		}
	}
	else
	{
		std::cout << "Cannot find a finite set of grasps based on the original set" << std::endl;
	}
}

struct CoverRecord
{
	size_t index;
	int numToBeCovered;
};

bool CoverRecordSmallerThan (CoverRecord cl, CoverRecord cr)
{
	return cl.numToBeCovered < cr.numToBeCovered;
}

bool GraspPlanningService::minimumCover(std::vector< std::vector<int> > coverList, int range, std::vector<size_t> & solution)
{
	// std::cout << "The cover list to solve" << std::endl;
	// for(int i = 0; i < coverList.size(); ++i)
	//   {
	//     for(int j = 0; j < coverList[i].size(); ++j)
	//{
	//  std::cout << coverList[i][j] << " ";
	//}
	//     std::cout << std::endl;
	//   }
	// std::cout << std::endl;

	//the current union of the sets during iteration
	std::vector<int> currentUnion;

	for(int k = 0; k < coverList.size(); ++k)//still not covered
	{
		currentUnion.resize(range+2, range+2);
		//initialization for the current iteration
		std::vector<CoverRecord> difference;
		difference.resize(coverList.size());

		//compute the difference
		for(size_t i = 0; i < coverList.size(); ++i)
		{
			std::vector<int> current = coverList[i];
			sort(current.begin(), current.end());
			sort(currentUnion.begin(), currentUnion.end());
			// we want the last item is range+1 used as a terminator
			std::vector<int> differenceResult;
			differenceResult.resize(range+2, range+2);
			set_difference(current.begin(), current.end(), currentUnion.begin(), currentUnion.end(), differenceResult.begin());
			int s = find(differenceResult.begin(), differenceResult.end(), range+2) - differenceResult.begin();

			difference[i].index = i;
			difference[i].numToBeCovered = s;
		}

		//sort by the size of the intersection
		sort(difference.begin(), difference.end(), CoverRecordSmallerThan);
		reverse(difference.begin(), difference.end());

		//union with the first one
		std::vector<int> tmpUnion;
		tmpUnion.resize((range+2) * 2, range+2);
		solution.push_back(difference[0].index);
		set_union(coverList[difference[0].index].begin(), coverList[difference[0].index].end(), currentUnion.begin(), currentUnion.end(), tmpUnion.begin());
		currentUnion = tmpUnion;
		currentUnion.erase(find(currentUnion.begin(), currentUnion.end(), range+2), currentUnion.end());

		if(currentUnion.size() == range + 1)
			return true;
	}
	return false;
}

int GraspPlanningService::filterGraspByCurrentObjectTablePose(Body* table, GraspableBody* model)
{
	std::vector<db_planner::Pose> poseList;
	if(!mDBMgr->GetObjectPoseList(&poseList)){
		DBGA("Pose list retrieval failed");
		return 0;
	} else {
		DBGA("Pose list retrieval succeeded");
	}
	std::cout << poseList.size() << " poses retrieved " << std::endl;

	for(std::vector<db_planner::Grasp*>::iterator it = mGraspList.begin(); it != mGraspList.end(); )
	{

		//the poses this grasp is good for
		std::vector<int> validGraspPoseIdList = (*it)->GetObjectPoseIdList();
		DBGA("grasp " << (*it)->GraspId() << " has " << validGraspPoseIdList.size() << " valid poase");
		if( validGraspPoseIdList[0] == 1)
		{
			DBGA("grasp: " << (*it)->GraspId() << " is expected to work with any type");
			it++;
			continue;
		}

		bool hasOnePoseMatch = false;
		for(size_t pIdx = 0; pIdx < validGraspPoseIdList.size(); ++pIdx)
		{
			int poseId = validGraspPoseIdList[pIdx];

			//look for the correct pose criterion to check against
			size_t i;
			for(i = 0; i < poseList.size(); ++i)
			{
				if(poseList[i].getId() != poseId)
					continue;
				else
					break;
			}

			if(i == poseList.size())
			{
				DBGA("No such pose ID defined: " << poseId);
				it ++;
				continue;
			}

			//now we find the correct the pose and get its criterion to check against

			std::vector<double> oa, ta;
			double lb, ub;

			oa = poseList[i].getObjectAxis();
			ta = poseList[i].getSupportingSurfaceAxis();
			vec3 objectAxis = vec3(oa[0], oa[1], oa[2]);
			vec3 tableAxis = vec3(ta[0], ta[1], ta[2]);
			lb = poseList[i].getLowerBound();
			ub = poseList[i].getUpperBound();

			double angle = acos( (objectAxis * model->getTran()) % (tableAxis * table->getTran()) );

			if( angle > ub || angle < lb )
			{
				//it = mGraspList.erase(it);
				continue;
			}
			else
			{
				DBGA("valid grasp: " << (*it)->GraspId() << " with matching pose: " << poseList[i].getPoseName());
				hasOnePoseMatch = true;
				break;
				//it++;
				//continue;
			}
		}//end of looping through all the acceptable poses

		if(!hasOnePoseMatch)
		{
			DBGA("deleting grasp: " << (*it)->GraspId() << " due to not matching pose any valid poses");
			it = mGraspList.erase(it);
		}
		else
		{
			++it;
		}
	}
	return mGraspList.size();
}
