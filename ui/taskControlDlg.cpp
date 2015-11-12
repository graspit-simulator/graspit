#include "taskControlDlg.h"
#include <QLineEdit>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "world.h"
#include "SemanticPlanner/SemanticMap.h"
#include "BlindPlanner/blindPlannerUtil.h"

#include "SensorInterface.h"
#include "quality.h"
#include "dof.h"

int TaskControlDlg::mExpId = 0;
int id_i, id_j;
SemanticMap sm;

void TaskControlDlg::writeOutGEDFromWorldFiles()
{
	//load all the world files in the directory and store their tactile experience and handpose
	std::vector<std::string> filenames;
	getFileNames(filenames, "C:/project/graspit_hao/worlds/simple/", "");
	char path[128];
	for(int i = 0; i < filenames.size(); ++i)
	{
		graspItGUI->getIVmgr()->emptyWorld();
		sprintf(path, "C:/project/graspit_hao/worlds/simple/%s", filenames[i].c_str());
		graspItGUI->getIVmgr()->getWorld()->load(path);
		//-1 is the index of this grasp, we use -1 for a dummy index
		char finalname[128];
		sprintf(finalname, "%s_-1", filenames[i].c_str());
		writeOutAsTactileExperience("",finalname);
	}
	return;
}

void TaskControlDlg::goButton_clicked()
{
	writeOutGEDFromWorldFiles();
	return;
}

void TaskControlDlg::writeOutExpResults()
{
	World* world = graspItGUI->getIVmgr()->getWorld();
	FILE *flog_joint, *flog_dof, *flog_sensor, *flog_handpose, *flog_tactile, *flog_quality;
	//for the number characters
	char num[30];

	world->updateGrasps();

#define QUALITY_OUTPUT
#ifdef QUALITY_OUTPUT
	flog_quality = fopen("quality.txt", "a");
	fprintf(flog_quality, "%s_%d ", world->getGB(0)->getName().ascii(), mExpId);
	//record the grasp info
	// if there is no collision, then begin computation
	Hand* h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if(mEpsQual){
		delete mEpsQual;
	}
	mEpsQual = new QualEpsilon( h->getGrasp(), "Examine_dlg_qm","L1 Norm");
	if(mVolQual){
		delete mVolQual;
	}
	mVolQual = new QualVolume( h->getGrasp(), "Examine_dlg_qm","L1 Norm");
	h->getWorld()->findAllContacts();
	h->getWorld()->updateGrasps();

	double eq = mEpsQual->evaluate();
	double vq = mVolQual->evaluate();
	fprintf(flog_quality,"%lf %lf\n", eq, vq);
	fclose(flog_quality);

#endif

#define HANDPOSE_OUTPUT
#ifdef HANDPOSE_OUTPUT
	flog_handpose = fopen("handpose.txt", "a");
	transf p = world->getCurrentHand()->getTran();
	fprintf(flog_handpose, "%s_%d ", world->getGB(0)->getName().ascii(), mExpId);
	fprintf(flog_handpose,"%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		p.translation().x(),
		p.translation().y(),
		p.translation().z(),
		p.rotation().w,
		p.rotation().x,
		p.rotation().y,
		p.rotation().z);
	fclose(flog_handpose);
#endif


#define JOINT_OUTPUT
#ifdef JOINT_OUTPUT
	//log file is the result file as well
	flog_joint = fopen("joints.txt","a");

	std::vector<double> jnts;
	jnts.resize(world->getCurrentHand()->getNumJoints());
	world->getCurrentHand()->getJointValues(&jnts[0]);

	fprintf(flog_joint,"%s_%d ",  world->getGB(0)->getName().ascii(), mExpId);
	for(int jIndex = 0; jIndex < jnts.size(); ++jIndex){
		fprintf(flog_joint,"%lf ", jnts[jIndex]);
	}
	fprintf(flog_joint,"\n");
	fclose(flog_joint);
#endif

#define TACTILE_READING
#ifdef TACTILE_READING
	//generate file name
	std::string fname("c:\\data\\new_barrett_150000_coke_gillette_canteen_uniform_sampling_tactile_experience\\tactile_sensor_reading\\");
	fname.append(world->getGB(0)->getName().ascii());
	fname.append("_");
	sprintf(num,"%d",mExpId);
	//itoa(mExpId,num,10);
	fname.append(num);
	fname.append(".txt");
	flog_tactile = fopen(fname.c_str(),"w");
	QTextStream qts(flog_tactile);
	writeOutTactileInfo(qts);
	fclose(flog_tactile);
#endif

#define TACTILE_LOCATION
#ifdef TACTILE_LOCATION
	fname = std::string("c:\\data\\new_barrett_150000_coke_gillette_canteen_uniform_sampling_tactile_experience\\tactile_sensor_location\\");
	fname.append(world->getGB(0)->getName().ascii());
	fname.append("_");
	sprintf(num,"%d",mExpId);
	//itoa(mExpId,num,10);
	fname.append(num);
	fname.append(".txt");
	flog_sensor = fopen(fname.c_str(),"w");

	for(int sensorInd = 0; sensorInd < world->getNumSensors(); sensorInd ++){
		transf sensorInWorld = world->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
		transf handInWorld = world->getCurrentHand()->getTran(); // considered as world-to-hand transform
		transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

		fprintf(flog_sensor,"%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",sensorInHand.translation().x(), sensorInHand.translation().y(), sensorInHand.translation().z(),
			sensorInHand.rotation().w, sensorInHand.rotation().x, sensorInHand.rotation().y, sensorInHand.rotation().z);
	}
	fclose(flog_sensor);

#endif

	mExpId++;
}

//for exporting tactile information
void TaskControlDlg::writeOutTactileInfo(QTextStream & qts){
	World * w = graspItGUI->getIVmgr()->getWorld();
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		qts << sensorInd << " ";
		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);
	}
}

void TaskControlDlg::init()
{
	if( !graspItGUI->getIVmgr()->getWorld()->getNumRobots() || !graspItGUI->getIVmgr()->getWorld()->getNumGB())
	{
		std::cout << "No robots or graspable bodies found" << std::endl;
		return;
	}
}

void TaskControlDlg::testButton_clicked()
{
	/*
	This function gets all the sampling poses based on each approaching direction
	by backing up every 1 cm along the approaching direction
	*/
	if(mAp.size() == 0)
	{
		position cog = graspItGUI->getIVmgr()->getWorld()->getGB(0)->getCoG() * graspItGUI->getIVmgr()->getWorld()->getGB(0)->getTran();
		//this sampling strategy assumes the z-axis is pointing up
		//we sample around it every 30 degrees
		mAp = sm.getApproachingDirections(vec3(cog.x(), cog.y(), cog.z()),
			////the wine glass, vase, mug, balloon, bottle, rotational objects
			//30*M_PI/180.0, 30*M_PI/180.0, 30*M_PI/180.0, //step is 30 degrees
			//0.5*M_PI, 0.5*M_PI + 1.5*M_PI/180.0, //1.5*M_PI - 1.5*M_PI/180.0, //z, longitude, fixed since these objects are rotationally symmetric
			//0.5*M_PI, M_PI+1.5*M_PI/180.0,//x, latitude, 90 degrees to 180 degrees
			//2); //version 2

			//the duck, rock
			//30*M_PI/180.0, 30*M_PI/180.0, 370*M_PI/180.0, //no rolling is used
			//0.5*M_PI, 1.5*M_PI - 1.5*M_PI/180.0, //z, longitude, 90 degrees to 270 degrees
			//0.5*M_PI + M_PI/6.0, 1.5*M_PI - M_PI/6.0 + 1*M_PI/180, //x, latitude, 120 degrees to 240 degrees
			//2); //version 2

			//the standard version for non-symmetric objects
			//duck, rock, canteen, flashlight, 
			60*M_PI/180.0, 30*M_PI/180.0, 60*M_PI/180.0, //step is 30 degrees, 60 degree step rolling considered
			0, 2*M_PI - 1.5*M_PI/180.0, //z, longitude, 0 degree to 180 degrees
			120*M_PI/180.0 , M_PI + 1.5*M_PI/180.0,//x, latitude, 120 degrees to 150 degrees
			2); //version 2

		std::cout << "loading: " << mAp.size() << std::endl;
		getchar();


	}

	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	for(size_t i = 0; i < mAp.size(); ++i)
	{
		int backup_limit = 10;
		double spread_limit = M_PI/3.0;

		for(double spread = 0.0001; spread < spread_limit; spread += M_PI/12.0) // 0 ~ 120 degrees for the spread angle at an interval of 30 degrees
		{
			h->setTran(mAp[i]);
			//open the hand
			h->autoGrasp(true, -1);
			//reset the breakaways
			resetHandDOFBreakAwayFlags();
			h->forceDOFVal(0, spread);
			std::cout << "new spread angle: " << spread << std::endl;
			//approach the object
			h->approachToContact(300);
			//close the fingers
			h->autoGrasp(true, 1);
			id_i = i;
			id_j = -1;
			intelligentWriteOutPoses();

			//back up
			for(int j = 0; j < backup_limit; ++j)
			{
				std::cout << "i,j: " << i << ", " << j << std::endl;
				h->autoGrasp(true, -1);
				//reset the breakaways
				resetHandDOFBreakAwayFlags();
				//every other 10 mm
				transf t(Quaternion::IDENTITY, vec3(0,0,-10.0));
				h->setTran(t * h->getTran());
				h->autoGrasp(true, 1);
				if(h->getNumContacts() < 1 || selfCollision() )
				{
					std::cout << "break " << j << std::endl;
					std::cout << "self collision result: " << selfCollision() << std::endl;
					break;
				}

				id_j = j;
				intelligentWriteOutPoses();

			}// for every back up pose
		}// for every spread angle
	}// for every approaching direction

	//graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(mAp[mExpId++]);
}

void TaskControlDlg::intelligentWriteOutPoses()
{
	if( numPadsWithTactileContacts() < 2)
	{
		std::cout << "numpads: " << numPadsWithTactileContacts() << std::endl;
		return;
	}
	writeOutCurrentPoses();
}
void TaskControlDlg::writeOutCurrentPoses()
{
	FILE *fp;
	fp = fopen("poses_to_text.txt", "a");
	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	double spread = h->getDOF(0)->getVal();
	transf pose = h->getTran() * 
		graspItGUI->getIVmgr()->getWorld()->getGB(0)->getTran().inverse();//handInWorld * worldInObject
	fprintf(fp,"'{%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}'\n", h->getTran().translation().x(),
		pose.translation().y(),
		pose.translation().z(),
		pose.rotation().w,
		pose.rotation().x,
		pose.rotation().y,
		pose.rotation().z,
		spread);
	fclose(fp);

}

void TaskControlDlg::writeOutGEDFromPoses_deprecated()
{
	/*
	just loop through the poses
	and record the tactile feedback from each of these poses
	*/
	mExpId = 0;
	if(mExpId == 0)
	{
		position cog = graspItGUI->getIVmgr()->getWorld()->getGB(0)->getCoG();
		mAp = sm.getApproachingDirections(vec3(cog.x(), cog.y(), cog.z()),
			5*M_PI/180.0, 5*M_PI/180.0, 5*M_PI/180.0,
			//cokecan and gillette
			M_PI, M_PI + 1*M_PI/180.0, //y
			0, 0.5*M_PI + 1*M_PI/180); //x
		//canteen
		//0.5*M_PI, M_PI + 1*M_PI/180.0, //y
		//0, 0.5*M_PI + 1*M_PI/180); //x

		std::cout << "loading: " << mAp.size() << std::endl;
	}

	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	for(size_t i = 0; i < mAp.size(); ++i)
	{
		h->setTran(mAp[i]);
		//open the hand
		h->autoGrasp(true, -1);
		//reset the breakaways
		resetHandDOFBreakAwayFlags();
		//approach the object
		h->approachToContact(300);
		//close the fingers
		h->autoGrasp(true, 1);
		id_i = i;
		id_j = -1;
		writeOutExpResults();

		//back up
		for(int j = 0; j < 20; ++j)
		{
			std::cout << i << ", " << j << std::endl;
			h->autoGrasp(true, -1);
			//reset the breakaways
			resetHandDOFBreakAwayFlags();
			//every other 10 mm
			transf t(Quaternion::IDENTITY, vec3(0,0,-10.0));
			h->setTran(t * h->getTran());
			h->autoGrasp(true, 1);

			if(h->getNumContacts() < 1 || selfCollision() )
				break;

			if(h->getNumContacts() >= 1 && numPadsWithTactileContacts() < 3)
				continue;

			id_j = j;
			writeOutExpResults();
		}
	}
}