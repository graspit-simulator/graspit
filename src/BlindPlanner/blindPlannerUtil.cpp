/*
* BLINDpLannerUtil.cpp
*
*  Created on: Jun 21, 2011
*      Author: dang
*/
#include "blindPlannerUtil.h"

#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "matvec3D.h"
#include "SensorInterface.h"
#include "quality.h"

#include <sstream>
//#define PROF_ENABLED
#include "profiling.h"

#ifndef WIN32
#include <sys/types.h>
#include <dirent.h>
#endif

PROF_DECLARE(BREAKAWAY_RESET);

void getCurrentGraspQuality(double * epsilon, double * volume);

double getCurrentHandSpread()
{
	Hand* h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	std::vector<double> jnts;
	jnts.resize(h->getNumJoints());
	h->getJointValues(&jnts[0]);
	return jnts[0];
}

void resetHandDOFBreakAwayFlags()
{
	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	//reset the breakaway dof's if necessary
	if(h->isA("Barrett"))
	{
		//std::cout << "is a barrett hand" << std::endl;
		//for every dof that is returned back to zero, reset it
		for(size_t i = 0; i < h->getNumDOF(); ++i)
		{
			if(h->getDOF(i)->getVal() < 0.000001)
			{
				h->getDOF(i)->reset();
			}
		}
	}
}

void resetHandPoseToZero()
{
	PROF_START_TIMER(BREAKAWAY_RESET);
	PROF_RESET(BREAKAWAY_RESET);
	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	resetHandDOFBreakAwayFlags();//5ms
	//PROF_PRINT(BREAKAWAY_RESET);

	h->forceDOFVal(0,0.00001);//10ms
	//PROF_PRINT(BREAKAWAY_RESET);
	h->forceDOFVal(1,0.00001);//70ms
	//PROF_PRINT(BREAKAWAY_RESET);
	h->forceDOFVal(2,0.00001);//70ms
	//PROF_PRINT(BREAKAWAY_RESET);
	h->forceDOFVal(3,0.00001);//70ms
	PROF_STOP_TIMER(BREAKAWAY_RESET);
	//PROF_PRINT(BREAKAWAY_RESET);
}

std::vector<ContactInfo> getCurrentGraspContactInfoList(World * w)
{
	std::vector<ContactInfo> contactInfoList;

	if(!w)
		w = graspItGUI->getIVmgr()->getWorld();
	//the contact list
	double epsilon = 0.00001;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		QString str;
		QTextStream qts(&str, QIODevice::WriteOnly);

		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);

		//std::string str = qts.string()->toStdString();
		int id;
		char sensorType[20];
		//bounding box of the sensor
		double lx, ly, lz, rx, ry, rz;
		//force and torque
		double fx, fy, fz, tx, ty, tz;
		sscanf(str.toStdString().c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", sensorType,
			&lx, &ly, &lz, &rx, &ry, &rz,
			&fx, &fy, &fz, &tx, &ty, &tz);
		//std::cout << str.toStdString().c_str() << std::endl;
		//std::cout << sensorType << " " << lx << " " << ly << " " << lz << std::endl;
		//getchar();

		if(fz < epsilon) //there is no contact
			continue;

		//get the contact orientation
		transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
		transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
		transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

		ContactInfo ci;
		ci.version = 1;
		ci.force = fz;
		ci.location = sensorInHand.translation();
		ci.orientation = sensorInHand.rotation();
		contactInfoList.push_back(ci);
		//ci.printMe();
	}
	return contactInfoList;
}

std::vector<double> getTactileSensorReadings(World * w)
{
	std::vector<double> tactileSensorReadings;
	tactileSensorReadings.resize(0);

	if(!w)
		w = graspItGUI->getIVmgr()->getWorld();
	//the contact list
	double epsilon = 0.00001;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		QString str;
		QTextStream qts(&str, QIODevice::WriteOnly);

		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);

		//std::string str = qts.string()->toStdString();
		int id;
		char sensorType[20];
		//bounding box of the sensor
		double lx, ly, lz, rx, ry, rz;
		//force and torque
		double fx, fy, fz, tx, ty, tz;
		sscanf(str.toStdString().c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", sensorType,
			&lx, &ly, &lz, &rx, &ry, &rz,
			&fx, &fy, &fz, &tx, &ty, &tz);
		//std::cout << str.toStdString().c_str() << std::endl;
		//std::cout << sensorType << " " << lx << " " << ly << " " << lz << std::endl;
		//getchar();

		tactileSensorReadings.push_back( (fz < epsilon ? 0 : fz) );
	}
	return tactileSensorReadings;
}


/*
This is used for ROS message format
*/
std::vector<ContactInfo> getContactInfoList(std::vector<double> tactileReadings, World * w)
{
	std::vector<ContactInfo> contactInfoList;
	if(!w)
		w = graspItGUI->getIVmgr()->getWorld();

	if(tactileReadings.size() != w->getNumSensors())
	{
		printf("The number of sensors input is not correct, input number is: %d while in the world we have %d sensors", tactileReadings.size(), w->getNumSensors());
	}

	//the contact list
	double epsilon = 0.00001;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		QString str;
		QTextStream qts(&str, QIODevice::WriteOnly);

		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);

		//std::string str = qts.string()->toStdString();
		int id;
		char sensorType[20];
		//bounding box of the sensor
		double lx, ly, lz, rx, ry, rz;
		//force and torque
		double fx, fy, fz, tx, ty, tz;
		sscanf(str.toStdString().c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", sensorType,
			&lx, &ly, &lz, &rx, &ry, &rz,
			&fx, &fy, &fz, &tx, &ty, &tz);
		//std::cout << str.toStdString().c_str() << std::endl;
		//std::cout << sensorType << " " << lx << " " << ly << " " << lz << std::endl;
		//getchar();

		if(tactileReadings[sensorInd] < epsilon) //there is no contact
			continue;

		//get the contact orientation
		transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
		transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
		transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

		ContactInfo ci;
		ci.location = sensorInHand.translation();
		ci.orientation = sensorInHand.rotation();
		contactInfoList.push_back(ci);
		//ci.printMe();
	}
	return contactInfoList;
}

void getContactInfoListPerPad(std::vector<double> tactileReadings,
							  std::vector<ContactInfo>& palm,
							  std::vector<ContactInfo>& f1,
							  std::vector<ContactInfo>& f2,
							  std::vector<ContactInfo>& f3,
							  World * w)
{

	if(!w)
		w = graspItGUI->getIVmgr()->getWorld();

	if(tactileReadings.size() != w->getNumSensors())
	{
		printf("The number of sensors input is not correct, input number is: %d while in the world we have %d sensors", tactileReadings.size(), w->getNumSensors());
	}

	//the contact list
	double epsilon = 0.00001;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		QString str;
		QTextStream qts(&str, QIODevice::WriteOnly);

		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);

		//std::string str = qts.string()->toStdString();
		int id;
		char sensorType[20];
		//bounding box of the sensor
		double lx, ly, lz, rx, ry, rz;
		//force and torque
		double fx, fy, fz, tx, ty, tz;
		sscanf(str.toStdString().c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", sensorType,
			&lx, &ly, &lz, &rx, &ry, &rz,
			&fx, &fy, &fz, &tx, &ty, &tz);
		//std::cout << str.toStdString().c_str() << std::endl;
		//std::cout << sensorType << " " << lx << " " << ly << " " << lz << std::endl;
		//getchar();

		if(tactileReadings[sensorInd] < epsilon) //there is no contact
			continue;

		//get the contact orientation
		transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
		transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
		transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

		ContactInfo ci;
		ci.location = sensorInHand.translation();
		ci.orientation = sensorInHand.rotation();

		//distribute to the right output vector
		if(sensorInd < 24)
			palm.push_back(ci);
		if(sensorInd >= 24 && sensorInd < 48)
			f1.push_back(ci);
		if(sensorInd >= 48 && sensorInd < 72)
			f2.push_back(ci);
		if(sensorInd >= 72 && sensorInd < 96)
			f3.push_back(ci);
		//ci.printMe();
	}
}


bool selfCollision()
{
	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	std::list<Contact *>cl = h->getContacts();
	bool hasSelfCollision = false;
	for(std::list<Contact *>::iterator cp = cl.begin(); cp != cl.end(); ++cp)
	{
		//std::cout << (*cp)->getBody1()->getOwner()->getName().toStdString() << std::endl;
		//std::cout << (*cp)->getBody2()->getOwner()->getName().toStdString() << std::endl;
		if( (*cp)->getBody1()->getOwner() == (*cp)->getBody2()->getOwner() )
			hasSelfCollision = true;
	}
	return hasSelfCollision;
}

int numPadsWithTactileContacts(World * w)
{
	if(!w)
		w = graspItGUI->getIVmgr()->getWorld();
	//the contact list
	double epsilon = 0.00001;
	int res = 0;

	for(size_t i = 0; i < 4; ++i) //4 pads
	{
		for(int sensorInd = 0; sensorInd < 24; sensorInd ++) // 24 cells a pad
		{
			w->getSensor(24*(i) + sensorInd)->updateSensorModel();
			if(w->getSensor( 24*(i) + sensorInd )->getNormalForce() > epsilon) //there is no contact
			{
				//std::cout << i << ", " << w->getSensor( 24*(i) + sensorInd )->getNormalForce() << std::endl;
				res ++;
				break;
			}
		}
	}

	return res;
}

void writeOutAsTactileExperience(std::string id, std::string surfix)
{
	World *world = graspItGUI->getIVmgr()->getWorld();
	Hand *hand = world->getCurrentHand();
	GraspableBody * body = world->getGB(0);
	FILE *flog_handpose, *flog_tactile;
	char filename[512];
	sprintf(filename, "perturbed_handpose_cgdb_%s.txt", id.c_str());
	flog_handpose = fopen(filename, "a");

	transf t = hand->getTran();
	fprintf(flog_handpose, "%s_%s %lf %lf %lf %lf %lf %lf %lf ", id.c_str(), surfix.c_str(), t.translation().x(), t.translation().y(), t.translation().z(),
		t.rotation().w, t.rotation().x, t.rotation().y, t.rotation().z);

	//spread angle
	fprintf(flog_handpose, "%lf\n", hand->getDOF(0)->getVal());

	fclose(flog_handpose);

	sprintf(filename, "perturbed_experience_cgdb_%s.txt", id.c_str());
	flog_tactile = fopen(filename, "a");

	double epsilon, volume;
	getCurrentGraspQuality(&epsilon, &volume);
	std::vector<ContactInfo> tactile_exp = getCurrentGraspContactInfoList();
	fprintf(flog_tactile, "%s_%s %lf %lf %d ", id.c_str(), surfix.c_str(), epsilon, volume, tactile_exp.size());

	for(size_t k = 0; k < tactile_exp.size(); ++k)
	{
		fprintf(flog_tactile, "%lf %lf %lf %lf %lf %lf %lf %lf ",
			tactile_exp[k].force,
			tactile_exp[k].location.x(),
			tactile_exp[k].location.y(),
			tactile_exp[k].location.z(),
			tactile_exp[k].orientation.w,
			tactile_exp[k].orientation.x,
			tactile_exp[k].orientation.y,
			tactile_exp[k].orientation.z
			);
	}
	fprintf(flog_tactile,"\n");
	fclose(flog_tactile);
}

void getCurrentGraspQuality(double * epsilon, double * volume)
{
	QualEpsilon* epsQual;
	//! Volume quality measure
	QualVolume* volQual;
	World *world = graspItGUI->getIVmgr()->getWorld();
	Hand *hand = world->getCurrentHand();

	epsQual = new QualEpsilon( hand->getGrasp(), "Examine_dlg_qm","L1 Norm");
	volQual = new QualVolume( hand->getGrasp(), "Examine_dlg_qm","L1 Norm");

	world->findAllContacts();
	world->updateGrasps();

	*epsilon = epsQual->evaluate();
	*volume = volQual->evaluate();

}

std::vector<ContactInfo> getContactWithLargestNormalForce(std::vector<ContactInfo> clist)
{
	if(clist.size() < 1)
		return clist;
	int optimalIndex = -1;
	double optimalForce = -1;
	for(int i = 0; i < clist.size(); ++i)
	{
		if(clist[i].force > optimalForce)
		{
			optimalIndex = i;
		}
	}
	std::vector<ContactInfo> res;
	res.push_back(clist[optimalIndex]);
	return res;
}

std::string getGraspCommand(transf finalPose, double* joints, bool asAdjustment, std::string cmd_prefix, double s)
{
	World* world = graspItGUI->getIVmgr()->getWorld();
	//transf t = world->getCurrentHand()->getTran();
	transf t = finalPose;

	//10cm back from the target pose
	transf backup(Quaternion::IDENTITY, vec3(0,0,-0));
	transf pre = backup * t;

	//double joints[7];
	//world->getCurrentHand()->getJointValues(joints);

	std::stringstream msg;
	double scale;
	if( !cmd_prefix.empty() )//this cmd prefix overides all others
	{
		msg << cmd_prefix << " ";
		scale = s;
	}
	else
	{
		if(asAdjustment)
		{
			msg << "A ";
			scale = 1.0;
		}
		else
		{
			msg << "G ";
			scale = 1000.0;
		}
	}

	msg << "[-1, -1, " <<
		//pregrasp position
		pre.translation().x()*scale << ", " << 
		pre.translation().y()*scale << ", " << 
		pre.translation().z()*scale << ", " << 
		pre.rotation().w << ", " <<
		pre.rotation().x << ", " <<
		pre.rotation().y << ", " <<
		pre.rotation().z << ", " <<
		//pregrasp dof
		joints[0] << ",0,0,0" << ", " <<
		//source
		"3, " <<
		// group id
		"1" <<
		"]" <<
		" [-1, -1, " <<
		//finalgrasp position
		t.translation().x()*scale << ", " << 
		t.translation().y()*scale << ", " << 
		t.translation().z()*scale << ", " << 
		t.rotation().w << ", " <<
		t.rotation().x << ", " <<
		t.rotation().y << ", " <<
		t.rotation().z << ", " <<
		//finalgrasp dof
		joints[0] << ", " << 
		joints[1] << ", " <<
		joints[4] << ", " <<
		joints[6] << "]";

	return msg.str();
}
void getFileNames(std::vector<string> &out, const string &directory, const string &filter)
{
#ifdef WIN32
	HANDLE dir;
	WIN32_FIND_DATA file_data;

	std::string str = directory + std::string("/*");
	std::wstring dirname;
	dirname.assign(str.begin(), str.end());
	if ((dir = FindFirstFile( dirname.c_str(), &file_data)) == INVALID_HANDLE_VALUE)
		return; /* No files found */

	do {
		wchar_t* wchr = file_data.cFileName;
		char* ascii = new char[wcslen(wchr) + 1];
		size_t l = wcslen(wchr);
		wcstombs( ascii, wchr, wcslen(wchr) );
		ascii[l] = '\0';
		const string file_name(ascii);
		const string full_file_name = directory + "/" + file_name;
		const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

		if (file_name[0] == '.')
			continue;

		if (is_directory)
			continue;

		if(file_name.find(filter) == std::string::npos)
			continue;

		out.push_back(file_name);
	} while (FindNextFile(dir, &file_data));

	FindClose(dir);
#else
	struct dirent *de=NULL;
	DIR *d=NULL;

	d=opendir(directory.c_str());
	if(d == NULL)
	{
		perror("Couldn't open directory");
		return;
	}

	// Loop while not NULL
	while(de = readdir(d))
	{
		//printf("%s\n",de->d_name);
		std::string file_name = std::string(de->d_name);
		if(file_name.find(filter) == std::string::npos)
			continue;
		out.push_back(file_name);
	}

	closedir(d);
#endif
}
