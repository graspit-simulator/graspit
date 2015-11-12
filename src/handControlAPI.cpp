#include "handControlAPI.h"
#include <QString>
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "SensorInterface.h"
Hand* HandControlAPI::h;

void HandControlAPI::importHand()
{
	h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	if(h)
		return;
	QString filename;
	QString graspitRoot = QString(getenv("GRASPIT"));
	filename = graspitRoot + QString("/models/robots/NewBarrett/NewBarrett.xml");
	h = (Hand*)graspItGUI->getIVmgr()->getWorld()->importRobot(filename);
}

void HandControlAPI::setJointValues(const std::vector<double>& j)
{
	if(j.size() != h->getNumJoints())
	{
		std::cout << "Different number of joints" << std::endl;
	}

	std::cout << "joint values sent to hand"  << std::endl;
	h->setJointValuesAndUpdate(&j[0]);
}

void HandControlAPI::outputTactileLocations()
{
	FILE *flog_sensor;
	std::string fname;
#define SENSOR_LOCATION
#ifdef SENSOR_LOCATION

	fname = std::string("c:\\sensor\\test.txt");
	flog_sensor = fopen(fname.c_str(),"a");

	World * w = graspItGUI->getIVmgr()->getWorld();
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
		transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
		transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

		fprintf(flog_sensor,"%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",sensorInHand.translation().x(), sensorInHand.translation().y(), sensorInHand.translation().z(),
			sensorInHand.rotation().w, sensorInHand.rotation().x, sensorInHand.rotation().y, sensorInHand.rotation().z);
	}
	fclose(flog_sensor);
#endif
}

std::vector< std::vector<double> > HandControlAPI::getTactileLocations()
{
	std::vector<double> contact;
	std::vector< std::vector<double> > contactList;
	contactList.clear();
	World * w = graspItGUI->getIVmgr()->getWorld();
        if(w->getNumSensors() < 1)
          std::cout << "No sensors found" << std::endl;
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
		transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
		transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor
		contact.clear();
		contact.push_back(sensorInHand.translation().x());
		contact.push_back(sensorInHand.translation().y());
		contact.push_back(sensorInHand.translation().z());
		contact.push_back(sensorInHand.rotation().w);
		contact.push_back(sensorInHand.rotation().x);
		contact.push_back(sensorInHand.rotation().y);
		contact.push_back(sensorInHand.rotation().z);

		contactList.push_back(contact);
	}
	return contactList;

}

void HandControlAPI::setPose(transf t)
{
  h->setTran(t);
}

void HandControlAPI::autoGrasp(bool render)
{
  h->autoGrasp(render);
}

void HandControlAPI::forceOpen()
{
  h->forceDOFVal(1,0);
  h->forceDOFVal(2,0);
  h->forceDOFVal(3,0);
}
