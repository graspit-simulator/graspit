#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>

#include "graspitGUI.h"
#include "ivmgr.h"

#include "body.h"
#include "world.h"
#include "SensorInterface.h"
#include "matvec3D.h"
#include "qstring.h"
#include "qstringlist.h"

void
TactileSensor::resetSensor(){
	for(int ind =0; ind < 6; ind ++)
		myOutput.sensorReading[ind] = 0;
}


bool
TactileSensor::filterContact(Contact * cp){
    position p = cp->getPosition();
    return filterContact(p);
}

bool
TactileSensor::filterContact(position & ps){

    double p0x = myOutput.pos[0].x();
    double p0y = myOutput.pos[0].y();
    double p0z = myOutput.pos[0].z();

    double p1x = myOutput.pos[1].x();
    double p1y = myOutput.pos[1].y();
    double p1z = myOutput.pos[1].z();

#if GRASPITDBG
	if(p0x > p1x || p0y > p1y || p0z > p1z)
		std::cout << "WARNING: THE TACTILE SENSORS ARE NOT DEFINED IN THE CORRECT WAY" << std::endl;
#endif

	if((p0x <= ps[0] && p0y <= ps[1] && p0z <= ps[2]) &&
        (p1x >= ps.x() && p1y >= ps.y() && p1z >= ps.z()))
    {
		return true;
	}

	return false;
}



bool
TactileSensor::setFilterParams(QString * params){
	QStringList qsl = params->split(",");
    myOutput.pos[0][0]= qsl[0].toFloat();
    myOutput.pos[0][1]= qsl[1].toFloat();
    myOutput.pos[0][2]= qsl[2].toFloat();
    myOutput.pos[1][0]= qsl[3].toFloat();
    myOutput.pos[1][1]= qsl[4].toFloat();
    myOutput.pos[1][2]= qsl[5].toFloat();
    return setFilterParams(myOutput.pos);
}

bool TactileSensor::setFilterParams(position pos[]){
	int32_t cIndex[30];
    sbv[0].setValue(myOutput.pos[0][0],myOutput.pos[0][1],myOutput.pos[0][2]);
    sbv[1].setValue(myOutput.pos[0][0],myOutput.pos[1][1],myOutput.pos[0][2]);
    sbv[2].setValue(myOutput.pos[0][0],myOutput.pos[1][1],myOutput.pos[1][2]);
    sbv[3].setValue(myOutput.pos[0][0],myOutput.pos[0][1],myOutput.pos[1][2]);
    sbv[4].setValue(myOutput.pos[1][0],myOutput.pos[0][1],myOutput.pos[0][2]);
    sbv[5].setValue(myOutput.pos[1][0],myOutput.pos[1][1],myOutput.pos[0][2]);
    sbv[6].setValue(myOutput.pos[1][0],myOutput.pos[1][1],myOutput.pos[1][2]);
    sbv[7].setValue(myOutput.pos[1][0],myOutput.pos[0][1],myOutput.pos[1][2]);
	//face 1
	cIndex[0] = 0;
	cIndex[1] = 1;
	cIndex[2] = 2;
	cIndex[3] = 3;
	cIndex[4] = -1;
	//face 2
	cIndex[5] = 0;
	cIndex[6] = 1;
	cIndex[7] = 5;
	cIndex[8] = 4;
	cIndex[9] = -1;
	//face 3
	cIndex[10] = 0;
	cIndex[11] = 3;
	cIndex[12] = 7;
	cIndex[13] = 4;
	cIndex[14] = -1;
	//face 4
	cIndex[15] = 2;
	cIndex[16] = 6;
	cIndex[17] = 7;
	cIndex[18] = 3;
	cIndex[19] = -1;
	//face 5
	cIndex[20] = 1;
	cIndex[21] = 2;
	cIndex[22] = 6;
	cIndex[23] = 5;
	cIndex[24] = -1;
	//face 6
	cIndex[25] = 4;
	cIndex[26] = 5;
	cIndex[27] = 6;
	cIndex[28] = 7;
	cIndex[29] = -1;

	coords->point.setValues(0,8,sbv);
	ifs->coordIndex.setValues(0,30,cIndex);
	IVMat->emissiveColor.setValue(0.5,0.0,0.0);
	IVMat->diffuseColor.setValue(0.0,0,0);
	IVMat->specularColor.setValue(0.0,0.0,0.0);
	IVMat->shininess.setValue(0.0);

	visualIndicator->removeAllChildren();
	visualIndicator->addChild(IVMat);
	visualIndicator->addChild(coords);
	visualIndicator->addChild(ifs);
	sbody->getIVRoot()->addChild(visualIndicator);
	return true;
}

bool
TactileSensor::updateSensorModel(){

	double forces[6] = {0,0,0,0,0,0};
	std::list<Contact *>::const_iterator cp;
	std::list<Contact *> cList = sbody->getContacts();
	//Adding contacts
    if(sbody->getWorld()->dynamicsAreOn())
    {
        for(cp = cList.begin(); cp != cList.end(); cp++){
            double * contactForce = (*cp)->getDynamicContactWrench();
            if(filterContact(*cp))
            {
                forces[2] = 1;
            }
        }
        //Adding Forces for the current sensor pad
        double ts = .0025;
        if (ts > 0.0)
        {
            for(int ind = 0; ind < 6; ind++){
                myOutput.sensorReading[ind] = forces[ind] * (retention_level) + myOutput.sensorReading[ind] * (1.0-retention_level);
            }
            if(myOutput.sensorReading[2] > 0)
            {
                std::cout << "non zero dynamic sensor reading: " << myOutput.sensorReading[2] << std::endl;
            }

        }
    }
    else{
        resetSensor();
        // loop through all the contacts
		for(cp = cList.begin(); cp != cList.end(); cp++){
			if(sbody->getWorld()->softContactsAreOn() && ((*cp)->getBody1()->isElastic() || (*cp)->getBody2()->isElastic())){
                std::vector<position> pVec;
				std::vector<double> forceVec;
				// get the discretized forces and locations within each sub-region of an ellipse
				(*cp)->getStaticContactInfo(pVec, forceVec);

				//get the transform from the body1 to contact
				transf contactInBody1 = (*cp)->getContactFrame();

				//rotation from the contact to the local curvature frame
				transf curvatureFrameInContact;
				mat3 contactRot = (*cp)->getRot();
				curvatureFrameInContact.set(contactRot.inverse(), vec3::ZERO);
				//printMat3(contactRot, "fitRot");

				//transform from local curvature frame to the common frame
				transf commonFrameInCurvature;
				mat3 commonFrameRot = (*cp)->getCommonFrameRot();
				commonFrameInCurvature.set(commonFrameRot.inverse(),vec3::ZERO);

				transf commonFrameInBody1 = commonFrameInCurvature*curvatureFrameInContact*contactInBody1;

				//mat3 commonFrameInBody1Rot;
				//commonFrameInBody1.rotation().inverse().ToRotationMatrix(commonFrameInBody1Rot);
				//printMat3(commonFrameInBody1Rot, "commonFrameInBody1Rot");

//				std::vector<position> renderPoints;
//				renderPoints.clear();
				// for the current contact, we loop through all the discretized sub-regions of this contact
				for (unsigned int pInd = 0; pInd < pVec.size(); pInd ++){
					//pos is the location of the current sensor, specifies the boundaries of the current sensor
					//pVec is the discretized locations within the common contact ellipse
					//We compute this in body1's coordinate system
					//position p = ((1000*pVec[pInd]))*commonFrameInCurvature*curvatureFrameInContact*contact;
					//std::cout << p[0] << "," << p[1] << "," << p[2] << std::endl;
					//renderPoints.push_back( (1000*pVec[pInd])*commonFrameInCurvature*curvatureFrameInContact*contact*(*cp)->getBody1Tran() );

					transf sampleInCommonFrame;
					sampleInCommonFrame.set(Quaternion::IDENTITY, vec3(1000*pVec[pInd].x(), 1000*pVec[pInd].y(), 1000*pVec[pInd].z()) );
					transf sampleInBody1 = sampleInCommonFrame*commonFrameInBody1;
					position sampleLocation;
					sampleLocation.set(sampleInBody1.translation());
//					renderPoints.push_back(sampleLocation * (*cp)->getBody1Tran());
                    if(filterContact(sampleLocation))
					{
                        myOutput.sensorReading[2]+= forceVec[pInd] * 1000;
					}
				}
			}
			else if (filterContact(*cp)){
				myOutput.sensorReading[2] += 1;
			}
		}
    }
    setColor();
}

void TactileSensor::setColor()
{
    float r = myOutput.sensorReading[2];
    float g = 0.2;
    float b = 1.0-myOutput.sensorReading[2];
    IVMat->emissiveColor.setValue(r,g,b);
}

TactileSensor::TactileSensor(Link * body) :
    retention_level(.1)
{
    sbody = body;
    myOutput.sensorReading = new double[6];
    memset(myOutput.sensorReading, 0, 6*sizeof(double));
    sbody->addBodySensor(this);

    coords = new SoCoordinate3;
    ifs = new SoIndexedFaceSet;
    visualIndicator = new SoSeparator;
    IVMat = new SoMaterial;
    return;
}

TactileSensor::~TactileSensor(){
	visualIndicator->removeAllChildren();
	sbody->getIVRoot()->removeChild(visualIndicator);
}

transf TactileSensor::getSensorTran()
{
    transf sensorInLink(Quaternion::IDENTITY,
                        vec3( (myOutput.pos[0][0] + myOutput.pos[1][0])/2.0,
                              (myOutput.pos[0][1] + myOutput.pos[1][1])/2.0,
                              (myOutput.pos[0][2] + myOutput.pos[1][2])/2.0 ));

	transf linkInWorld = sbody->getTran();
    transf res = sensorInLink * linkInWorld;
	return res;
}
