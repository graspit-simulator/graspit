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

//!Zero the sensor.  Set all values to 0.
void
TactileSensor::resetSensor(){
	for(int ind =0; ind < 6; ind ++)
		myOutput.sensorReading[ind] = 0;
}


//! True if the contact falls within the boundaries of the tactile sensor
//! otherwise false.
bool
TactileSensor::filterContact(Contact * cp){
    position p = cp->getPosition();
    return filterContact(p);
}

//! True if the position falls within the boundaries of the tactile sensor
//! otherwise false.
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

//! Set the 8 points which define the boundary of tactile sensor rectangle.
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

//! Set the 8 points which define the boundary of tactile sensor rectangle.
bool TactileSensor::setFilterParams(position pos[]){

    mSensorBoundingVolume[0].setValue(myOutput.pos[0][0],myOutput.pos[0][1],myOutput.pos[0][2]);
    mSensorBoundingVolume[1].setValue(myOutput.pos[0][0],myOutput.pos[1][1],myOutput.pos[0][2]);
    mSensorBoundingVolume[2].setValue(myOutput.pos[0][0],myOutput.pos[1][1],myOutput.pos[1][2]);
    mSensorBoundingVolume[3].setValue(myOutput.pos[0][0],myOutput.pos[0][1],myOutput.pos[1][2]);
    mSensorBoundingVolume[4].setValue(myOutput.pos[1][0],myOutput.pos[0][1],myOutput.pos[0][2]);
    mSensorBoundingVolume[5].setValue(myOutput.pos[1][0],myOutput.pos[1][1],myOutput.pos[0][2]);
    mSensorBoundingVolume[6].setValue(myOutput.pos[1][0],myOutput.pos[1][1],myOutput.pos[1][2]);
    mSensorBoundingVolume[7].setValue(myOutput.pos[1][0],myOutput.pos[0][1],myOutput.pos[1][2]);

    int32_t coordIndex[30];

	//face 1
    coordIndex[0] = 0;
    coordIndex[1] = 1;
    coordIndex[2] = 2;
    coordIndex[3] = 3;
    coordIndex[4] = -1;
	//face 2
    coordIndex[5] = 0;
    coordIndex[6] = 1;
    coordIndex[7] = 5;
    coordIndex[8] = 4;
    coordIndex[9] = -1;
	//face 3
    coordIndex[10] = 0;
    coordIndex[11] = 3;
    coordIndex[12] = 7;
    coordIndex[13] = 4;
    coordIndex[14] = -1;
	//face 4
    coordIndex[15] = 2;
    coordIndex[16] = 6;
    coordIndex[17] = 7;
    coordIndex[18] = 3;
    coordIndex[19] = -1;
	//face 5
    coordIndex[20] = 1;
    coordIndex[21] = 2;
    coordIndex[22] = 6;
    coordIndex[23] = 5;
    coordIndex[24] = -1;
	//face 6
    coordIndex[25] = 4;
    coordIndex[26] = 5;
    coordIndex[27] = 6;
    coordIndex[28] = 7;
    coordIndex[29] = -1;

    coords->point.setValues(0,8,mSensorBoundingVolume);
    ifs->coordIndex.setValues(0,30,coordIndex);
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


//! update sensor values if dynamics are ON
bool TactileSensor::updateDynamicSensorModel()
{
    double forces[6] = {0,0,0,0,0,0};
    std::list<Contact *>::const_iterator cp;
    std::list<Contact *> cList = sbody->getContacts();

    for(cp = cList.begin(); cp != cList.end(); cp++){
        double * contactForce = (*cp)->getDynamicContactWrench();
        if(filterContact(*cp))
        {
            forces[2] = 1.0;
        }
    }

    for(int ind = 0; ind < 6; ind++){
        double forceValue =  forces[ind];
        myOutput.sensorReading[ind] = forceValue;
    }
}

//! update sensor values if dynamics are OFF
bool TactileSensor::updateStaticSensorModel()
{
    std::list<Contact *>::const_iterator cp;
    std::list<Contact *> cList = sbody->getContacts();

    //Since this in static mode, we want to 0 the sensor first.
    resetSensor();
    // loop through all the contacts
    for(cp = cList.begin(); cp != cList.end(); cp++){

        //If we are dealing with softContacts
        if(sbody->getWorld()->softContactsAreOn() &&
                ((*cp)->getBody1()->isElastic() || (*cp)->getBody2()->isElastic())){
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

            //transform from local curvature frame to the common frame
            transf commonFrameInCurvature;
            mat3 commonFrameRot = (*cp)->getCommonFrameRot();
            commonFrameInCurvature.set(commonFrameRot.inverse(),vec3::ZERO);

            transf commonFrameInBody1 = commonFrameInCurvature*curvatureFrameInContact*contactInBody1;

            // for the current contact, we loop through all the discretized sub-regions of this contact
            for (unsigned int pInd = 0; pInd < pVec.size(); pInd ++){
                //pVec is the discretized locations within the common contact ellipse
                transf sampleInCommonFrame;
                sampleInCommonFrame.set(Quaternion::IDENTITY, vec3(1000*pVec[pInd].x(), 1000*pVec[pInd].y(), 1000*pVec[pInd].z()) );
                transf sampleInBody1 = sampleInCommonFrame*commonFrameInBody1;
                position sampleLocation;
                sampleLocation.set(sampleInBody1.translation());
                if(filterContact(sampleLocation))
                {
                    myOutput.sensorReading[2]+= forceVec[pInd] * 1000;
                }
            }
        }
        //We are dealing with normal contacts, not soft contacts
        else{
            if(filterContact(*cp)){
                myOutput.sensorReading[2] += 1;
            }
        }
    }
}


bool
TactileSensor::updateSensorModel(){

    if(sbody->getWorld()->dynamicsAreOn())
    {
        updateDynamicSensorModel();
    }
    else{
        updateStaticSensorModel();
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
