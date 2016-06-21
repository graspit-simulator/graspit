#ifndef GRASPIT_SENSORINTERFACE_H
#define GRASPIT_SENSORINTERFACE_H

class Link;
class SoSeparator;
class SoTransform;
class SoIndexedFaceSet;
class SoCoordinate3;
class SoMaterial;
class QString;
class Contact; 
class QTextStream;

struct SensorReading
{
    position pos[2];
    double * sensorReading;
};

class BodySensor
{
public:
    virtual SensorReading * getSensorOutput() = 0;
    virtual bool updateSensorModel() = 0;
    virtual void resetSensor() = 0;
    virtual transf getSensorTran() = 0;
    virtual double getNormalForce(){return -1;}
    virtual ~BodySensor(){}
};



class TactileSensor : public BodySensor
{

protected:

    Link * sbody;
    SensorReading myOutput;
    double retention_level;
    SoSeparator *visualIndicator;
    SoCoordinate3 *coords;
    SoMaterial * IVMat;
    SoIndexedFaceSet * ifs;

    bool updateDynamicSensorModel();
    bool updateStaticSensorModel();

    virtual void setColor();
    bool filterContact(Contact * cp);
    bool filterContact(position & ps);

public:
    TactileSensor(Link * body);

    virtual SensorReading * getSensorOutput(){return &myOutput;}
    virtual bool updateSensorModel();
    virtual double getNormalForce(){return myOutput.sensorReading[2];}

    virtual void resetSensor();
	bool setFilterParams(QString * params);
	bool setFilterParams(position pos[]);

	SbVec3f sbv[8];

	virtual transf getSensorTran();

    ~TactileSensor();

};



#endif
