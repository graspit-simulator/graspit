#ifndef GRASPIT_SENSORINTERFACE_H
#define GRASPIT_SENSORINTERFACE_H

enum SensorType {BODY,TBODY};
class Link;
class SensorLink;
class SoSeparator;
class SoTransform;
class SoIndexedFaceSet;
class SoCoordinate3;
class SoMaterial;
class QString;
class Contact; 
class QTextStream;

struct SensorOutput{
    SensorType stype;
    position pos[2];
    double * sensorReading;
};

class SensorInterface{
public:
    //front end should allow only polling and resetting the sensor,
    //or asking the sensor to update its model.
    //It should also support asking for visualization of the sensor output
    //and sensor groupname, which may be relevant to control strategies.
    virtual SensorOutput * getSensorOutput() = 0;
    virtual bool updateSensorModel() = 0;
    virtual void resetSensor() = 0;
    virtual SoSeparator * getVisualIndicator() = 0;
    virtual int getGroupNumber() = 0;
    virtual transf getSensorTran() = 0;
    virtual void setColor(double maxVal) = 0;
    virtual double getNormalForce(){return -1;}
};


class BodySensor : public SensorInterface{
private:
	void init(Link * body);
protected:
	SensorLink * sbody;
	SensorOutput myOutput;
	static double retention_level;
	double getTimeStep();
	int groupNumber;
public:
    virtual SensorOutput * getSensorOutput(){return &myOutput;}
    virtual bool updateSensorModel();
    bool setGroupNumber(int gn);
    virtual int getGroupNumber(){return groupNumber;}
    virtual void resetSensor();
    virtual SoSeparator * getVisualIndicator();
    BodySensor(Link* body);
    BodySensor(const BodySensor & fs, Link * sl);
    virtual BodySensor * clone(Body* b){Q_UNUSED(b); return new BodySensor(*this);}
    virtual transf getSensorTran();
    virtual void setColor(double maxVal){Q_UNUSED(maxVal);}
    virtual double getNormalForce(){return myOutput.sensorReading[2];}
};

class RegionFilteredSensor : public BodySensor {
private:
	void init();

protected:
	bool filterContact(Contact * cp);
	bool filterContact(const position & boundaryPos0, const position & boundaryPos1, const position & ps);
	SoSeparator *visualIndicator;
	SoCoordinate3 *coords;
	SoMaterial * IVMat;
	SoIndexedFaceSet * ifs;
public:
	bool setFilterParams(QString * params);
	bool setFilterParams(position pos[]);
	RegionFilteredSensor(Link * body);
	RegionFilteredSensor(const RegionFilteredSensor & fs, Link * sl);
	SoSeparator * getVisualIndicator();
	SbVec3f sbv[8];
	virtual BodySensor * clone(SensorLink * sl);
	~RegionFilteredSensor();
	virtual transf getSensorTran();
	virtual void setColor(double maxVal);
    virtual bool updateSensorModel();
};



#endif
