#ifndef BODY_SENSOR_H
#define BODY_SENSOR_H

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
  //! The position position of the "bottom left" and "upper right" corners
  //! defining the sensor location
  position pos[2];

  //! value of the sensor reading
  double *sensorReading;
};

class BodySensor
{
  public:
    virtual SensorReading *getSensorOutput() = 0;
    virtual bool updateSensorModel() = 0;
    virtual void resetSensor() = 0;
    virtual transf getSensorTran() = 0;
    virtual double getNormalForce() {return -1;}
    virtual ~BodySensor() {}
};



class TactileSensor : public BodySensor
{

  protected:

    Link *mBody;
    SensorReading mOutput;
    SoSeparator *mVisualIndicator;
    SoCoordinate3 *mCoords;
    SoMaterial *mIVMat;
    SoIndexedFaceSet *mIndexedFaceSet;

    bool updateDynamicSensorModel();
    bool updateStaticSensorModel();

    virtual void setColor();
    bool filterContact(Contact *cp);
    bool filterContact(position &ps);

  public:
    TactileSensor(Link *body);

    virtual SensorReading *getSensorOutput() {return &mOutput;}
    virtual bool updateSensorModel();
    virtual double getNormalForce() {return mOutput.sensorReading[2];}

    virtual void resetSensor();
    bool setFilterParams(QString *params);
    bool setFilterParams(position pos[]);

    SbVec3f mSensorBoundingVolume[8];

    virtual transf getSensorTran();

    ~TactileSensor();

};


#endif
