#ifndef _ARIZONA_RAW_EXP_H_
#define _ARIZONA_RAW_EXP_H_

#include  "matvec3D.h"
#include <vector>
#include <QString>


class ArizonaRawExp
{
	int mID, mTrial, mCenterOfMassPattern;
	enum type {RANDOM, BLOCKED, TEST};
	int mCondition; // blocked or random
	vec3 mCenter, mBase;
	double mRadius;
	std::vector <vec3> mFingerTips; // where the fingertips
	std::vector <vec3> mContacts;
	std::vector <vec3> mNormals;
	std::vector <Quaternion> mQuats;
	std::vector <vec3> mMarkers;

public:
	ArizonaRawExp();
	~ArizonaRawExp();
	
	//parse one line in the raw data file and store the raw data
	bool parse(QString line);

	//calculate the center of the sphere
	vec3 getCenter();

	//calculate the base
	vec3 getBase();

	//normalize the point s.t. the radius is correct
	vec3 normalizePoint(vec3 center, vec3 point, double radius);

	//get the normal
	vec3 getNormal(vec3 center, vec3 point);

	//get the Quatenrion
	Quaternion getQuaternion(vec3 orientation);

	//get Quaternion that is already existent
	Quaternion getQuaternion(int i);

	//process the raw data
	void processLine();

	//set the contacts
	void setContacts();

	vec3 getContact(int i) { return mContacts[i]; }

	//set the normal
	void setNormals();

	vec3 getNormal(int i) { return mNormals[i]; }

	//set the Quatenrion
	void setQuaternions();

	int getNumContacts() { return (int)mContacts.size(); }

	//create the correspoinding virtual grasp file
	void writeToFile(QString ellipsoidFileName, QString outputFileName);

	int getType() { return mCondition; }

	int getCMPattern() { return mCenterOfMassPattern; }


private:
	void printEllipsoid(QString ellipsoidFileName, FILE *fp);
};

#endif