#ifndef _ARIZONA_TEST_H_
#define _ARIZONA_TEST_H_

#include <vector>
#include <matvec3d.h>
#include <QString>

class VirtualContactOnObject;
class PQP_Model;
class Grasp;
class QualityMeasure;
class GraspableBody;
class GWS;
class GWSprojection;
class ArizonaRawExp;

#define EPSILON 10e-5
#define NUMCMPATTERN 4 // define the different cm patterns

class ArizonaTest{

	Grasp *mGrasp;
	QualityMeasure* mQual;
	GraspableBody* mObject;
	bool mIsBuildIn3D;
	bool mIsFlipped;
	std::vector<VirtualContactOnObject*> mContacts;

	// this stores the coordinates of the convex hull
	std::vector<position> coords;
	// indices of the vertices of the convex hull
	std::vector<int> indices;

	// random or blocked or test
	int mCondition; 
	// cm pattern 1-7
	int mCMPattern;

	double forceScales[7];

public:
	ArizonaTest();
	~ArizonaTest();

	void alignAxis(GWS * gws);
	// calculate the disturbance of the specific trial type
	static double* getDisturbance(int trialNumber, double forceScale, double radius);

	//set the object that we run the tests on
	void setObject(GraspableBody *b);

	//set the most primitive data
	void setTestData(ArizonaRawExp* are, QString ellipsoidFile, bool isBuildIn3D, bool isFlipped = false);
	void set3D(bool d){mIsBuildIn3D = d;}
	void setFlipped(bool d) {mIsFlipped = d;}

	//set object set grasp, create quality measure, update the grasp
	void initializeTest();

	double getQuality();

	//get the minimun force based on the trial type i(1-7)
	double getMinimunForce(int i);

	// set the convext hull which will be used for the calculation of the minimun force sum
	void setConvexHull(std::vector<position> coords_gws, std::vector<int> indices_gws);

	// calculate the minimun of sum of normal force for a given disturbance
	double getScale2Surface(double *dis);

	// creates the 3D GWS projection
	void createGWSProjection();

	// creates another gws projection, this time for displaying purposes
	void showGWSProjection();

	bool isOutside(PQP_Model * model, double* pt);
	double binarySearch(const vec3 &direction, PQP_Model &volume, double startValue, double endValue);

	//add one disturbance
	void createDisturbanceSeparator(int trialNumber, SoSeparator * disturbanceSphere, double scale, double forceScale, bool isOriginal);

	void addDisturbance2GWS(GWSprojection* gp);

	double getForce(){ return forceScales[mCMPattern - 1]; }

	void writeForce2File(FILE* fp);

	void writeQuality2File(FILE* fp);

	void writeContact2File(FILE* fp);

	QString getCurrentType();
	QString getCMPattern();		
};

#endif