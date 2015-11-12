#ifndef _ARIZONA_PROCESS_H_
#define _ARIZONA_PROCESS_H_

class GraspableBody;

#include <vector>
#include "qstring.h"
#include "Arizona_Raw_Exp.h"
#include "Arizona_Test.h"
/*
In Arizona Project, ArizonaProcess manipulates ArizonaTest and ArizonaRawExp together.
ArizonaTest deals with only one trial
ArizonaRawExp deals with only one trial as well
*/
class ArizonaProcess{
	// mTotal number of total trials loaded in
	// mCurrent index of the current frame
	int mTotal, mCurrent;
	std::vector<ArizonaRawExp*> mRawData;
	ArizonaTest* mAt;


public:
	ArizonaProcess();
	~ArizonaProcess();

	// load in the raw data and store it in rawData
	// at the same time, preprocess the data to find the center, base, ...
	bool load_and_preprocess(QString fileName);

	// save all the loaded and pre-processed data into the selected folder
	void saveExpToVGR(QString rawFileName, QString ellipsoidFileName, QString destinationFileFolder);

	// set the graspable object
	void setObject(GraspableBody* object);

	// get the next frame
	ArizonaRawExp * getNextRawData();
	//get the previous frame
	ArizonaRawExp * getCurrentRawData();
	//get the previous frame
	ArizonaRawExp * getPrevRawData();

	ArizonaTest* getTest(){ return mAt; }

	int getCurrentExp() { return mCurrent; }

	int getTotalExp() { return mTotal; }

	void testAllQual(bool isBuildIn3D, bool isFlipped, QString ellipsoidFileName, QString fn);

	void testAllForce(bool isBuildIn3D, bool isFlipped, QString ellipsoidFileName, QString fn);

	void writeAllContact(bool isBuildIn3D, bool isFlipped, QString ellipsoidFileName, QString fn);

};

#endif