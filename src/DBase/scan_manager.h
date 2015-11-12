#ifndef _SCAN_MANAGER_H_
#define _SCAN_MANAGER_H_

#include "scanSimulator.h"
class ScanManager{
private:
	ScanSimulator sim;
	vec3 mLocation, mDir, mUp, mAt; // both mDir and mUp are just unit vectors indicating directions, mAt is a point on the line of the view
	std::vector<position> mCloud;
	std::vector<RawScanPoint> mRawData;
	std::vector<double> mDepth;
	int mScanType;

public:
	void scanToFile(std::string rawFilePath, std::string modelFilePath);
	void setupCameraPose(vec3 location, vec3 dir, vec3 up, vec3 mAt = vec3(0,0,0));
	void scan1();
	int scan2(std::string model_file_path, std::string output_directory_path, double model_rescale_factor);
		
	void setOptics(double horizontal, double vertical, int hLines, int vLines)
	{
		//need to be adjusted to SwissRanger for now
		sim.setOptics(-horizontal , horizontal , hLines ,
		      -vertical , vertical , vLines);
		sim.setType(ScanSimulator::SCANNER_COORDINATES);
	}

	void saveParamsToFile(FILE * fp);
	std::vector<position> getCloud(){ return mCloud; }
	std::vector<RawScanPoint> getRaw(){ return mRawData; }
	std::vector<double> getDepth(){ return mDepth; }
	bool checkWithin(vec3 origin, vec3 corner, double vertical, double horizontal, vec3 up, vec3 approach);
};
#endif