#ifndef _SEMANTIC_MAP_H_
#define _SEMANTIC_MAP_H_

#include <string>
#include <vector>

#include "matvec3D.h"
#include "SemanticGrasp.h"

struct compareItem{
int sampleIdx;
double distance;
};

class SemanticMap{
private:
	std::string mCodebookFilePath;
	std::string mSemanticGraspFilePath;
	int mNumSamples, mNumBins, mIntervalD, mIntervalH, mIntervalV;
	std::vector<std::vector<double> > mCodebook;
	std::vector<SemanticGrasp> mSemanticGraspList;
	std::vector<double> mTargetScan;
	std::vector<double> mTargetScanCode;
	std::vector<compareItem> mComparison;

	std::vector<double> getMin(std::vector<std::vector<double> > data);
	std::vector<double> getMax(std::vector<std::vector<double> > data);

	void normalize_L1(std::vector<double> &vec);
	void normalize_L2(std::vector<double> &vec);

	//! Output is in radians
	std::vector<double> getSphericalCoordinateFromDepth(int i, int j, double depth);

	int getBinIndex(double distance, double delta_h, double delta_v,
		double dist_1, double dist_2, double fov_h_1, double fov_h_2, double fov_v_1, double fov_v_2,
		int num_d, int num_h, int num_v);

	//! Just euclidean distance
	double computeDistance(std::vector<double> item1, std::vector<double> item2);

	//! Input is in radians
	double getSphericalDistance(double longitude1, double latitude1, double longitude2, double latitude2);

public:
	SemanticMap() : mNumSamples(0), mNumBins(0)
	{

	}
	void setCodebookFilePath(std::string path) { mCodebookFilePath = path; }
	void setSemanticGraspFilePath(std::string path) { mSemanticGraspFilePath = path; }
	void loadCodebook();
	void loadSemanticGrasp();
	int getNumSamples(){ return mNumSamples; }
	int getNumBins(){ return mNumBins; }
	int getIntervalD(){ return mIntervalD; }
	int getIntervalH(){ return mIntervalH; }
	int getIntervalV(){ return mIntervalV; }
	void setTargetScan(std::vector<double> d){ mTargetScan = d; }
	void encodeTargetScan();

	//! output is in radians
	std::vector<double> getLongitudeLatitude(int index, bool isHemisphere = false);
	transf getOriginalPose(double longitude, double latitude, vec3 cog, double back = -130);
	transf getOriginalPose2(double longitude, double latitude, vec3 cog, double back = -130);
	std::vector<compareItem> getNN();
	compareItem getVotingWinner(std::vector<compareItem> neighbors, int numNeighbors);
	//! computes in the current model's coordinate system the best pose specified in sample object by currentSampleIdx
	transf getBestPoseInCurrentModel(int currentSampleIdx, int bestSampleIdx, vec3 cog);

	std::vector<SemanticGrasp> getSemanticGraspList() { return mSemanticGraspList; }

	/*
	based on the hand coordinate system for the transformation
	*/
	std::vector<transf> getApproachingDirections(vec3 cog = vec3::ZERO, double stepLongitude = 5.0 * M_PI/180.0, double stepLatitude = 5.0 * M_PI/180.0, double stepRolling = 5 * M_PI/180.0,
		double longitudeStart=0.0, double longitudeEnd=2*M_PI, double latitudeStart=-M_PI/2, double latitudeEnd=M_PI/2, int version = 1);
};
#endif //_SEMANTIC_MAP_H_
