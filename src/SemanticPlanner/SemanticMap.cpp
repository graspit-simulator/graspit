#include "SemanticMap.h"
#include "math.h"

#include <algorithm>
#include <iostream>
#include <fstream>

//helper functor that decide the order of two grasps based on their average test score
bool shorterInDistance(compareItem i1, compareItem i2){
	return i1.distance < i2.distance;
}

void SemanticMap::loadCodebook()
{
	std::vector<double> code;
	double num;
	FILE * fp;
	fp = fopen(mCodebookFilePath.c_str(), "r");

	fscanf(fp, "%d %d %d %d %d\n", &mNumSamples, &mNumBins, &mIntervalD, &mIntervalH, &mIntervalV);
	for(int i = 0; i < mNumSamples; ++i){
		code.clear();
		for(int j = 0; j < mNumBins - 1; ++j){
			fscanf(fp, "%lg ", &num);
			code.push_back(num);
		}
		fscanf(fp,"%lg\n", &num);
		code.push_back(num);
		mCodebook.push_back(code);
	}
	fclose(fp);
}

void SemanticMap::loadSemanticGrasp()
{
	mSemanticGraspList.clear();
	FILE * fp = fopen(mSemanticGraspFilePath.c_str(), "r");
	int numOfGrasps;
	char handName[256], semantics[256];
	int approachingDirectionID;
	double rollingAngleInDegree;
	fscanf(fp,"%d\n", &numOfGrasps);
	for(int i = 0; i < numOfGrasps; ++i)
	{
		fscanf(fp, "%s\n", handName);
		fscanf(fp, "%s\n", semantics);
		fscanf(fp, "%d\n", &approachingDirectionID);
		fscanf(fp, "%lf\n", &rollingAngleInDegree);

		//dof
		int numDOF;
		double dof;
		std::vector<double> dofs;
		fscanf(fp, "%d", &numDOF);
		for(int j = 0; j < numDOF; ++j)
		{
			fscanf(fp, "%lf", &dof);
			dofs.push_back(dof);
		}



		char paramsPath[128], tePath[128];
		fscanf(fp, "%s\n", paramsPath);

		//tactile feedback
		int numTactile;
		double tactile;
		std::vector<double> tactiles;
		fscanf(fp, "%d", &numTactile);
		for(int j = 0; j < numTactile; ++j)
		{
			fscanf(fp, "%lf", &tactile);
			tactiles.push_back(tactile);
		}

		//path to tactile experience
		fscanf(fp, "%s\n", tePath);
		
		SemanticGrasp sg;
		sg.setHandName(handName);
		sg.setSemantics(semantics);
		sg.setApproachingDirection(approachingDirectionID);
		sg.setRollingAdjustment(rollingAngleInDegree);
		sg.setTactile(tactiles);
		sg.setDOFs(dofs);
		//sg.setContactList(cl);
		sg.setMap(this);
		sg.setGraspParamsFilePath(std::string(paramsPath));
		sg.setGraspTactileExperienceFilePath(std::string(tePath));

		////////set the grasp params
		//////std::ifstream file(paramsPath);
		//////if(!file.is_open())
		//////{
		//////	std::cout << "File not found: " << paramsPath << std::endl;
		//////	continue;
		//////}
		//////std::string line;
		//////	//file opened
		//////getline(file, line);

		//////int dofType, offset, offset_so_far;
		//////double dof;
		//////std::vector<double> dofs;

		//////sscanf(line.c_str(), "%d %n", &dofType, &offset);
		//////offset_so_far = offset;
		//////sg.setDOFType(dofType);
		//////while(true)
		//////{
		//////	int val = sscanf(line.c_str()+offset_so_far, "%lf %n", &dof, &offset);
		//////	offset_so_far += offset;
		//////	if(val == EOF || val < 1)
		//////		break;
		//////	dofs.push_back(dof);
		//////}
		//////sg.setDOFs(dofs);
		//////file.close();

		//set tactile experience
		std::ifstream file;
		file.open(tePath);
		std::string line;
		if(!file.is_open())
		{
			std::cout << "File not found: " << tePath << std::endl;
			continue;
		}
			//file opened
		getline(file, line);
		sscanf(line.c_str(), "%lf", &dof);
		getline(file, line);
		GraspExperienceEntry gee(line.c_str(),1);
		//hard code the spread angle from the semantic grasp
		//this is hack for iros 2012, should consider all the dof that are under control, but for Barrett hand, this is the only one
		gee.setSpread(dof);
		sg.setGraspExperienceEntry(gee);
		file.close();

		mSemanticGraspList.push_back(sg);
	}
	fclose(fp);
}

void SemanticMap::encodeTargetScan()
{
	mTargetScanCode.clear();
	mTargetScanCode.resize(mNumBins, 0);
	std::vector< std::vector<double> > dhvs;
	std::vector<double> dhv;
	for(size_t i = 0; i < mTargetScan.size(); ++i)
	{
		if(mTargetScan[i] < 0)
			continue;
		dhv = getSphericalCoordinateFromDepth(i/144 + 1, i%144 + 1, mTargetScan[i]);
		dhvs.push_back(dhv);
	}

	std::vector<double> mins, maxs;
	mins = getMin(dhvs);
	maxs = getMax(dhvs);

	double dist_1 = mins[0];
	double dist_2 = maxs[0];
	double fov_h_1 = mins[1];
	double fov_h_2 = maxs[1];
	double fov_v_1 = mins[2];
	double fov_v_2 = maxs[2];

	int binIdx;
	for(int i = 0; i < dhvs.size(); ++i)
	{
		binIdx = getBinIndex(dhvs[i][0], dhvs[i][1], dhvs[i][2], dist_1, dist_2, fov_h_1, fov_h_2, fov_v_1, fov_v_2, mIntervalD, mIntervalH, mIntervalV);
		mTargetScanCode[binIdx] = mTargetScanCode[binIdx] + 1;
	}
	normalize_L1(mTargetScanCode);

	for(int i = 0; i < mTargetScanCode.size(); ++i)
		std::cout << mTargetScanCode[i] << " ";
}

//! Output is in radians
std::vector<double> SemanticMap::getSphericalCoordinateFromDepth(int i, int j, double depth)
{
	double ImageWidth  = 144.0;
	double ImageHeight = 176.0;
	double FOV_H = 39.6 * M_PI / 180.0;
	double FOV_V = 47.5 * M_PI / 180.0;

	// Camera intrinsics to re-create perspective correctly
	double FocalLengthH    = ImageWidth / (2 * tan(FOV_H / 2));
	double FocalLengthV    = ImageHeight / (2 * tan(FOV_V / 2));
	double PrincipalPointH = ImageWidth/2.0;
	double PrincipalPointV = ImageHeight/2.0;


	double offset_h = j - PrincipalPointH;
	double delta_h;
	if(fabs(offset_h) < 0.00001) //actually it is 0 now
		delta_h = 0;
	else
		delta_h = fabs(offset_h)/offset_h * atan( fabs(offset_h / FocalLengthH) );

	// i's increasing direction is down, while the angle is increasing upwards
	double offset_v = i - PrincipalPointV;
	double delta_v;
	if(fabs(offset_v) < 0.00001)
		delta_v = 0;
	else
		delta_v = fabs(-offset_v)/(-offset_v) * atan( fabs(offset_v / FocalLengthV) );

	std::vector<double>result;
	result.push_back(depth);
	result.push_back(delta_h);
	result.push_back(delta_v);
	return result;
}

std::vector<double> SemanticMap::getMin(std::vector< std::vector<double> > data)
{
	std::vector<double> res;
	res.resize(3, pow(10.0,10.0));

	std::vector<double> tmp;

	for(size_t i = 0; i < data.size(); ++i)
	{
		tmp = data[i];
		if(tmp[0] < res[0])
			res[0] = tmp[0];
		if(tmp[1] < res[1])
			res[1] = tmp[1];
		if(tmp[2] < res[2])
			res[2] = tmp[2];
	}
	return res;
}

std::vector<double> SemanticMap::getMax(std::vector< std::vector<double> > data)
{
	std::vector<double> res;
	res.resize(3, -pow(10.0,10.0));

	std::vector<double> tmp;

	for(size_t i = 0; i < data.size(); ++i)
	{
		tmp = data[i];
		if(tmp[0] > res[0])
			res[0] = tmp[0];
		if(tmp[1] > res[1])
			res[1] = tmp[1];
		if(tmp[2] > res[2])
			res[2] = tmp[2];
	}
	return res;
}

void SemanticMap::normalize_L2(std::vector<double> &vec)
{
	double squareSum = 0, squareRoot;
	for(size_t i = 0; i < vec.size(); ++i)
	{
		squareSum += vec[i] * vec[i];
	}
	squareRoot = sqrt(squareSum);
	for(size_t i = 0; i < vec.size(); ++i)
	{
		vec[i] /= squareRoot;
	}
}

void SemanticMap::normalize_L1(std::vector<double> &vec)
{
	double sum = 0;
	for(size_t i = 0; i < vec.size(); ++i)
	{
		sum += vec[i];
	}
	for(size_t i = 0; i < vec.size(); ++i)
	{
		vec[i] /= sum;
	}
}

int SemanticMap::getBinIndex(double distance, double delta_h, double delta_v,
							 double dist_1, double dist_2, double fov_h_1, double fov_h_2, double fov_v_1, double fov_v_2,
							 int num_d, int num_h, int num_v)
{
	//% d part
	int d = (int)floor((distance - dist_1) * num_d / (dist_2 - dist_1));

	//% h part
	double fov_h = fov_h_2 - fov_h_1;
	int h = (int)floor((delta_h - fov_h_1) * num_h / fov_h);

	//%v part
	double fov_v = fov_v_2 - fov_v_1;
	int v = (int)floor((delta_v - fov_v_1) * num_v / fov_v);

	//% want to make sure they are not out of range
	if(d == num_d )
		d = num_d - 1;

	if(h == num_h)
		h = num_h - 1;

	if(v == num_v)
		v = num_v - 1;

	if(d < 0)
		d = 0;

	if(h < 0)
		h = 0;

	if(v < 0)
		v = 0;

	return d * num_h * num_v + h * num_v + v;

}

std::vector<compareItem> SemanticMap::getNN()
{
	mComparison.clear();
	for(size_t i = 0; i < mCodebook.size(); ++i)
	{
		compareItem item;
		item.sampleIdx = i;
		item.distance = computeDistance(mTargetScanCode, mCodebook[i]);
		mComparison.push_back(item);
	}

	std::sort(mComparison.begin(), mComparison.end(), shorterInDistance);
	return mComparison;
}

double SemanticMap::computeDistance(std::vector<double> item1, std::vector<double> item2)
{
	//just euclidean distance
	double sum = 0.0;
	for(size_t i = 0; i < item1.size(); ++i)
	{
		sum += pow(item1[i] - item2[i], 2.0);
	}
	return sqrt(sum);
}
//! return is in degrees
std::vector<double> SemanticMap::getLongitudeLatitude(int index, bool isHemisphere)
{
	std::vector<double> result;
	double DEGREE2RADIAN = M_PI / 180.0;

	if (mNumSamples == 8100)
	{
		isHemisphere = true;
		std::cout << "in hemisphere mode" << std::endl;
	}

	if(!isHemisphere)
	{
		result.push_back(index/90 * 2 * DEGREE2RADIAN); // every theta there are 90 scans, longitude, 0 - 360
		result.push_back( ((index%90) * 2 - 90) * DEGREE2RADIAN ); //latitude -90 - +90
	}
	else
	{
		result.push_back(index/45 * 2 * DEGREE2RADIAN); // every theta there are 90 scans, longitude, 0 - 360
		result.push_back( (index%45) * 2 * DEGREE2RADIAN ); //latitude -90 - +90
	}
	return result;
}

transf SemanticMap::getOriginalPose(double longitude, double latitude, vec3 cog, double back)
{
	transf pose;
	vec3 x = vec3(1,0,0);
	vec3 y =  vec3(0,1.0,0);
	vec3 z = vec3(0,0,1);

	//you definitely want to rotate along the local x,y axis since you are in the local system
	//consider the globle, we are always traveling along one big circle and the x,y axes should be perpendicular to each other
	transf Rot_y = transf(Quaternion(longitude,y), vec3::ZERO);
	transf Rot_x = transf(Quaternion(latitude,x), vec3::ZERO);
	transf Rot_z = transf(Quaternion(longitude,z), vec3::ZERO);
	transf backup = transf(Quaternion::IDENTITY,vec3(0,0,back));
	pose = backup * Rot_x * Rot_y * transf(Quaternion::IDENTITY, cog);
	return pose;

}

/*
This function will sample around the object according to the object system
not based on the hand system for the transformations
*/
transf SemanticMap::getOriginalPose2(double longitude, double latitude, vec3 cog, double back)
{
	transf pose;
	vec3 x = vec3(1,0,0);
	//vec3 y =  vec3(0,1.0,0);
	vec3 z = vec3(0,0,1);

	//you definitely want to rotate along the local x,y axis since you are in the local system
	//consider the globle, we are always traveling along one big circle and the x,y axes should be perpendicular to each other
	//transf Rot_y = transf(Quaternion(longitude,y), vec3::ZERO);
	transf Rot_x = transf(Quaternion(latitude,x), vec3::ZERO);
	transf Rot_z = transf(Quaternion(longitude,z), vec3::ZERO);
	transf backup = transf(Quaternion::IDENTITY,vec3(0,0,back));
	pose = backup * Rot_x * Rot_z * transf(Quaternion::IDENTITY, cog);
	//pose = backup * Rot_z * Rot_x * transf(Quaternion::IDENTITY, cog);
	return pose;

}

transf SemanticMap::getBestPoseInCurrentModel(int currentSampleIdx, int bestSampleIdx, vec3 cog)
{
	transf tmp;

	std::vector<double> originalLongitudeLatitude = getLongitudeLatitude(currentSampleIdx);
	transf moveBackToZeroPose = getOriginalPose(originalLongitudeLatitude[0], originalLongitudeLatitude[1], cog).inverse();

	std::vector<double> bestLongitudeLatitude = getLongitudeLatitude(bestSampleIdx);
	transf moveToBestPose = getOriginalPose(bestLongitudeLatitude[0], bestLongitudeLatitude[1], cog);

	tmp = moveBackToZeroPose * moveToBestPose;

	return tmp;
}

double SemanticMap::getSphericalDistance(double longitude1, double latitude1, double longitude2, double latitude2)
{
	//double haversine = pow(sin( (latitude1 - latitude2)/2 ),  2) + cos(latitude1) * cos(latitude2) * pow(sin( (longitude1 - longitude2)/2 ), 2);
	//double dist = 2 * atan2(sqrt(haversine), sqrt(1-haversine));

	//from mathworks http://mathworld.wolfram.com/GreatCircle.html
	double dist = acos( cos(latitude1) * cos(latitude2) * cos(longitude1 - longitude2) + sin(latitude1) * sin(latitude2));
	return dist;
}

compareItem SemanticMap::getVotingWinner(std::vector<compareItem> neighbors, int numNeighbors)
{
	int n = numNeighbors;
	std::vector<double> scores;
	scores.resize(n,0);

	std::vector<double> ll1, ll2;
	for(int i = 0; i < n; ++i)
	{
		for(int j = i + 1; j < n; ++j)
		{
			ll1 = getLongitudeLatitude(neighbors[i].sampleIdx);
			ll2 = getLongitudeLatitude(neighbors[j].sampleIdx);
			double DEGREE2RADIAN = M_PI / 180.0;
			double d = getSphericalDistance(ll1[0], ll1[1], ll2[0], ll2[1]);
			scores[i] += d;
			scores[j] += d;
		}
	}

	double smallestV = pow(10.0,10);
	int smallestI;
	for(int k = 0; k < n; ++k)
	{
		if(scores[k] < smallestV)
		{
			smallestV = scores[k];
			smallestI = k;
		}
	}
	return neighbors[smallestI];
}

//assume that the z-axis is the approaching direction, we are always rotate first around x axis (x-z) is the plane dividing latitude
//and then rotate along y axis rotating around y is changing longitude
std::vector<transf> SemanticMap::getApproachingDirections(vec3 cog, double stepLongitude, double stepLatitude, double stepRolling,
														  double longitudeStart, double longitudeEnd, double latitudeStart, double latitudeEnd, int version)
{
	std::vector<transf> ap;
	for(double longitude = longitudeStart; longitude < longitudeEnd; longitude += stepLongitude)
	{
		for(double latitude = latitudeStart; latitude < latitudeEnd; latitude += stepLatitude)
		{
			for(double roll = 0; roll < 2*M_PI-0.5*stepRolling; roll += stepRolling)
			{
				transf rollTransf(Quaternion(roll, vec3::Z), vec3::ZERO);
				if(version == 1)
				{
					ap.push_back( rollTransf * getOriginalPose(longitude, latitude, cog, -300) );
				}
				else if(version == 2)
				{
					ap.push_back( rollTransf * getOriginalPose2(longitude, latitude, cog, -300) );
				}
			}
		}
	}
	return ap;
}
