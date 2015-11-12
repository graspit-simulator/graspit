#ifndef _QUALITY_ESTIMATOR_H_
#define _QUALITY_ESTIMATOR_H_
/*
This class accepts the 7 joint angles of a new barrett hand model.
It will shape the hand as specified by mJointAngles.
SVM will be run on the mTactileReadings.
*/

#include <vector>
#include <string>
class QualityEstimator{
private:
	struct svm_model * mModel;
	struct svm_node * mNode;

	//number of dimension of the feature space
	//should be the same as the size of the mCodebook
	int mNumDim;
	std::vector<double> mTactileReadings;
	std::vector< std::vector<double> > mTactileLocations;
	std::vector< std::vector<double> > mCodebook;
	std::vector<double> mNormalizer;

	//run SVM on the given feature vector
	double runSVM(const std::vector<double>& v);
	//encode the current test specified by mTactileReadings, mTactileLocations, mCodebook, and mNormalizer
	std::vector<double> encode();

	//returns the vector of distances between c and the contacts in the codebook
	std::vector<double> computeDistribute(const std::vector<double>& c);

	//Helper function to print a vector
	void printV(const std::vector<double>& v);
public:
	QualityEstimator() : mModel(NULL), mNode(NULL) {};
	void setModel(std::string modelFileName);
	void setCodebook(std::string codebookFileName);
	void setTactileReadings(const std::vector<double>& t){ mTactileReadings = t; printV(t);}
	void setTactileLocations(const std::vector< std::vector<double> >& l){ mTactileLocations = l;}
	
	double estimateStability();
};
#endif // _QUALITY_ESTIMATOR_H_