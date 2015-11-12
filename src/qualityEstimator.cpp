#include "qualityEstimator.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "svm.h"
#include <math.h>
double QualityEstimator::estimateStability()
{
	std::vector<double> encoded = encode();
	return runSVM(encoded);
}

void QualityEstimator::setModel(std::string modelFileName)
{
	if(mModel)
	{
		svm_free_and_destroy_model(&mModel);
		mModel = NULL;
	}
	mModel = svm_load_model(modelFileName.c_str());
	if(!mModel)
	{
		std::cout << "model load failed" << std::endl;
		return;
	}
	std::cout << "model load successfully" << std::endl;
}

double QualityEstimator::runSVM(const std::vector<double>& v)
{
	if(mNode)
	{
		free(mNode);
		mNode = NULL;
	}
	
	if(v.size() != mNumDim)
	{
		std::cout << "dimension of the estimate vector does not match the feature" << std::endl;
		return -1;
	}
	mNode = (struct svm_node *) malloc(sizeof(struct svm_node) * (mNumDim + 1));	// need one more for index = -1
	for(int i = 0; i < mNumDim; ++i)
	{
		//maybe I should blame libsvm for starting with 1 instead of 0
		//maybe not, but it is definitely an evil indexing difference
		//libsvm starts with 1, NOT 0
		mNode[i].index = i + 1;
		mNode[i].value = v[i];
		//std::cout << i << ": " << v[i] << std::endl;
	}
	mNode[mNumDim].index = -1;
	double label = svm_predict(mModel, mNode);
	free(mNode); mNode = NULL;
	svm_free_and_destroy_model(&mModel); mModel = NULL;
	return label;
}

std::vector<double> QualityEstimator::encode()
{
	//collect valid contacts according to tactile reading
	double epsilon = 0.00001;
	std::vector< std::vector<double> > validContacts;


#define CONSIDER_FORCE
#ifdef CONSIDER_FORCE
        //total force
        double fp=0, f1=0, f2=0, f3=0;
        for(size_t i = 0; i < mTactileReadings.size(); ++i)
	{
      
          if(mTactileReadings[i] > epsilon) //contact detected
          {
            if(i < 24)
            {
              fp += mTactileReadings[i];
            }
            else if(i >= 24 && i < 48)
            {
              f1 += mTactileReadings[i];
            }
            else if(i >= 48 && i < 72)
            {
              f2 += mTactileReadings[i];
            }
            else if(i >= 72 && i < 96)
            {
              f3 += mTactileReadings[i];
            }
          }
        }

        //compute encoded grasps for each pad
        std::vector<double> encoded;
        encoded.resize(mNumDim,0);
        
        for(size_t i = 0; i < mTactileReadings.size(); ++i)
        {
          if(mTactileReadings[i] > epsilon)
          {
            std::vector<double> distribute = computeDistribute(mTactileLocations[i]);
            double total_force;
            if(i < 24) //palm
            {
              total_force = fp;
            } 
            else if(i >= 24 && i < 48)
            {
              total_force = f1;
            }
            else if(i >= 48 && i < 72)
            {
              total_force = f2;
            }
            else if(i >= 72 && i < 96)
            {
              total_force = f3;
            }

            for(int j = 0; j < mNumDim; ++j)
            {
              encoded[j] += distribute[j] * mTactileReadings[i] / total_force;
            }
          }
        }

        int num_pads = (fp > epsilon ? 1 : 0) +
          (f1 > epsilon ? 1 : 0) + 
          (f2 > epsilon ? 1 : 0) + 
          (f3 > epsilon ? 1 : 0);

        for(int k = 0; k < mNumDim; ++k)
        {
          encoded[k] /= (double)num_pads;
        }

#endif

#ifdef EQUAL_WEIGHT

        /* every cell is treated equally*/
	for(size_t i = 0; i < mTactileReadings.size(); ++i)
	{
		if(mTactileReadings[i] > epsilon) //contact detected
			validContacts.push_back(mTactileLocations[i]);
	}

#endif

#ifdef ONE_PER_PAD
        /* every pad contributes only one contact modelb00 */
        for(size_t k = 0; k < 4; ++k)
        {
          double weightedLoc[3], force;
          weightedLoc[0] = weightedLoc[1] = weightedLoc[2] = 0;
          force = 0;
          for(size_t i = k * 24; i < 24 + k * 24; ++i)
          {
            if(mTactileReadings[i] > epsilon)
            {
              force += mTactileReadings[i];
              weightedLoc[0] += mTactileReadings[i] * mTactileLocations[i][0];
              weightedLoc[1] += mTactileReadings[i] * mTactileLocations[i][1];
              weightedLoc[2] += mTactileReadings[i] * mTactileLocations[i][2];
              //std::cout << "computing: " << mTactileLocations[i][0] << ", " << mTactileLocations[i][1] << ", " << mTactileLocations[i][2] << std::endl;
            }
          }
          if(force > epsilon)
          {
            std::vector<double> loc;
            loc.push_back(weightedLoc[0] / force);
            loc.push_back(weightedLoc[1] / force);
            loc.push_back(weightedLoc[2] / force);
            validContacts.push_back(loc);
            //std::cout << "contacts: " << weightedLoc[0]/force << ", " << weightedLoc[1] / force << ", " << weightedLoc[2] / force << std::endl;
          }
        }


	//compute feature vector
	std::vector<double> encoded;
	encoded.resize(mNumDim,0);
	for(size_t i = 0; i < validContacts.size(); ++i)
	{

          //std::cout << "contact: " << validContacts[i][0] << " , " << validContacts[i][1] << ", " << validContacts[i][2] << std::endl;
		//compute the distribution of current valid contact among contact centers
		std::vector<double> distribute = computeDistribute(validContacts[i]);

		//add it to the feature vector
		for(int j = 0; j < mNumDim; ++j)
		{
			encoded[j] += distribute[j];
		}
	}

        for(int k = 0; k < mNumDim; ++k)
        {
          encoded[k] /= (double)(validContacts.size());
        }

#endif
	//normalization
	// double sum = 0;
	// for(int k = 0; k < mNumDim; ++k)
	// {
	// 	sum += encoded[k];
	// }

	//print out the encoded vector
	
	//normalize the encoded vector
	for(int k = 0; k < mNumDim; ++k)
	{
		encoded[k] = encoded[k] / (mNormalizer[k]);
		//std::cout << "print out encoded vector: " << std::endl;
		//std::cout << encoded[k] << " ";
	}
	//std::cout << std::endl;
	

	return encoded;
}

bool printed = true;

std::vector<double> QualityEstimator::computeDistribute(const std::vector<double>& c)
{
	std::vector<double> distribute;
	distribute.resize(mNumDim,0);
	double sigma = 31.6456;
	for(int i = 0; i < mNumDim; ++i)
	{
		std::vector<double> center = mCodebook[i];
		double l2_dist = ((c[0] - center[0]) * (c[0] - center[0]) +
			(c[1] - center[1]) * (c[1] - center[1]) + 
			(c[2] - center[2]) * (c[2] - center[2]));
		distribute[i] = exp(- l2_dist/(sigma * sigma));
                if(!printed)
                  std::cout << center[0] << ", " << center[1] << ", " << center[2] << std::endl;
	}
        printed = true;
	return distribute;
}

void QualityEstimator::setCodebook(std::string codebookFileName)
{
	FILE* fp = fopen(codebookFileName.c_str(), "r");
	fscanf(fp, "%d\n", &mNumDim);
	mNormalizer.resize(mNumDim);
	for(int i = 0; i < mNumDim; ++i)
	{
		fscanf(fp, "%lf\n", &mNormalizer[i]);
		//std::cout << "normalizer: " << i << ": " << mNormalizer[i] << std::endl;
	}
	
	std::vector<double> contact;
	contact.resize(3);
	mCodebook.clear();
	for(int i = 0; i < mNumDim; ++i)
	{
		fscanf(fp, "%lf %lf %lf\n",&contact[0], &contact[1], &contact[2]);
		mCodebook.push_back(contact);
		//std::cout << "contact: " << contact[0] << " " << contact[1] << " " << contact[2] << std::endl;
	}
}

void QualityEstimator::printV(const std::vector<double>& v)
{
  std::cout << v.size() << ": " << std::endl;
	for(size_t i = 0; i < v.size(); ++i)
		std::cout << v[i] << " ";
	std::cout << std::endl;
}
