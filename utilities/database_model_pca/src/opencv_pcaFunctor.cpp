#include "opencv_pcaFunctor.h"
#include "meshHandler.h"
#include <opencv/cv.h>

void opencv_pcaFunctor::operator()(meshHandler * m, meshHandler * zs, float * princomps){
  float covar[9];
  float rd[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; float evd[3]  = {0.0,0.0,0.0};
  m->zScores(zs);
  CvMat cvCovar = cvMat(3,3,CV_32F, covar), cvEigen = cvMat(3,3,CV_32F, princomps),
    eigVals = cvMat(3,1,CV_32F,evd), right = cvMat(3,3,CV_32F,rd); 
  zs->covarianceMatrix(covar);
  cvSVD(&cvCovar, &eigVals, &cvEigen, &right,CV_SVD_MODIFY_A );
}
