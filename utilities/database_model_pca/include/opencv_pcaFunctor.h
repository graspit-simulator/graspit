#ifndef OPENCV_PCAFUNCTOR_H_
#define OPENCV_PCAFUNCTOR_H_
#include "pcaFunctor.h"

class meshHandler;

class opencv_pcaFunctor:public pcaFunctor{
public:
	virtual void operator() (meshHandler * m, meshHandler * zs, float * princomps);
};

#endif 
