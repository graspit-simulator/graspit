#ifndef PCAFUNCTOR_H_
#define PCAFUNCTOR_H_

class meshHandler;
class pcaFunctor{
public:
	 virtual void operator() (meshHandler * m, meshHandler * zs, float * princomps) = 0;
};

#endif 
