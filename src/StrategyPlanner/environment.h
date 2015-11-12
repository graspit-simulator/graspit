#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <string>
#include "triangle.h"

class Surface
{
	/*
	A Surface is 
	*/

public:
	virtual bool isOn(vec3 point) = 0;
	virtual vec3 getNormal() = 0;
	virtual void setData(void* d) = 0;
	virtual void printMe() = 0;
};

class TriangleSurface : public Surface
{
private:
	Triangle* mTriangle;
public:
	TriangleSurface() : Surface () {}
	~TriangleSurface();
	virtual bool isOn(vec3 point);
	virtual vec3 getNormal();
	virtual void setData(void* d) { mTriangle = (Triangle*)d; }
	virtual void printMe();
};

class Environment
{
	/*
	Environment consists of a set of surfaces
	*/
private:
	std::vector<Surface*> mSurfaceList;

public:
	Environment(){}
	void setSurfaceListFromTriangleSet(std::vector<Triangle> t);
	Surface* findSurfaceHit(position contact);



};
#endif //_ENVIRONMENT_H_