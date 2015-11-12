#include "environment.h"
//------------------Surface------------------//
TriangleSurface::~TriangleSurface()
{
	//delete the triangle
	delete mTriangle;
}

bool TriangleSurface::isOn(vec3 point)
{

	//check if the point is close to the plane
	//plane equation: ax + by + cz + d = 0
	double a, b, c, d;
	a = mTriangle->v1[1] * (mTriangle->v2[2] - mTriangle->v3[2]) + 
		mTriangle->v2[1] * (mTriangle->v3[2] - mTriangle->v1[2]) +
		mTriangle->v3[1] * (mTriangle->v1[2] - mTriangle->v2[2]);
	
	b = mTriangle->v1[2] * (mTriangle->v2[0] - mTriangle->v3[0]) + 
		mTriangle->v2[2] * (mTriangle->v3[0] - mTriangle->v1[0]) +
		mTriangle->v3[2] * (mTriangle->v1[0] - mTriangle->v2[0]);
	
	c = mTriangle->v1[0] * (mTriangle->v2[1] - mTriangle->v3[1]) + 
		mTriangle->v2[0] * (mTriangle->v3[1] - mTriangle->v1[1]) +
		mTriangle->v3[0] * (mTriangle->v1[1] - mTriangle->v2[1]);

	d = - mTriangle->v1[0] * (mTriangle->v2[1] * mTriangle->v3[2] - mTriangle->v3[1] * mTriangle->v2[2])
		- mTriangle->v2[0] * (mTriangle->v3[1] * mTriangle->v1[2] - mTriangle->v1[1] * mTriangle->v3[2])
		- mTriangle->v3[0] * (mTriangle->v1[1] * mTriangle->v2[2] - mTriangle->v2[1] * mTriangle->v1[2]);

	double dist;
	dist = fabs(a * point.x() + b * point.y() + c * point.z() + d) / 
		sqrt(a*a + b*b + c*c);

	if(dist > 1.0)
	{
		//std::cout << "too far from the plane" << std::endl;
		return false;
	}

	//now check the angles
	vec3 v01, v02, v03, vn;
	v01 = vec3(mTriangle->v1.x(), mTriangle->v1.y(), mTriangle->v1.z()) - point;
	v02 = vec3(mTriangle->v2.x(), mTriangle->v2.y(), mTriangle->v2.z()) - point;
	v03 = vec3(mTriangle->v3.x(), mTriangle->v3.y(), mTriangle->v3.z()) - point;
	vn = mTriangle->normal();

	if( (v01 * v02) % vn < 0 ||
		(v02 * v03) % vn < 0 ||
		(v03 * v01) % vn < 0 )
	{
		//std::cout << "point close to the plane, but lies outside the triangle" << std::endl;
		return false;
	}

	return true;
}

vec3 TriangleSurface::getNormal()
{
	return mTriangle->normal();
}

void TriangleSurface::printMe()
{
	std::cout << mTriangle->v1 << std::endl;
	std::cout << mTriangle->v2 << std::endl;
	std::cout << mTriangle->v3 << std::endl;
	std::cout << mTriangle->normal() << std::endl;
}

//------------------Environment--------------//
void Environment::setSurfaceListFromTriangleSet(std::vector<Triangle> t)
{
	mSurfaceList.resize(t.size());
	for(size_t i = 0; i < mSurfaceList.size(); ++i)
	{
		mSurfaceList[i] = new TriangleSurface;
		mSurfaceList[i]->setData( (new Triangle(t[i])) );
	}
}

Surface* Environment::findSurfaceHit(position contact)
{
	for(size_t i = 0; i < mSurfaceList.size(); ++i)
	{
		if(mSurfaceList[i]->isOn(vec3(contact.x(), contact.y(), contact.z())))
		{
			std::cout << "found hit surface: " << std::endl;
			mSurfaceList[i]->printMe();
		}
	}
	return NULL;
}