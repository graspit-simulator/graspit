#ifndef _UTILS_H_
#define _UTILS_H_
#include "matrix.h"
#include "matvec3D.h"
#include "TX60L.h"
#include "include/soapStub.h"

transf matrix2Transf(Matrix m)
{
	double data[9];
	
	data[0] = m.elem(0,0);
	data[1] = m.elem(0,1);
	data[2] = m.elem(0,2);

	data[3] = m.elem(1,0);
	data[4] = m.elem(1,1);
	data[5] = m.elem(1,2);

	data[6] = m.elem(2,0);
	data[7] = m.elem(2,1);
	data[8] = m.elem(2,2);

	mat3 mat(data);

	vec3 v(m.elem(0,3), m.elem(1,3), m.elem(2,3));
	transf t(mat,v);
	return t;
}

Matrix transf2Matrix(transf t)
{
	Matrix m(4,4);
	return m;
}

std::vector<double> quaternion2StaubliXYZ(Quaternion q)
{
	std::vector<double> xyz;
	mat3 m;
	//forwardKinematics.rotation().ToRotationMatrix(m);
	q.ToRotationMatrix(m);
	ns6__Frame * frame = new ns6__Frame();

	frame->nx = m.element(0,0);
	frame->ny = m.element(0,1);
	frame->nz = m.element(0,2);

	frame->ox = m.element(1,0);
	frame->oy = m.element(1,1);
	frame->oz = m.element(1,1);

	frame->ax = m.element(2,0);
	frame->ay = m.element(2,1);
	frame->az = m.element(2,2);

	double rx, ry, rz;
	TX60L mStaubliDummy;
	mStaubliDummy.GetRxRyRzCoord(frame, &rx, &ry, &rz);
	delete frame;

	xyz.push_back(rx);
	xyz.push_back(ry);
	xyz.push_back(rz);

	return xyz;
}

Quaternion staubliXYZ2Quaternion(std::vector<double> xyz)
{
	Quaternion q = Quaternion::IDENTITY;
	return q;
}

//returns a vector of 4 elements, first three specify the rotation axis
//the last one specifies the rotation angle
std::vector<double> rotationBetween2Vec(vec3 v1, vec3 v2)
{
	double angle = acos(v1%v2);
	vec3 axis = v1 * v2;
	axis = axis / axis.len();
	std::vector<double> r;
	r.push_back(axis.x());
	r.push_back(axis.y());
	r.push_back(axis.z());
	r.push_back(angle);
	return r;
}
#endif