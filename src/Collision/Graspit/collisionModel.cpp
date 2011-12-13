//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: collisionModel.cpp,v 1.8 2009/07/21 22:38:04 cmatei Exp $
//
//######################################################################

#include "collisionModel.h"

#include <algorithm>

//#define GRASPITDBG
#include "debug.h"
#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

#include "collisionStructures.h"

namespace Collision {

const double Leaf::TOLERANCE = 1.0e-2;
const int Leaf::MAX_LEAF_TRIANGLES = 1;

void Jacobi(double a[3][3], double v[3][3]);
void print(const double a[3][3]);

position Leaf::getMeanVertex()
{
	position mean(0,0,0);
	if (mTriangles.empty()) return mean;

	std::list<Triangle>::iterator it;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		mean = mean + (*it).v1;
		mean = mean + (*it).v2;
		mean = mean + (*it).v3;
	}
	mean = ( 1.0 / ( 3.0 * (int)mTriangles.size() ) ) * mean;
	return mean;
}

double Leaf::getMedianProjection(const vec3 &axis)
{
	if (mTriangles.empty()) return 0.0;
	std::vector<double> projections;
	std::list<Triangle>::iterator it;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		position mean = (*it).v1 + (*it).v2 + (*it).v3;
		mean = (1.0 / 3.0) * mean;
		double projection = (mean - position::ORIGIN) % axis;
		projections.push_back(projection);
	}
	if (projections.size() == 1) {
		return projections.front();
	}
	std::nth_element(projections.begin(), projections.begin() + projections.size()/2, projections.end());
	return *(projections.begin() + projections.size()/2);
}

void boxSize(const position &p, vec3 &min, vec3 &max, 
			 const vec3 &x, const vec3 &y, const vec3 &z, double tolerance)
{
	vec3 d = p - position::ORIGIN;
	double dx = d % x;
	double dy = d % y;
	double dz = d % z;		
	if ( dx + tolerance > max.x() ) max.x() = dx + tolerance;
	if ( dy + tolerance > max.y() ) max.y() = dy + tolerance;
	if ( dz + tolerance > max.z() ) max.z() = dz + tolerance;
	if ( dx - tolerance < min.x() ) min.x() = dx - tolerance;
	if ( dy - tolerance < min.y() ) min.y() = dy - tolerance;
	if ( dz - tolerance < min.z() ) min.z() = dz - tolerance;
}

void Leaf::fitBox(const mat3 &R, vec3 &center, vec3 &halfSize)
{
	vec3 x = R.row(0);
	vec3 y = R.row(1);
	vec3 z = R.row(2);
	vec3 max(-1.0e10, -1.0e10, -1.0e10);
	vec3 min( 1.0e10,  1.0e10,  1.0e10);
	std::list<Triangle>::iterator it;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		boxSize( (*it).v1, min, max, x, y, z, TOLERANCE);
		boxSize( (*it).v2, min, max, x, y, z, TOLERANCE);
		boxSize( (*it).v3, min, max, x, y, z, TOLERANCE);
	}
	DBGP("Max: " << max);
	DBGP("Min: " << min);
	for (int i=0; i<3; i++) {
		halfSize[i] = 0.5 * (max[i] - min[i]);
	}
	DBGP("computed halfsize: " << halfSize);
	//halfSize = 0.5 * (max - min);
	center = min + halfSize;
	center = R.inverse() * center;
	//sanity check
	for (int i=0; i<3; i++) {
		if (halfSize[i] < TOLERANCE) {
			if (halfSize[i] < 0.5 * TOLERANCE) {
				DBGA("Warning: degenerate box computed");
			}
			halfSize[i] = TOLERANCE;
		}
	}
	DBGP("returned halfsize: " << halfSize);
}

/*! Formula taken from "Real Time Collision Detection" by C. Ericson */
void Leaf::areaWeightedCovarianceMatrix(double covMat[3][3])
{
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			covMat[i][j] = 0.0;
		}
	}
	double totalArea = 0;
	position areaWeightedMedian(0.0, 0.0, 0.0);
	std::list<Triangle>::iterator it;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		position m = it->centroid();
		double area = it->area();
		totalArea += area;
		areaWeightedMedian = areaWeightedMedian + area * m;
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				//column major storage
				covMat[i][j] += (area / 12.0) * (9*m[i]*m[j] + 
							 	 it->v1[i]*it->v1[j] + 
								 it->v2[i]*it->v2[j] +
								 it->v3[i]*it->v3[j] );
			}
		}
	}
	areaWeightedMedian = (1.0 / totalArea) * areaWeightedMedian;
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			covMat[i][j] = (1.0 / totalArea) * covMat[i][j] - 
							areaWeightedMedian[i]*areaWeightedMedian[j];
		}
	}
}

void Leaf::computeBboxOO()
{
	//compute the covariance matrix
	double covMat[3][3], v[3][3];
	areaWeightedCovarianceMatrix(covMat);
	DBGP("Cov mat:"); DBGST(print(covMat));

	//perform singular value decomposition
	Jacobi(covMat, v);
	DBGP("eigenvalues:"); DBGST(print(covMat));
	DBGP("eigenVectors:"); DBGST(print(v));
	int first = 0, second = 1, third = 2;
	if (covMat[1][1] > covMat[0][0]) {
		std::swap(first, second);
	}
	if (covMat[2][2] > covMat[first][first]) {
		std::swap(first, third);
	}
	if (covMat[third][third] > covMat[second][second]) {
		std::swap(second, third);
	}
	DBGP("Eigenvalues: " << covMat[first][first] << " " << covMat[second][second] 
		 << " "  << covMat[third][third]);

	//set up rotation matrix
	vec3 xAxis(v[0][first], v[1][first], v[2][first]); 
	vec3 yAxis(v[0][second], v[1][second], v[2][second]);
	vec3 zAxis = normalise(xAxis) * normalise(yAxis);
	yAxis = zAxis * normalise(xAxis);
	xAxis = yAxis * zAxis;
	mat3 R(xAxis, yAxis, zAxis);

	DBGP("Matrix: " << R);

	//compute bbox extents
	vec3 halfSize, center;
	fitBox(R, center, halfSize);

	//rotate box so that x axis always points in the direction of largest extent
	first = 0;
	if (halfSize.y() > halfSize.x()) first = 1;
	if (halfSize.z() > halfSize[first]) first = 2;
	transf rotate = transf::IDENTITY;
	if (first == 1) {
		// y has the largest extent, rotate around z
		rotate = rotate_transf(M_PI/2.0, vec3(0,0,1));
	} else if (first == 2) {
		// z has the largest extent, rotate around y
		rotate = rotate_transf(M_PI/2.0, vec3(0,1,0));
	}
	halfSize = halfSize * rotate;
	for (int i=0; i<3; i++) {
		if (halfSize[i] < 0) halfSize[i] = -halfSize[i];
	}
	mat3 RR;
	rotate.rotation().ToRotationMatrix(RR);
	R = RR * R;

	mBbox.halfSize = halfSize;
	mBbox.setTran( transf(R, center ) );
}

void Leaf::computeBboxAA()
{
	mat3 R( vec3::X, vec3::Y, vec3::Z);
	vec3 halfSize, center;
	fitBox(R, center, halfSize);
	mBbox.halfSize = halfSize;
	mBbox.setTran( transf(R, center ) );
}

/*! The split process goes through the followins stages:
	<ul>
	<li> if this leaf has a single triangle, return NULL
	<li> choose a split axis and point
	<li> divide the triangles between the children
	<li> if one child has no triangles, split again, this time at random
	<li> ask each child to compute its bounding box
	<li> create and return a new branch with the 2 new children
	</ul>
*/
Branch* Leaf::split()
{	
	//if we have few enough triangles, we are done
	if ( (int)mTriangles.size() <= MAX_LEAF_TRIANGLES ) return NULL;

	//choose axis as dominant axis of bbox
	//the box is rotated after fitting so that the x axis always points in the direction
	//of the largest extent
	vec3 xAxis = vec3(1,0,0) * mBbox.getTran();
	vec3 yAxis = vec3(0,1,0) * mBbox.getTran();
	vec3 zAxis = vec3(0,0,1) * mBbox.getTran();
	if (mBbox.halfSize.y() > mBbox.halfSize.x() + 1.0e-5) {
		DBGA("Unexpected bounding box dominant axis. Extents: ");
		DBGA(mBbox.halfSize.x() << " " << mBbox.halfSize.y() << " " << mBbox.halfSize.z());
		//assert(0);
		std::swap(xAxis, yAxis);
		yAxis = - yAxis;
	}
	if (mBbox.halfSize.z() > mBbox.halfSize.y() && 
		mBbox.halfSize.z() > mBbox.halfSize.x() + 1.0e-5 ) {
		DBGA("Unexpected bounding box dominant axis. Extents: ");
		DBGA(mBbox.halfSize.x() << " " << mBbox.halfSize.y() << " " << mBbox.halfSize.z());
		//assert(0);
		std::swap(xAxis, zAxis);
		zAxis = - zAxis;
	}


	//choose point as mean of triangle vertices
	double sepPoint = (getMeanVertex() - position::ORIGIN) % xAxis;

	//choose point as median of triangle centroids
	//double sepPoint = getMedianProjection(xAxis);

	//create children
	Leaf *child1 = new Leaf();
	Leaf *child2 = new Leaf();

	
	//split according to centroid
	balancedSplit(xAxis, sepPoint, child1, child2);

	//if unbalanced, try splitting based on median
	if (!child1->getNumTriangles() || !child2->getNumTriangles()) {
		child1->clearTriangles();
		child2->clearTriangles();
		sepPoint = getMedianProjection(xAxis);
		balancedSplit(xAxis, sepPoint, child1, child2);
	}
	

	//try optimal split
	//the dominant axis chosen earlier is always X
	//optimalSplit(xAxis, yAxis, zAxis, child1, child2);

	//if STILL unbalanced, split randomly
	if (!child1->getNumTriangles() || !child2->getNumTriangles()) {
		child1->clearTriangles();
		child2->clearTriangles();
		randomSplit(child1, child2);
	}

	//ask children to compute bboxes
	child1->computeBbox();
	child2->computeBbox();

	//create branch with new children
	Branch* b = new Branch(child1, child2, mBbox);
	return b;
}

/*! Splits triangles based on the location of their centroid. */
void Leaf::balancedSplit(vec3 axis, double sepPoint, Leaf *child1, Leaf *child2)
{
	std::list<Triangle>::iterator it;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		position median(0,0,0);
		median = median + (*it).v1;
		median = median + (*it).v2;
		median = median + (*it).v3;
		median =  ( 1.0 / 3.0 ) * median;
		vec3 m = median - position::ORIGIN;
		double d = m % axis;
		if ( d < sepPoint ) {
			child1->addTriangle(*it);
		} else {
			child2->addTriangle(*it);
		}
	}
}

/*! Assigns triangles to the two children randomly */
void Leaf::randomSplit(Leaf *child1, Leaf *child2)
{
	std::list<Triangle>::iterator it;
	bool one = true;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		if (one) {
			child1->addTriangle(*it);
		} else {
			child2->addTriangle(*it);
		}
		one = !one;
	}
}

bool 
compareProjections(const std::pair<Triangle, double> &p1, 
				   const std::pair<Triangle, double> &p2) {
	return p1.second < p2.second;
}

/*! The split is done along the given x axis, and the optimal child bounding boxes
	are aligned with the x,y,z axes. Of course, there is no guarantee that the 
	children will actually build their own bboxes along those axes; as they will
	do their own principal components they might end up with boxes aligned
	differently.
*/
void 
Leaf::optimalSplit(const vec3 &x, const vec3 &y, const vec3 &z, Leaf *child1, Leaf *child2)
{
	//sort triangles by centroid projections
	std::vector< std::pair<Triangle,double> > sortedTriangles;
	std::list<Triangle>::iterator it;
	for (it=mTriangles.begin(); it!=mTriangles.end(); it++) {
		position median = (*it).v1 + (*it).v2 + (*it).v3;
		median =  ( 1.0 / 3.0 ) * median;
		double projection = (median - position::ORIGIN) % x;
		sortedTriangles.push_back( std::pair<Triangle,double>(*it,projection) );
	}
	std::sort(sortedTriangles.begin(), sortedTriangles.end(), compareProjections);

	//compute bbox volumes going up
	std::vector<double> volumesUp;
	vec3 max(-1.0e10, -1.0e10, -1.0e10);
	vec3 min( 1.0e10,  1.0e10,  1.0e10);
	std::vector< std::pair<Triangle,double> >::iterator it2;
	for (it2 = sortedTriangles.begin(); it2!=sortedTriangles.end(); it2++) {
		boxSize((*it2).first.v1, min, max, x, y, z, Leaf::TOLERANCE);
		boxSize((*it2).first.v2, min, max, x, y, z, Leaf::TOLERANCE);
		boxSize((*it2).first.v3, min, max, x, y, z, Leaf::TOLERANCE);
		double volumeSq = (max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2]);
		volumesUp.push_back(volumeSq);
	}
	//and bbox volumes going down
	std::vector<double> volumesDown;
	max.set(-1.0e10, -1.0e10, -1.0e10);
	min.set( 1.0e10,  1.0e10,  1.0e10);
	std::vector< std::pair<Triangle,double> >::reverse_iterator it3;
	for (it3 = sortedTriangles.rbegin(); it3!=sortedTriangles.rend(); it3++) {
		boxSize((*it3).first.v1, min, max, x, y, z, Leaf::TOLERANCE);
		boxSize((*it3).first.v2, min, max, x, y, z, Leaf::TOLERANCE);
		boxSize((*it3).first.v3, min, max, x, y, z, Leaf::TOLERANCE);
		double volumeSq = (max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2]);
		volumesDown.push_back(volumeSq);
	}
	assert( volumesUp.size() == volumesDown.size() );

	//find optimal split point
	for (int i=0; i<(int)volumesUp.size(); i++) {
		volumesUp[i] += volumesDown[ volumesDown.size()-i-1 ];
	}
	std::vector<double>::iterator minVolIt = std::min_element(volumesUp.begin(), volumesUp.end());

	//assign triangles to children
	std::vector<double>::iterator it4;
	int index=0;
	for (it4 = volumesUp.begin(); it4 != minVolIt; it4++) {
		child1->addTriangle(sortedTriangles[index].first);
		index++;
	}
	for (it4 = minVolIt; it4 != volumesUp.end(); it4++) {
		child2->addTriangle(sortedTriangles[index].first);
		index++;
	}
}


void Node::getBVRecurse(int currentDepth, int desiredDepth, std::vector<BoundingBox> *bvs)
{
	if (currentDepth == desiredDepth || isLeaf() ) {
		bvs->push_back( BoundingBox(mBbox) );
		DBGP("BBox tran: " << mBbox.getTran());
		DBGP("BBox size: " << mBbox.halfSize);
		return;
	}
}

void Branch::getBVRecurse(int currentDepth, int desiredDepth, std::vector<BoundingBox> *bvs)
{
	Node::getBVRecurse(currentDepth, desiredDepth, bvs);
	if (currentDepth < desiredDepth) {
		mChild1->getBVRecurse(currentDepth+1, desiredDepth, bvs);
		mChild2->getBVRecurse(currentDepth+1, desiredDepth, bvs);
	}
}

/*! This model does not have its own bbox hierarchy, but instead uses the one
	of the \a original model. WARNING: the entire cloning system is not 
	completely finished. If the original is changed or deleted then the 
	clone is almost guaranteed to cause a crash!

	The clone can also be created (when using the constructor) in a different, 
	thread than the original, which is the way cloning is most usually applied.
*/
void CollisionModel::cloneModel(CollisionModel *original)
{
	if (mClone || original->mClone) {
		DBGA("WARNING: cloning of clones! Not well tested!"); 
	}
	delete mRoot;
	mRoot = original->mRoot;
	mClone = true;
}

void CollisionModel::getBoundingVolumes(int depth, std::vector<BoundingBox> *bvs)
{
	mRoot->getBVRecurse(0, depth, bvs);
}

/*! After all the triangles have been added, use build() to build
	the bb hierarchy. Use reset() to clear the bb hierarchy of an
	already built model.
*/
void 
CollisionModel::addTriangle(Triangle t) 
{
	if (mClone) {
		DBGA("Cannot add triangles to clones!");
		assert(0);
		return;
	} 
	if (!mRoot->isLeaf()) {
		DBGA("Reset model before adding triangles");
		return;
	}
	static_cast<Leaf*>(mRoot)->addTriangle(t);
}

/*! In the future, this should also have a version where all the triangles
	that used to be in the hierarchy are centralized back in the root. 
	Does not work for clones.
*/
void
CollisionModel::reset()
{
	if (mClone) {
		DBGA("Cannot reset a clone!");
		assert(0);
		return;
	} 
	delete mRoot; 
	mRoot = new Leaf();
}


void 
CollisionModel::build() 
{
	if (mClone) {
		DBGA("Cannot build a cloned model!");
		assert(0);
		return;
	}
	if (!mRoot->isLeaf()) {
		DBGA("Model already built. Reset first.");
		return;
	}
	DBGP("Buid bboxes from triangles: " << static_cast<Leaf*>(mRoot)->getNumTriangles());
	static_cast<Leaf*>(mRoot)->computeBbox();
	
	Branch *tmp = mRoot->split();
	if (tmp) {
		delete mRoot;
		mRoot = tmp;
		tmp->splitRecurse(0,-1);
	}
	DBGP("Split finished, hierarchy size: " << mRoot->countRecurse());		
}

//-------------------------------------

/* Some mathematical tools. I will try to find a better place for these, 
   maybe in matvec.cpp */

inline void multiply(double r[3][3], const double a[3][3], const double b[3][3]) {
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			r[i][j] = 0.0;
			for (int k=0; k<3; k++) {
				r[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

inline void copy(double r[3][3], const double o[3][3]) {
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			r[i][j] = o[i][j];
		}
	}
}

inline void transpose(double r[3][3], const double o[3][3]) {
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			r[i][j] = o[j][i];
		}
	}
}

void print(const double a[3][3])
{
	DBGA(a[0][0] << " " << a[0][1] << " " << a[0][2]);
	DBGA(a[1][0] << " " << a[1][1] << " " << a[1][2]);
	DBGA(a[2][0] << " " << a[2][1] << " " << a[2][2]);
}

// 2-by-2 Symmetric Schur decomposition. Given an n-by-n symmetric matrix
// and indicies p, q such that 1 <= p < q <= n, computes a sine-cosine pair
// (s, c) that will serve to form a Jacobi rotation matrix.
//
// See Golub, Van Loan, Matrix Computations, 3rd ed, p428
void SymSchur2(double a[3][3], int p, int q, double &c, double &s)
{
    if (fabs(a[p][q]) > 0.0001f) {
        double r = (a[q][q] - a[p][p]) / (2.0f * a[p][q]);
        double t;
        if (r >= 0.0f)
            t = 1.0f / (r + sqrt(1.0f + r*r));
        else
            t = -1.0f / (-r + sqrt(1.0f + r*r));
        c = 1.0f / sqrt(1.0f + t*t);
        s = t * c;
    } else {
        c = 1.0f;
        s = 0.0f;
    }
}

// Computes the eigenvectors and eigenvalues of the symmetric matrix A using
// the classic Jacobi method of iteratively updating A as A = J^T * A * J,
// where J = J(p, q, theta) is the Jacobi rotation matrix.
//
// On exit, v will contain the eigenvectors, and the diagonal elements
// of a are the corresponding eigenvalues.
//
// See Golub, Van Loan, Matrix Computations, 3rd ed, p428
void Jacobi(double a[3][3], double v[3][3])
{
    int i, j, n, p, q;
    double prevoff, c, s;
    double J[3][3], tmp1[3][3], tmp2[3][3];

    // Initialize v to identity matrix
    for (i = 0; i < 3; i++) {
        v[i][0] = v[i][1] = v[i][2] = 0.0f;
        v[i][i] = 1.0f;
    }

    // Repeat for some maximum number of iterations
    const int MAX_ITERATIONS = 50;
    for (n = 0; n < MAX_ITERATIONS; n++) {
        // Find largest off-diagonal absolute element a[p][q]
        p = 0; q = 1;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                if (i == j) continue;
                if (fabs(a[i][j]) > fabs(a[p][q])) {
                    p = i;
                    q = j;
                }
            }
        }

        // Compute the Jacobi rotation matrix J(p, q, theta)
        // (This code can be optimized for the three different cases of rotation)
        SymSchur2(a, p, q, c, s);
        for (i = 0; i < 3; i++) {
            J[i][0] = J[i][1] = J[i][2] = 0.0f;
            J[i][i] = 1.0f;
        }
        J[p][p] =  c; J[p][q] = s;
        J[q][p] = -s; J[q][q] = c;

        // Cumulate rotations into what will contain the eigenvectors
        // v = v * J;
		multiply(tmp1, v, J);
		copy(v, tmp1);

        // Make 'a' more diagonal, until just eigenvalues remain on diagonal
        // a = (J.Transpose() * a) * J;
		transpose(tmp1, J);
		multiply(tmp2, tmp1, a);
		multiply(tmp1, tmp2, J);
		copy(a, tmp1);
    
        // Compute "norm" of off-diagonal elements
        double off = 0.0f;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                if (i == j) continue;
                off += a[i][j] * a[i][j];
            }
        }
        /* off = sqrt(off); not needed for norm comparison */

        // Stop when norm no longer decreasing
        if (n > 2 && off >= prevoff)
            return;
        
        prevoff = off;
    }
}


}
