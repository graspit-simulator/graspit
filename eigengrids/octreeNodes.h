/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Matei Ciocarlie */

#ifndef _octreenodes_h_
#define _octreenodes_h_

#include <stdlib.h>
#include <list>
#include <assert.h>
#include <cmath>

#include "dataTypes.h"
#include "intersection_triangle.h"
#include "intersection_obb.h"
#include "intersection_sphere.h"

namespace scan_utils{

//! Constants that need to be chars to save space
namespace OctreeChildType {
const char NULL_CHILD = 0;
const char BRANCH = 1;
const char LEAF = 2;
}

/*!  A generic Octree node, could either be a leaf or a branch. Could
  be defined privately inside the Octree class, but in the future we
  might implement faster accessors that return the node that they
  accessed, so that subsequent calls can use this information.
*/
template <typename T>
class OctreeNode {
 private:
 public:
	OctreeNode(){}
	virtual ~OctreeNode(){}
	virtual bool isLeaf() const = 0;
	virtual void serialize(char*, unsigned int&) const {}
	virtual bool deserialize(char*, unsigned int&, unsigned int){return true;}
	virtual int computeMaxDepth() const {return 0;}
	virtual void recursiveAggregation(){}

};


/*! This structure holds a pointer to a node as well as the spatial
    coordinates that the node is responsible for. It is designed to
    facilitate navigation through the Octree for intersection tests
    without using recursion. Traversal functions can build their own
    stack of this structure instead of using recursion, where the
    system would provide this for them. */
template <typename T>
struct SpatialNode {
	OctreeNode<T> *node;
	float cx, cy, cz;
	float dx, dy, dz;
};

/*! A leaf simply holds a value and nothing else. Do not use a leaf to
    store the empty value, use a NULL pointer in its parent instead.
 */
template <typename T>
class OctreeLeaf : public OctreeNode<T> {
 private:
	T mValue;
 public:
	OctreeLeaf(T val) {setVal(val);}
	OctreeLeaf(){}
	virtual ~OctreeLeaf(){}
	bool isLeaf() const {return true;}

	T getVal() const {return mValue;}
	void setVal(T val){mValue = val;}
	// Serializes the content of this leaf
	virtual void serialize(char *destinationString, unsigned int &address) const;
	// Reads in the content of this leaf
	virtual bool deserialize(char *sourceString, unsigned int &address, 
							 unsigned int size);
	//! Returns 0
	virtual int computeMaxDepth() const {return 0;}
};

/*! An Octree branch. Always contains exactly 8 children pointers. A
    NULL child pointer means that the respective child points to an
    unexplored region of space and thus is equivalent to having a
    child with the mEmptyValue of the Octree set.
 */
template <typename T>
class OctreeBranch : public OctreeNode<T> {
 private:
	OctreeNode<T> **mChildren;

	//! Returns true if the given child is a leaf and it needs to be triangulated given the required values
	bool triangulateChild(unsigned char address, bool(*testFunc)(T), T emptyValue) const;
	//! Actually creates the triangles that enclose the given box
	void createTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
			     float cx, float cy, float cz,
			     float dx, float dy, float dz,
			     std::list<Triangle> &triangles) const;
 public:
	bool isLeaf() const {return false;}

	//! Initializes a branch with all NULL (unexplored) children
	inline OctreeBranch();
	//! Initializes a branch with all children set to the value \a val 
	inline OctreeBranch(T val);
	//! Destructor will delete all children first. Thus, delete an Octree top-down by just deleting its root.
	inline virtual ~OctreeBranch();

	//! Return the child at address \a adress, between 0 and 7
	OctreeNode<T>* getChild(unsigned char address) { return mChildren[address]; }
	//! Const version of the above
	const OctreeNode<T>* getChild(unsigned char address) const { return mChildren[address]; }

	//! Returns a structure that contains both a pointer to the child and its spatial coordinates
	inline SpatialNode<T>* getSpatialChild(int address, 
					      float cx, float cy, float cz,
					      float dx, float dy, float dz);

	//! Sets the child at address \a adress to point at \a child. 
	/*! If a child was already present at that address it is deleted.*/
	inline void setChild(unsigned char address, OctreeNode<T> *child);
	//! Replaces \a oldChild with \a newChild. \a oldChild is also deleted.
	inline void replaceChild(OctreeNode<T> *oldChild, OctreeNode<T> *newChild);
	//! Recursively returns the total number of branches below this one (including this one)
	int getNumBranches() const;
	//! Recursively returns the number of leaves between this one
	int getNumLeaves() const;
	//! Recursively counts how many CELLS below hold a given value
	int cellCount(T value, T emptyVal, int height) const;

	//! Recursively goes down the tree and creates triangles
	void getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
			  float cx, float cy, float cz,
			  float dx, float dy, float dz,
			  std::list<Triangle> &triangles, bool(*testFunc)(T), T emptyValue) const;

	//! Recursively serializes everything below this branch
	virtual void serialize(char *destinationString, unsigned int &address) const;
	//! Recursively reads in everything below this branch
	virtual bool deserialize(char *sourceString, unsigned int &address, unsigned int size);
	//! Recursively computes the max depth under this branch
	virtual int computeMaxDepth() const;

	//! Checks if all children of this branch are indentical
	inline bool aggregate(OctreeLeaf<T> **newLeaf) const;
	//! Recursively checks the entire subtree under this branch for possible aggregations
	virtual void recursiveAggregation();
};

//------------------------------------ Constructors and destructors -------------------------

template <typename T>
OctreeBranch<T>::OctreeBranch()
{
	mChildren = new OctreeNode<T>*[8];
	for (int i=0; i<8; i++) {
		mChildren[i] = NULL;
	}
}	

template <typename T>
OctreeBranch<T>::OctreeBranch(T val)
{
	mChildren = new OctreeNode<T>*[8];
	for (int i=0; i<8; i++) {
		mChildren[i] = new OctreeLeaf<T>(val);
	}
}	

template <typename T>
OctreeBranch<T>::~OctreeBranch()
{
	for(int i=0; i<8; i++) {
		if (mChildren[i]) delete mChildren[i];
	}
	delete mChildren;
}

//------------------------------------- Navigation ------------------------------------------
 
template <typename T>
void OctreeBranch<T>::setChild(unsigned char address, OctreeNode<T> *child) 
{
	if (mChildren[address]) delete mChildren[address];
	mChildren[address] = child; 
}

template <typename T>
void OctreeBranch<T>::replaceChild(OctreeNode<T> *oldChild, OctreeNode<T> *newChild)
{
	for (unsigned char i=0; i<8; i++) {
		if (mChildren[i]==oldChild) {
			setChild(i, newChild);
			return;
		}
	}
}

/*!  If all children of this leaf are identical, returns true and
  places in \a newLeaf a pointer to a new leaf that can replace this
  branch.

  Otherwise returns false.
 */
template <typename T>
bool OctreeBranch<T>::aggregate(OctreeLeaf<T> **newLeaf) const
{
	if (!mChildren[0]) {
		//first child is NULL
		for (int i=1; i<8; i++) {
			//some children are NULL, some not. We are done
			if (mChildren[i]) return false;
		}
		//all children are NULL
		*newLeaf = NULL;
		return true;
	}

	//if any child is a branch, return false
	if (!mChildren[0]->isLeaf()) return false;

	T val = ((OctreeLeaf<T>*)(mChildren[0]))->getVal();

	for (int i=1; i<8; i++) {
		//some children are NULL, some not. We are done
		if (!mChildren[i]) return false;
		//if any child is a branch, return false
		if (!mChildren[i]->isLeaf()) return false;
		//two children leaves have different values
		if ( ((OctreeLeaf<T>*)(mChildren[i]))->getVal()!= val) return false;
	}
	//all children are leaves and they have the same value
	*newLeaf = new OctreeLeaf<T>(val);
	return true;
}

template <typename T>
void OctreeBranch<T>::recursiveAggregation()
{
	//aggregation must be performed bottom-up so go down first
	for (int i=0; i<8; i++) {
		if (mChildren[i]) mChildren[i]->recursiveAggregation();
	}

	OctreeLeaf<T> *newLeaf;
	for (unsigned char i=0; i<8; i++) {
		if (!mChildren[i] || mChildren[i]->isLeaf()) continue;
		if ( ((OctreeBranch<T>*)mChildren[i])->aggregate(&newLeaf) ) {
			setChild(i,newLeaf);
		}
	}
}

/*! Returns a structure that contains both a pointer to the child and
    its spatial coordinates
 */
template <typename T>
SpatialNode<T>* OctreeBranch<T>::getSpatialChild(int address, 
					      float cx, float cy, float cz,
					      float dx, float dy, float dz)
{
	SpatialNode<T> *sn = new SpatialNode<T>;
	sn->node = getChild(address);

	sn->dx=dx/2.0; sn->dy=dy/2.0; sn->dz=dz/2.0;

	if (address/4 == 0) sn->cx = cx-sn->dx/2.0;
	else sn->cx = cx+sn->dx/2.0;
	
	if ( (address%4)/2 == 0 ) sn->cy = cy-sn->dy/2.0;
	else sn->cy = cy+sn->dy/2.0;       
	
	if ( (address%4)%2 == 0 ) sn->cz = cz-sn->dz/2.0;
	else sn->cz = cz+sn->dz/2.0;

	return sn;
}

//-------------------------------------- Statistics -----------------------------------------

template <typename T>
int OctreeBranch<T>::getNumBranches() const
{
	int n=1;
	for (int i=0; i<8; i++) {
		if (mChildren[i] && !mChildren[i]->isLeaf()) {
			n += ((OctreeBranch*)mChildren[i])->getNumBranches();
		}
	}
	return n;
}

template <typename T>
int OctreeBranch<T>::getNumLeaves() const
{
	int n=0;
	for (int i=0; i<8; i++) {
		if ( !mChildren[i] ) continue;
		if ( mChildren[i]->isLeaf()) n += 1;
		else n += ((OctreeBranch*)mChildren[i])->getNumLeaves();
	}
	return n;
}

template <typename T>
int OctreeBranch<T>::computeMaxDepth() const
{
	int maxDepth = 0;
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) continue;
		int tmp = mChildren[i]->computeMaxDepth();
		if (tmp > maxDepth) maxDepth = tmp;
	}
	return maxDepth + 1;
}

/*!  Note that a LEAF at height h holding a given value is equivalent
  to 2^h cells! That's because cells are the smallest possible leafes,
  when many cells have the same value the Octree, by its nature,
  stores them in a single leaf etc.
 */
template <typename T>
int OctreeBranch<T>::cellCount(T value, T emptyVal, int height) const
{
	assert(height > 0);
	int n=0;
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) {
			if (value == emptyVal) {
				n += (int)pow((float)8, height-1);
			}
		}
		else if (mChildren[i]->isLeaf()) {
			if ( ((OctreeLeaf<T>*)mChildren[i])->getVal() == value) {
				n += (int)pow((float)8, height-1);
			}
		} else {
			n += ((OctreeBranch<T>*)mChildren[i])->cellCount(value, emptyVal, height-1);
		}
	}
	return n;
}

//----------------------------------------- Serialization -----------------------------------

template <typename T>
void OctreeBranch<T>::serialize(char *destinationString, unsigned int &address) const
{
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) {
			destinationString[address] = OctreeChildType::NULL_CHILD;
			address++;
			continue;
		}
		if (mChildren[i]->isLeaf()) {
			destinationString[address] = OctreeChildType::LEAF;
		} else {
			destinationString[address] = OctreeChildType::BRANCH;
		}
		address++;
		mChildren[i]->serialize(destinationString, address);
	}
}

template <typename T>
bool OctreeBranch<T>::deserialize(char *sourceString, unsigned int &address, unsigned int size)
{
	for (int i=0; i<8; i++) {
		if (address >= size) return false;
		if (sourceString[address] == OctreeChildType::NULL_CHILD) {
			setChild(i,NULL);
			address++;
			continue;
		}
		if (sourceString[address] == OctreeChildType::LEAF) {
			setChild(i, new OctreeLeaf<T>() );

		} else if (sourceString[address] == OctreeChildType::BRANCH) {
			setChild(i, new OctreeBranch<T>() );
		} else {
			//error
			address = size;
			return false;
		}
		address++;
		if (!mChildren[i]->deserialize(sourceString, address, size)) {
			//error
			address = size;
			return false;
		}
	}
	return true;

}

template <typename T>
void OctreeLeaf<T>::serialize(char *destinationString, unsigned int &address) const
{
	memcpy(&destinationString[address], &mValue, sizeof(mValue));
	address += sizeof(mValue);
}

template <typename T>
bool OctreeLeaf<T>::deserialize(char *sourceString, unsigned int &address, unsigned int size)
{
	if (address + sizeof(mValue) > size) {
		address = size;
		return false;
	}
	memcpy(&mValue, &sourceString[address], sizeof(mValue));
	address += sizeof(mValue);
	return true;
}

//--------------------------------------------- Intersection tests ---------------------------

template <typename T>
inline bool nodeTriangleIntersection(const SpatialNode<T> &sn, 
				     float trivert0[3], float trivert1[3], float trivert2[3])
{
	float boxcenter[3];
	float boxhalfsize[3];

	boxcenter[0] = sn.cx; boxcenter[1] = sn.cy; boxcenter[2] = sn.cz;
	boxhalfsize[0] = sn.dx/2.0; 
	boxhalfsize[1] = sn.dy/2.0;
	boxhalfsize[2] = sn.dz/2.0;

	return triBoxOverlap(boxcenter, boxhalfsize,
			     trivert0, trivert1, trivert2);
}

template <typename T>
inline bool nodeBoxIntersection(const SpatialNode<T> &sn, 
				const float *center, const float *extents, const float axes[][3])
{
	float boxcenter[3];
	float boxhalfsize[3];

	boxcenter[0] = sn.cx; boxcenter[1] = sn.cy; boxcenter[2] = sn.cz;
	boxhalfsize[0] = sn.dx/2.0; 
	boxhalfsize[1] = sn.dy/2.0;
	boxhalfsize[2] = sn.dz/2.0;

	return boxIntersectionTest( boxcenter, boxhalfsize, NULL,
				    center, extents, axes );
}

template <typename T>
inline bool nodeSphereIntersection(const SpatialNode<T> &sn, 
				   const float *center, float radius)
{
	float boxcenter[3];
	float boxhalfsize[3];

	boxcenter[0] = sn.cx; boxcenter[1] = sn.cy; boxcenter[2] = sn.cz;
	boxhalfsize[0] = sn.dx/2.0; 
	boxhalfsize[1] = sn.dy/2.0;
	boxhalfsize[2] = sn.dz/2.0;

	return boxSphereTest( boxcenter, boxhalfsize, center, radius);
}

//--------------------------------------------- Triangulation --------------------------------

template <typename T>
bool OctreeBranch<T>::triangulateChild(unsigned char address, bool(*testFunc)(T), T emptyValue) const
{
	T val;
	if (!mChildren[address]) {
		val = emptyValue;
	} else if ( mChildren[address]->isLeaf() ) {
		val = ((OctreeLeaf<T>*)(mChildren[address]))->getVal();
	} else {
		return false;
	}
	if (!testFunc) {
		return val != emptyValue;
	}
	return testFunc(val);
}

/*!  \param value - if this is NULL, the fctn returns the triangles
  from all non-empty leaves. Otherwise, it only returns the triangles
  from the leaves that hold the given value.
 */
template <typename T>
void OctreeBranch<T>::getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
				   float cx, float cy, float cz,
				   float dx, float dy, float dz,
				   std::list<Triangle> &triangles, bool(*testFunc)(T), T emptyValue) const
{
	dx/=2.0; dy/=2.0; dz/=2.0;
	
	float nextx, nexty, nextz;
	bool nextpx, nextnx, nextpy, nextny, nextpz, nextnz;
	
	for (int i=0; i<8; i++) {
		
		if (!mChildren[i] || mChildren[i]->isLeaf()) {
			if (!triangulateChild(i, testFunc, emptyValue)) continue;
		}

		if (i/4 == 0) {
			nextx = cx-dx;
			nextnx = nx;
			if ( triangulateChild(i+4, testFunc, emptyValue) ) nextpx = false;
			else nextpx = true;
		} else {
			nextx = cx+dx;
			nextpx = px;
			if ( triangulateChild(i-4, testFunc, emptyValue) ) nextnx = false;
			else nextnx = true;			
		}

		if ( (i%4)/2 == 0 ) {
			nexty = cy-dy;
			nextny = ny;
			if ( triangulateChild(i+2, testFunc, emptyValue) ) nextpy = false;
			else nextpy = true;			
		} else {
			nexty = cy+dy;
			nextpy = py;
			if ( triangulateChild(i-2, testFunc, emptyValue) ) nextny = false;
			else nextny = true;			
		}

		if ( (i%4)%2 == 0 ) {
			nextz = cz-dz;
			nextnz = nz;
			if ( triangulateChild(i+1, testFunc, emptyValue) ) nextpz = false;
			else nextpz = true;			
		} else {
			nextz = cz+dz;
			nextpz = pz;
			if ( triangulateChild(i-1, testFunc, emptyValue) ) nextnz = false;
			else nextnz = true;			
		}

		if (!mChildren[i] || mChildren[i]->isLeaf()) {
			createTriangles( nextpx, nextnx, nextpy, nextny, nextpz, nextnz, 
					 nextx, nexty, nextz, dx, dy, dz, triangles);
		} else {
			((OctreeBranch<T>*)(mChildren[i]))->getTriangles( nextpx, nextnx, nextpy, nextny, nextpz, nextnz, 
									  nextx, nexty, nextz, dx, dy, dz, 
									  triangles, testFunc, emptyValue);
		}
	}
 }

template <typename T>
void OctreeBranch<T>::createTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
				      float cx, float cy, float cz,
				      float dx, float dy, float dz,
				      std::list<Triangle> &triangles) const
{
	if (px) {
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz,
					       cx+dx, cy-dy, cz-dz,
					       cx+dx, cy+dy, cz-dz ) );
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz,
					       cx+dx, cy-dy, cz+dz,
					       cx+dx, cy-dy, cz-dz ) );
	}
	if (nx) {
		triangles.push_back( Triangle( cx-dx, cy+dy, cz+dz,
					       cx-dx, cy+dy, cz-dz,
					       cx-dx, cy-dy, cz-dz ) );
		triangles.push_back( Triangle( cx-dx, cy+dy, cz+dz,
					       cx-dx, cy-dy, cz-dz,
					       cx-dx, cy-dy, cz+dz ) );
	}

	if (py) {
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz, 
					       cx+dx, cy+dy, cz-dz,
					       cx-dx, cy+dy, cz-dz ) );
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz, 
					       cx-dx, cy+dy, cz-dz,
					       cx-dx, cy+dy, cz+dz ) );
	}
	if (ny) {
		triangles.push_back( Triangle( cx+dx, cy-dy, cz+dz, 
					       cx-dx, cy-dy, cz-dz,
					       cx+dx, cy-dy, cz-dz ) );
		triangles.push_back( Triangle( cx+dx, cy-dy, cz+dz, 
					       cx-dx, cy-dy, cz+dz,
					       cx-dx, cy-dy, cz-dz ) );
	}

	if(pz) {
		triangles.push_back( Triangle( cx-dx, cy-dy, cz+dz,
					       cx+dx, cy-dy, cz+dz,
					       cx+dx, cy+dy, cz+dz ) );
		triangles.push_back( Triangle( cx-dx, cy-dy, cz+dz,
					       cx+dx, cy+dy, cz+dz,
					       cx-dx, cy+dy, cz+dz ) );
	}
	if(nz) {
		triangles.push_back( Triangle( cx-dx, cy-dy, cz-dz,
					       cx+dx, cy+dy, cz-dz,
					       cx+dx, cy-dy, cz-dz ) );
		triangles.push_back( Triangle( cx-dx, cy-dy, cz-dz,
					       cx-dx, cy+dy, cz-dz,
					       cx+dx, cy+dy, cz-dz ) );
	}
}

} //namespace scan_utils


#endif
