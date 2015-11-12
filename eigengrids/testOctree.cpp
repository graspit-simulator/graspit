#include "octree.h"
#include <gtest/gtest.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fstream>
#include <scan_utils/OctreeMsg.h>
#include <dataTypes.h>
#include <list>

/*! \file Since the Octree is templated, it is completely contained in
  header files. As a result it is no longer compiled into the
  scan_utils .so. This file creates an executable that is just meant
  to make the library actually compiles the Octree header files to make
  sure they work. Can be considered a (fairly simple) unit test for
  the octree class.
*/

TEST(OctreeTests, constructionDeconstruction)
{
	scan_utils::Octree<int> *octree = new scan_utils::Octree<int>(0,0,0, 1,1,1, 1, 0);
	delete octree;
}

TEST(OctreeTests, insertionExtraction)
{
	scan_utils::Octree<float> *octree = new scan_utils::Octree<float>(0,0,0, 1.0,2.0,2.0, 5, 0.2);
	srand( (unsigned)time(NULL) );
	bool pass = true;
	float compVal;
	for (int i=0; i<100; i++) {
		float x,y,z;
		x = ((float)rand()) / (RAND_MAX + 1);
		y = ((float)rand()) / (RAND_MAX + 1);
		z = ((float)rand()) / (RAND_MAX + 1);
		if (!octree->testBounds(x,y,z)) {
			compVal = 0.2;
		} else {
			compVal = 1.7;
		}
		octree->insert(x, y, z, 1.7);

		float r = octree->get(x,y,z);
		if ( fabs(r - compVal) > 1.0e-6 ) {
			pass = false;
		}
	}
	EXPECT_TRUE(pass);

	const scan_utils::Octree<float> *const_octree = octree;
	//const_octree->insert(0,0,0, 1.9); //this should not compile
	const_octree->get(0,0,0); //this should compile
	delete octree;
}

TEST(OctreeTests, insertionDeletion)
{
	scan_utils::Octree<float> *octree = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, 5, 0.2);
	srand( (unsigned)time(NULL) );
	for (int i=0; i<100; i++) {
		float x,y,z;
		x = ((float)rand()) / (RAND_MAX ) - 0.5;
		y = ((float)rand()) / (RAND_MAX ) - 0.5;
		z = ((float)rand()) / (RAND_MAX ) - 0.5;
		if (!octree->testBounds(x,y,z)) {
			continue;
		}

		octree->insert(x, y, z, 1.7);
		EXPECT_TRUE( octree->getNumLeaves() > 0);
		EXPECT_TRUE( octree->getNumBranches() > 1);
		//fprintf(stderr,"Full: %d %d\n",octree->getNumLeaves(), octree->getNumBranches());

		octree->erase(x,y,z);
		EXPECT_TRUE( octree->getNumLeaves() == 0);
		EXPECT_TRUE( octree->getNumBranches() == 1);
		//fprintf(stderr,"Empty: %d %d\n",octree->getNumLeaves(), octree->getNumBranches());
	}
	delete octree;
}

TEST(OctreeTests, cellAccessors)
{
	int depth = 4;
	scan_utils::Octree<float> *original = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, depth, (float)0.2);
	scan_utils::Octree<float> *copy = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, depth, (float)0.2);
	int numCells = original->getNumCells();
	float cellSize = 1.0 / numCells;
	srand( (unsigned)time(NULL) );

	for (int i=0; i<1000; i++) {
		float x,y,z;
		x = ((float)rand()) / (RAND_MAX ) - 0.5;
		y = ((float)rand()) / (RAND_MAX ) - 0.5;
		z = ((float)rand()) / (RAND_MAX ) - 0.5;
		original->insert(x, y, z, ((float)rand())/RAND_MAX);
	}
	
	/*
	fprintf(stderr,"Original %d leaves and %d branches (before deletions)\n",
		original->getNumLeaves(), original->getNumBranches() );
	*/
	for (int i=0; i<100; i++) {
		float x,y,z;
		x = ((float)rand()) / (RAND_MAX ) - 0.5;
		y = ((float)rand()) / (RAND_MAX ) - 0.5;
		z = ((float)rand()) / (RAND_MAX ) - 0.5;
		original->erase(x, y, z);
	}

	for (int i=0; i<numCells; i++) {
		for (int j=0; j<numCells; j++) {
			for (int k=0; k<numCells; k++) {
				float val = original->cellGet(i,j,k);
				if (val != (float)0.2) copy->cellInsert( i, j, k, val );
			}
		}
	}

	EXPECT_TRUE( original->getNumLeaves() == copy->getNumLeaves() );
	EXPECT_TRUE( original->getNumBranches() == copy->getNumBranches() );

	//fprintf(stderr,"Original %d leaves and %d branches\n",original->getNumLeaves(), original->getNumBranches() );
	//fprintf(stderr,"Copy     %d leaves and %d branches\n",copy->getNumLeaves(), copy->getNumBranches() );

	bool pass = true;
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				float c = copy->get(i1-0.5, i2-0.5, i3-0.5);
				float o = original->get(i1-0.5, i2-0.5, i3-0.5);
				if ( c!= o) {
					pass = false;
					//fprintf(stderr,"Original %f and copy %f \n",o,c);
				}
				/*
				if ( c != (float)0.2) {
					fprintf(stderr,"Original %f and copy %f \n",o,c);
				}
				*/
			}
		}
	}
	EXPECT_TRUE(pass);
	
	delete original;
	delete copy;	
}

TEST (OctreeTests, triangleIntersection)
{
	int depth = 1;
	scan_utils::Octree<float>  octree(0,0,0, 2.0,2.0,2.0, depth, 0.2);
	
	float v0[] = {1.5, 0.0, 0.5};
	float v1[] = {0.0, 1.5, 0.5};
	float v2[] = {1.5, 1.5, 0.5};

	EXPECT_FALSE( octree.intersectsTriangle(v0,v1,v2) );

	octree.insert(-0.5, -0.5, -0.5, 1.0);
	EXPECT_FALSE( octree.intersectsTriangle(v0,v1,v2) );

	octree.insert(+0.5, +0.5, +0.5, 1.0);
	EXPECT_TRUE( octree.intersectsTriangle(v0,v1,v2) );

	octree.erase(0.5, 0.5, 0.5);
	EXPECT_FALSE( octree.intersectsTriangle(v0,v1,v2) );
}

TEST (OctreeTests, boxIntersection)
{
	int depth = 1;
	scan_utils::Octree<float>  octree(0,0,0, 2.0,2.0,2.0, depth, 0.2);

	float center[] = {1.0, 1.0, 1.0};
	float extents[] = {0.5, 0.5, 0.5};

	float axes[3][3];
	axes[0][0] =  0.7071; axes[0][1] = 0.7071; axes[0][2] = 0.0;
	axes[1][0] = -0.7071; axes[1][1] = 0.7071; axes[1][2] = 0.0;
	axes[2][0] =  0.0; axes[2][1] = 0.0; axes[2][2] = 1.0;
  
	EXPECT_FALSE( octree.intersectsBox(center, extents, axes) );

	octree.insert(-0.5, -0.5, -0.5, 1.0);
	EXPECT_FALSE( octree.intersectsBox(center, extents, axes) );

	octree.insert(+0.5, +0.5, +0.5, 1.0);
	EXPECT_TRUE( octree.intersectsBox(center, extents, axes) );

	octree.erase(0.5, 0.5, 0.5);
	EXPECT_FALSE( octree.intersectsBox(center, extents, axes) );

	octree.insert(+0.5, +0.5, +0.5, 1.0);
	center[0] = 0.5; center[1] = 1.6; center[2] = 0.5;

	axes[0][0] =  1.0; axes[0][1] = 0.0; axes[0][2] = 0.0;
	axes[1][0] =  0.0; axes[1][1] = 1.0; axes[1][2] = 0.0;
	axes[2][0] =  0.0; axes[2][1] = 0.0; axes[2][2] = 1.0;
	EXPECT_FALSE( octree.intersectsBox(center, extents, axes) );

	axes[0][0] =  0.7071; axes[0][1] = 0.7071; axes[0][2] = 0.0;
	axes[1][0] = -0.7071; axes[1][1] = 0.7071; axes[1][2] = 0.0;
	axes[2][0] =  0.0; axes[2][1] = 0.0; axes[2][2] = 1.0;
	EXPECT_TRUE( octree.intersectsBox(center, extents, axes) );
}

bool positive(float t) {
	return t > 0.0;
}

bool negative(float t) {
	return t < 0.0;
}

bool any(float t) {
	return t < 5;
}

bool nonzero(float t) {
	return t!=(float)0.0;
}

TEST (OctreeTests, triangulation)
{
	scan_utils::Octree<float>  octree(0,0,0, 2.0, 2.0, 2.0, 1, (float)0.0);
	std::list<scan_utils::Triangle> triangles;
	octree.insert(0.5, 0.5, 0.5, 1.0);
	octree.insert(-0.5, 0.5, 0.5, -1.0);

	octree.getTriangles(triangles,nonzero);
	EXPECT_TRUE(triangles.size() == 20);
	triangles.clear();

	octree.getAllTriangles(triangles);
	EXPECT_TRUE(triangles.size() == 20);
	triangles.clear();

	octree.getTriangles(triangles, &positive);
	EXPECT_TRUE(triangles.size() == 12);
	triangles.clear();

	octree.getTriangles(triangles, &negative);
	EXPECT_TRUE(triangles.size() == 12);
	triangles.clear();

	octree.getTriangles(triangles, &any);
	EXPECT_TRUE(triangles.size() == 48);
	triangles.clear();

}

TEST (OctreeTests, sphereIntersection)
{
	int depth = 1;
	scan_utils::Octree<float>  octree(0,0,0, 2.0,2.0,2.0, depth, 0.2);
	
	float center[] = {1.5, 1.5, 1.5};
	float radius = 0.9;

	EXPECT_FALSE( octree.intersectsSphere(center, radius) );

	octree.insert(-0.5, -0.5, -0.5, 1.0);
	EXPECT_FALSE( octree.intersectsSphere(center, radius) );

	octree.insert(+0.5, +0.5, +0.5, 1.0);
	EXPECT_TRUE( octree.intersectsSphere(center, radius) );

	octree.erase(0.5, 0.5, 0.5);
	EXPECT_FALSE( octree.intersectsSphere(center, radius) );
}

TEST (OctreeTests, serializationDeserialization)
{
	int depth = 4;
	int numCells = (int)pow(2,depth);
	float cellSize = 1.0 / numCells;

	scan_utils::Octree<float> *original = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, depth, 0.2);
	srand( (unsigned)time(NULL) );
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				original->insert(i1,i2,i3,((float)rand()) / RAND_MAX);
			}
		}
	}

	//fprintf(stderr,"Empty: %d %d\n",original->getNumLeaves(), original->getNumBranches());
	std::fstream os;
	os.open("testOctree.dat",std::fstream::out);
	original->writeToFile(os);
	os.close();

	scan_utils::Octree<float> *copy = new scan_utils::Octree<float>(0.4,0.4,0.1, 1.5,1.7,1.1, depth, 0.5);
	
	std::fstream is;
	is.open("testOctree.dat",std::fstream::in);
	bool result = copy->readFromFile(is);
	EXPECT_TRUE(result);
	is.close();

	bool pass = true;
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				float c = copy->get(i1,i2,i3);
				float o = original->get(i1,i2,i3);
				if ( fabs( c - o ) > 1.0e-6 ) {
					pass = false;
					//fprintf(stderr,"Original %f and copy %f \n",o,c);
				}
			}
		}
	}
	EXPECT_TRUE(pass);
	
	delete original;
	delete copy;
}

TEST (OctreeTests, toFromROSMessage)
{
	int depth = 4;
	int numCells = (int)pow(2,depth);
	float cellSize = 1.0 / numCells;

	scan_utils::Octree<float> *original = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, depth, 0.2);
	srand( (unsigned)time(NULL) );
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				original->insert(i1,i2,i3,((float)rand()) / RAND_MAX);
			}
		}
	}

	scan_utils::OctreeMsg msg;
	original->getAsMsg(msg);

	scan_utils::Octree<float> *copy = new scan_utils::Octree<float>(0.4,0.4,0.1, 1.5,1.7,1.1, depth, 0.5);
	bool result = copy->setFromMsg(msg);
	EXPECT_TRUE(result);
	
	bool pass = true;
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				float c = copy->get(i1,i2,i3);
				float o = original->get(i1,i2,i3);
				if ( fabs( c - o ) > 1.0e-6 ) {
					pass = false;
					//fprintf(stderr,"Original %f and copy %f \n",o,c);
				}
			}
		}
	}
	EXPECT_TRUE(pass);
	
	delete original;
	delete copy;
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
