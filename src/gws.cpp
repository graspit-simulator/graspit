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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: gws.cpp,v 1.24 2009/08/14 18:11:05 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the abstract Grasp Wrench Space class and specific GWS classes.
 */

#include <math.h>
#include <string.h>

#include "gws.h"
#include "grasp.h"
#include "mytools.h"
#include "body.h"

//#define GRASPITDBG
#include "debug.h"

extern "C" {
#include "qhull_a.h"
}
#include "qhull_mutex.h"

//! A global mutex used for synchronizing access to QHull, which is not thread-safe
QMutex qhull_mutex;
//Who actually needs this???
//char qh_version[] = "GraspIt 2.0b";

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

#define IVTOL 1e-5

const char *GWS::TYPE_LIST[] = {"L1 Norm","LInfinity Norm",NULL};
const char *L1GWS::type = "L1 Norm";
const char *LInfGWS::type = "LInfinity Norm";

typedef double *doublePtr;

/*! WARNING: this is replicated in Grasp::ALL_DIMENSIONS */
const std::vector<int> GWS::ALL_DIMENSIONS = std::vector<int>(6, 1);

/*!
  Deletes all the GWS hyperplanes.
*/
GWS::~GWS()
{
  DBGA("deleting GWS");
  clearGWS();
}

void
GWS::clearGWS()
{
  if (hyperPlanes) {
    for (int i=0;i<numHyperPlanes;i++)
      delete [] hyperPlanes[i];
    delete [] hyperPlanes;
    hyperPlanes=NULL;
  }
  numHyperPlanes = 0;
  hullArea = hullVolume = 0.0;
  forceClosure = false;
}

/*!
  Creates a GWS whose type string matches \a desiredType.
*/
GWS *
GWS::createInstance(const char *desiredType,Grasp *g)
{
  if (!strcmp(desiredType,L1GWS::getClassType()))
    return new L1GWS(g);

  if (!strcmp(desiredType,LInfGWS::getClassType()))
    return new LInfGWS(g);

  return NULL;
}


/*! Uses the \a projCoords and \a fixedCoordSet (which identifies the indices
	of the fixed coordinates) to project the 6D hyperplanes of the Grasp Wrench 
	Space into 3D. The \a projCoords is an array of 6 values, but only 3 are used.  
	It then calls qhull to perform a halfspace intersection to get the vertices 
	of the 3D volume.  These vertices are stored in \a hullCoords, and indices 
	of the individual faces that make up the volume are stored in \a hullIndices 
	(an Indexed Face Set).
*/
int
GWS::projectTo3D(double *projCoords, std::set<int> fixedCoordSet,
				 std::vector<position> &hullCoords, std::vector<int> &hullIndices)
{
  int i,j,k,validPlanes,numCoords,numInLoop;
  double **planes;
  int freeCoord[3],fixedCoord[3];

  // qhull variables
  boolT ismalloc;
  int curlong,totlong,exitcode;
  char options[200];  
  facetT *facet;

  if (numHyperPlanes == 0) {
	  DBGP("No hyperplanes");
	  return SUCCESS;
  }

  planes = (double **) malloc(numHyperPlanes * sizeof(double *));
  if (!planes) {
#ifdef GRASPITDBG
    pr_error("GWS::ProjectTo3D,Out of memory allocating planes array");
    printf("NumHyperplanes: %d\n",numHyperPlanes);
#endif
    return FAILURE;
  }

  validPlanes = 0;

  // determine which dimensions are free and which are fixed
  // the set keeps things ordered
  for (i=0,j=0,k=0;i<6;i++) {
    if (fixedCoordSet.find(i) == fixedCoordSet.end()) {
      freeCoord[k++] = i;
	} else {
      fixedCoord[j++] = i;
	}
  }

  // project the hyperplanes to three dimensional planes
  for (i=0;i<numHyperPlanes;i++) {
    double len = sqrt(hyperPlanes[i][freeCoord[0]]*hyperPlanes[i][freeCoord[0]] +
                      hyperPlanes[i][freeCoord[1]]*hyperPlanes[i][freeCoord[1]] +
                      hyperPlanes[i][freeCoord[2]]*hyperPlanes[i][freeCoord[2]]);

    if (len>1e-11) {
      planes[validPlanes] = (double *) malloc(4 * sizeof(double));
      if (!planes[validPlanes]) {
		pr_error("Out of memory allocating planes array");
	    DBGP("Out of memory allocating planes array");
		return FAILURE;
	  }
      planes[validPlanes][0] = hyperPlanes[i][freeCoord[0]]/len;
      planes[validPlanes][1] = hyperPlanes[i][freeCoord[1]]/len;
      planes[validPlanes][2] = hyperPlanes[i][freeCoord[2]]/len;
      planes[validPlanes][3] = (hyperPlanes[i][6] + 
	                            hyperPlanes[i][fixedCoord[0]]*projCoords[fixedCoord[0]] +
		                        hyperPlanes[i][fixedCoord[1]]*projCoords[fixedCoord[1]] +
		                        hyperPlanes[i][fixedCoord[2]]*projCoords[fixedCoord[2]])/len;
      
      validPlanes++;
	}
  }

  if (validPlanes<numHyperPlanes) {
	  DBGP("Ignored " << numHyperPlanes-validPlanes << 
		   " hyperplanes which did not intersect this 3-space");
  }
  if (!validPlanes) {
	  DBGA("No valid planes in 3D projection!");
	  return FAILURE;
  }
	   

  //
  // call qhull to do the halfspace intersection
  //
  coordT *array = new coordT[validPlanes*3];
  coordT *p = &array[0];
  
  boolT zerodiv;
  coordT *point, *normp, *coordp, **pointp, *feasiblep;
  vertexT *vertex, **vertexp;

#ifdef GRASPITDBG
  printf("Calling qhull to perform a 3D halfspace intersection of %d planes...\n",validPlanes);
#endif

  ismalloc = False; 	// True if qh_freeqhull should 'free(array)'

  // I want to get rid of this but qh_init needs some sort of file pointer
  // for stdout and stderr
  FILE *qhfp = fopen("logfile","w");

  if (!qhfp) {
	fprintf(stderr,"Could not open qhull logfile!\n");
	qh_init_A(NULL, stdout, stderr, 0, NULL);
  }
  else
   qh_init_A(NULL, qhfp, qhfp, 0, NULL);

  if ((exitcode = setjmp(qh errexit))) {
    delete [] array;
	qh NOerrexit= True;
	qh_freeqhull(!qh_ALL);
	qh_memfreeshort (&curlong, &totlong);
	if (curlong || totlong)  	/* optional */
	   fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
		 totlong, curlong);
    for (i=0;i<validPlanes;i++)
      free(planes[i]);
    free(planes);
	if (qhfp) fclose(qhfp);
	DBGP("Qhull forces the exit; probably no valid intersection");
    return FAILURE; //exit(exitcode);
  }
  sprintf(options, "qhull -H0,0,0 Pp");
  qh_initflags(options);
  qh_setfeasible(3);
  if (!(qh feasible_point)) printf("why is qh_qh NULL?\n");
  for(i=0;i<validPlanes;i++) {
    qh_sethalfspace (3, p, &p, planes[i],&(planes[i][3]), qh feasible_point);
  }

  qh_init_B(&array[0], validPlanes, 3, ismalloc);
  qh_qhull();
  qh_check_output();
  if (qhfp) fclose(qhfp);

  //
  // Collect the vertices of the volume
  //
  hullCoords.clear();
  numCoords = qh num_facets;
  hullCoords.reserve(numCoords);
  int *indices = new int[numCoords];

  double scale = grasp->getMaxRadius(); // Hmm, is this right?

  point= (pointT*)qh_memalloc (qh normal_size);

  FORALLfacets {
    coordp = point;
	if (facet->offset > 0)
      goto LABELprintinfinite;
	
    normp= facet->normal;
    feasiblep= qh feasible_point;
    if (facet->offset < -qh MINdenom) {
      for (k= qh hull_dim; k--; )
        *(coordp++)= (*(normp++) / - facet->offset) + *(feasiblep++);
    }else {
      for (k= qh hull_dim; k--; ) {
        *(coordp++)= qh_divzero (*(normp++), facet->offset, qh MINdenom_1,&zerodiv) + *(feasiblep++);
        if (zerodiv) {
          goto LABELprintinfinite;
        }
      }
    }    
    hullCoords.push_back(position(point[0]*scale,point[1]*scale,
				  point[2]*scale));
    continue;
    LABELprintinfinite:
    hullCoords.push_back(position(qh_INFINITE,qh_INFINITE,qh_INFINITE));
    fprintf(stderr,"intersection at infinity!\n");
  }
  qh_memfree (point, qh normal_size);


  //
  // use adjacency information to build faces of the volume
  //
  double dot;
  vec3 testNormal, refNormal;

  int numfacets, numsimplicial, numridges, totneighbors, numneighbors, numcoplanars;
  setT *vertices, *vertex_points, *coplanar_points;
  int numpoints= qh num_points + qh_setsize (qh other_points);
  int vertex_i, vertex_n;
  facetT *neighbor, **neighborp;
  int unused_numnumtricoplanarsp; //added because countfacets takes more arguments in qhull 2012
								  //FIXME - understand what this argument does. 
  qh_countfacets (qh facet_list, NULL, !qh_ALL, &numfacets, &numsimplicial, 
      &totneighbors, &numridges, &numcoplanars, &unused_numnumtricoplanarsp);  /* sets facet->visitid */

  qh_vertexneighbors();
  vertices= qh_facetvertices (qh facet_list, NULL, !qh_ALL);
  vertex_points= qh_settemp (numpoints);
  coplanar_points= qh_settemp (numpoints);
  qh_setzero (vertex_points, 0, numpoints);
  qh_setzero (coplanar_points, 0, numpoints);
  FOREACHvertex_(vertices)
    qh_point_add (vertex_points, vertex->point, vertex);
  FORALLfacet_(qh facet_list) {
    FOREACHpoint_(facet->coplanarset)
      qh_point_add (coplanar_points, point, facet);
  }

  FOREACHvertex_i_(vertex_points) {
    if (vertex) { 
      numneighbors= qh_setsize (vertex->neighbors);

      numInLoop = numneighbors;
      qh_order_vertexneighbors (vertex);
      j=0;
      FOREACHneighbor_(vertex) {
		if (!neighbor->visitid) {
			fprintf(stderr,"Uh oh! neighbor->visitId==0, -neighbor->id: %d\n",-int(neighbor->id));
			numInLoop--;
		}
		else
			indices[j] = neighbor->visitid - 1;

		if (j>0 && ((hullCoords[indices[j]]-hullCoords[indices[j-1]]).len() < IVTOL ||
			(hullCoords[indices[j]]-hullCoords[indices[0]]).len() < IVTOL)) 
		{
			numInLoop--;
		}
		else j++;
      }
	} else if ((facet= SETelemt_(coplanar_points, vertex_i, facetT))) {
      numInLoop=1;
	} else {
      numInLoop=0;
      continue;
    }

    if (numInLoop<3) {
      DBGP("too few vertices to make a face. Ignoring ...");
      continue;
    }

    // check if the current orientation of the face matches the plane's normal
    testNormal = (hullCoords[indices[1]] - hullCoords[indices[0]]) * (hullCoords[indices[j-1]] - hullCoords[indices[0]]);
	refNormal = vec3(planes[vertex_i][0],planes[vertex_i][1],planes[vertex_i][2]);

    if ((dot = testNormal % refNormal) > 0.0) {
      for (j=0;j<numInLoop;j++)
		hullIndices.push_back(indices[j]);
      hullIndices.push_back(-1);
    } else {
      // reverse the vertex ordering
      for (j=numInLoop-1;j>=0;j--)
		hullIndices.push_back(indices[j]);
      hullIndices.push_back(-1);
    }
  }

  qh_settempfree (&coplanar_points);
  qh_settempfree (&vertex_points);
  qh_settempfree (&vertices);

  qh NOerrexit= True;
  qh_freeqhull(!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);
  if (curlong || totlong)  	/* optional */
     fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
        totlong, curlong);

  for (i=0;i<validPlanes;i++)
    free(planes[i]);
  free(planes);

  delete [] indices;
  delete [] array;
  DBGP("Succesfull completion of intersection task");
  return SUCCESS;
}


void
GWS::setGravity(bool use, vec3 gd)
{
	useGravity = use;
	if (!use) gravDirection = vec3(0,0,0);
	else gravDirection = gd;
}

/*! Given a set of hyperplanes that have already been computed and stored 
	internally in hyperPlanes, this will compute the metrics that we care
	about: for now, the only computation is the min offset of a hyperplane
	from the origin. The other metric, the volume of the hull, is computed
	when a quality metric actually asks for it.

	This also uses the min offset to tell the grasp to display the min
	wrench that will break it.
*/
void
GWS::computeHyperplaneMetrics()
{
	if (!numHyperPlanes) {
		forceClosure = false;
		return;
	}

	int posOffsets = 0;
	int minIndex = 0;
	double minOffset = -1.0;

	for (int i=0; i<numHyperPlanes; i++) {
		if (hyperPlanes[i][6] > 0) posOffsets++;
		if (minOffset == -1.0 || -hyperPlanes[i][6]<minOffset) {
			minOffset = -hyperPlanes[i][6];
			minIndex = i;
		}
	}

	grasp->setMinWrench(hyperPlanes[minIndex]);

	if (posOffsets>0) {
		DBGP("QUALITY: NON FORCE CLOSURE GRASP, late exit");
		forceClosure = false;
	}
	else {
		DBGP("FORCE CLOSURE GRASP");
		forceClosure = true;
	}
}

/*! Encapsulates the qhull calls to build the convex hull of a set of wrenches.
	\a wr is a set of \a numWrenches wrenches, each of them having \a dimensions 
	dimensions. The convex hull is computed as the intersection of a number of
	halfspaces, each defined by a hyperplane. The hyperplanes are computed
	using qhull and stored internally in the GWS class.

	\a useDimensions tells us which dimensions we use when building the hull.
	The wrenches themselves are tightly packed, i.e. only contain the values for
	those dimensions. As far as qhull is concerned, it does not matter which 
	those directions are, only how many of them we have. But we need to know
	which the dimensions are so we can store that information in the hyperplanes.
*/
int 
GWS::buildHyperplanesFromWrenches(void *wr, int numWrenches, 
								  std::vector<int> useDimensions)
{
  DBGP("Qhull init on " << numWrenches << " wrenches"); 

  int dimensions = 0;
  for (int d=0; d<6; d++) {
    if (useDimensions[d]) dimensions++;
  }

  // qhull variables
  coordT *wrenches = (coordT*)wr;
  boolT ismalloc;
  int curlong, totlong, exitcode;
  char options[200];  
  facetT *facet;
  int i;

  ismalloc = False; 	// True if qh_freeqhull should 'free(array)'
  FILE *qhfp = fopen("logfile-qhull","w");
  if (!qhfp) {
	fprintf(stderr,"Could not open qhull logfile!\n");
	qh_init_A(NULL, stdout, stderr, 0, NULL);
  }
  else
   qh_init_A(NULL, qhfp, qhfp, 0, NULL);


  if ((exitcode = setjmp(qh errexit))) {
    DBGP("QUALITY: 0 volume in " << dimensions <<"D, quick exit");
	qh NOerrexit= True;
	qh_freeqhull(!qh_ALL);
	qh_memfreeshort (&curlong, &totlong);
	if (curlong || totlong)  	
	   fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
		  totlong, curlong);
	if (qhfp) fclose(qhfp);
    return FAILURE;
  }
  
  sprintf(options, "qhull Pp n Qx C-0.0001");
  qh_initflags(options);
  qh_init_B(&wrenches[0], numWrenches, dimensions, ismalloc);
  qh_qhull();
  qh_check_output();
  qh_getarea(qh facet_list);
  if (qhfp) fclose(qhfp);

  hullArea = qh totarea;
  hullVolume = qh totvol;
  numHyperPlanes = qh num_facets;

  hyperPlanes = new doublePtr[numHyperPlanes];
  if (!hyperPlanes) {
    DBGA("Out of memory allocating hyperPlanes array");
    return FAILURE;
  }

  for (i=0;i<numHyperPlanes;i++) {
    hyperPlanes[i] = new double[7];
    if (!hyperPlanes[i]) {
      DBGA("Out of memory allocating hyperPlanes array");
      return FAILURE;
    }
  }

  i=0;

  mUsedDimensions = useDimensions;
  FORALLfacets {
	  int hd = 0;
	  for (int d=0; d<6; d++) {
		  if (useDimensions[d]) {
			  hyperPlanes[i][d] = facet->normal[hd];
			  hd++;
		  } else {
			  hyperPlanes[i][d] = 0;
		  }
	  }
	  hyperPlanes[i][6] = facet->offset;  
	  i++;
  }

  qh NOerrexit= True;
  qh_freeqhull(!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);
  if (curlong || totlong) {
     fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
        totlong, curlong);
  }
  DBGP("Qhull SUCCESS");
  return SUCCESS;
}

/*!
  Builds an L1 GWS by simply taking the union of all the contact wrenches,
  and using these as the input for the convex hull operation
  (performed by qhull). The output of qhull is the list of equations of the
  hyperplanes that bound the hull as well as the hull area and hull volume.
  These are all saved within the GWS.
  Returns FAILURE or SUCCESS.

  You can ask for the GWS to be built using only a subset of the 6 dimensions 
  of force and torque normally available. These are, in this order: 
  
  fx, fy, fz, tx, ty, tz

  If you want only a subset to be used, pass a vector of 6 ints with 1 for the
  dimensions you want or 0 for those that you do not want. For example, if you
  want a GWS that only uses fx, fy and tz, pass the following vector:

  1, 1, 0, 0, 0, 1

  If you want to use all 6 dimensions, pass the default value of 
  GWS::ALL_DIMENSIONS.
*/
int
L1GWS::build(std::vector<int> useDimensions)
{
	DBGP("in L1 gws build");
	if (useGravity) {
		DBGP("Gravity: " << gravDirection.x() << " " << gravDirection.y() 
		  			     << " " << gravDirection.z());
	} else {
		DBGP("No gravity");
	}

	clearGWS();

	if (!grasp->getNumContacts()) {
		forceClosure = false;
		DBGP(" No contacts, returning");
		return SUCCESS;
	}

	int wrenchCount = 0;
	for (int i=0;i<grasp->getNumContacts();i++) {
		wrenchCount += grasp->getContact(i)->getMate()->numFrictionEdges;
	}

	//count the number of dimensions actually in use
	int numDimensions = 0;
	for (int d=0; d<6; d++) {
		if (useDimensions[d]) {
			numDimensions++;
		}
	}
	DBGP("Building a " << numDimensions << "D L1 GWS");

	if (numDimensions < 2) {
		DBGA("At least 2 used dimensions needed");
		return FAILURE;
	}
		
	coordT *array = new coordT[wrenchCount*numDimensions];
	if (!array) {
		DBGA("Failed to allocate memory for wrenchCount" << wrenchCount);
		return FAILURE;
	}

	coordT *p = &array[0];
	for (int i=0; i<grasp->getNumContacts(); i++) {    
		for (int w=0; w<grasp->getContact(i)->getMate()->numFrictionEdges; w++) {

			DBGP(" x: " << grasp->getContact(i)->getMate()->wrench[w].force.x() <<
				 " y: " << grasp->getContact(i)->getMate()->wrench[w].force.y() << 
				 " z: " << grasp->getContact(i)->getMate()->wrench[w].force.z() ); 
			DBGP("tx: " << grasp->getContact(i)->getMate()->wrench[w].torque.x() <<
				 " ty: " << grasp->getContact(i)->getMate()->wrench[w].torque.y() << 
				 " tz: " << grasp->getContact(i)->getMate()->wrench[w].torque.z() );		  

			if (useDimensions[0]) *p++ = grasp->getContact(i)->getMate()->wrench[w].force.x();
			if (useGravity) *(p-1) = *(p-1) + gravDirection.x();
			if (useDimensions[1]) *p++ = grasp->getContact(i)->getMate()->wrench[w].force.y();
			if (useGravity) *(p-1) = *(p-1) + gravDirection.y();
			if (useDimensions[2]) *p++ = grasp->getContact(i)->getMate()->wrench[w].force.z();
			if (useGravity) *(p-1) = *(p-1) + gravDirection.z();

			if (useDimensions[3]) *p++ = grasp->getContact(i)->getMate()->wrench[w].torque.x();
			if (useDimensions[4]) *p++ = grasp->getContact(i)->getMate()->wrench[w].torque.y();
			if (useDimensions[5]) *p++ = grasp->getContact(i)->getMate()->wrench[w].torque.z();

		}
	}

	//-----lock qhull access
	qhull_mutex.lock();

	int result;
	try {
		result = buildHyperplanesFromWrenches(array, wrenchCount, useDimensions);
	} catch(...) {
		DBGA("Build QHull failed!!!");
		result = FAILURE;
	}

	//-----unlock qhull access
	qhull_mutex.unlock();

	if (result == SUCCESS) {
		computeHyperplaneMetrics();
	} else{
		clearGWS();
	}

	DBGP("HULL AREA: " << hullArea);
	DBGP("HULL VOLUME: " << hullVolume);

	delete [] array;
	return result;
}

/*! 
  This computes the minkowski sum of the wrenches from a set of contacts.
  The result is a m^n set of wrenches where m is the number of boundary
  wrenches on each contact and n is the number of contacts.
*/
void
minkowskiSum(Grasp *g, int c, int &wrenchNum, coordT *wrenchArray, Wrench prevSum,
			 std::vector<int> useDimensions)
{
	static Wrench sum;

	int nd = 0;
	for (int d=0; d<6; d++) {
		if (useDimensions[d]) nd++;
	}
	if (c == g->getNumContacts()) {
		if (useDimensions[0]) wrenchArray[ wrenchNum*nd + 0 ] = prevSum.force.x();
		if (useDimensions[1]) wrenchArray[ wrenchNum*nd + 1 ] = prevSum.force.y();
		if (useDimensions[2]) wrenchArray[ wrenchNum*nd + 2 ] = prevSum.force.z();
		if (useDimensions[3]) wrenchArray[ wrenchNum*nd + 3 ] = prevSum.torque.x();
		if (useDimensions[4]) wrenchArray[ wrenchNum*nd + 4 ] = prevSum.torque.y();
		if (useDimensions[5]) wrenchArray[ wrenchNum*nd + 5 ] = prevSum.torque.z();
		wrenchNum++;
		return;
	}

	// Perform a sum that doesn't include any contribution from this contact
	minkowskiSum(g, c+1, wrenchNum, wrenchArray, prevSum, useDimensions);

	// perform a sum that includes all of this contact wrenches
	for (int w=0; w < g->getContact(c)->numFrictionEdges; w++) {

		sum.force = prevSum.force + g->getContact(c)->getMate()->wrench[w].force;
		sum.torque = prevSum.torque + g->getContact(c)->getMate()->wrench[w].torque;

		minkowskiSum(g, c+1, wrenchNum, wrenchArray, sum, useDimensions);
	}
}

/*!
  Builds an L Infinity GWS by finding the minkowski sum of the set of contact
  wrenches, and using these as the input for the convex hull operation.
  (performed by qhull). The output of qhull is the list of equations of the
  hyperplanes that bound the hull as well as the hull area and hull volume.
  These are all saved within the GWS.
  Returns FAILURE or SUCCESS.

  You can ask for the GWS to be built using only a subset of the 6 dimensions 
  of force and torque normally available. These are, in this order: 
  
  fx, fy, fz, tx, ty, tz

  If you want only a subset to be used, pass a vector of 6 ints with 1 for the
  dimensions you want or 0 for those that you do not want. For example, if you
  want a GWS that only uses fx, fy and tz, pass the following vector:

  1, 1, 0, 0, 0, 1

  If you want to use all 6 dimensions, pass the default value of 
  GWS::ALL_DIMENSIONS.
*/
int
LInfGWS::build(std::vector<int> useDimensions)
{
  clearGWS();
  if (!grasp->getNumContacts()) {
    forceClosure = false;
    return SUCCESS;
  }

  //count the number of dimensions actually in use
  int numDimensions = 0;
  for (int d=0; d<6; d++) {
	  if (useDimensions[d]) {
		  numDimensions++;
	  }
  }

  int wrenchCount = 1;
  for (int i=0; i<grasp->getNumContacts(); i++) {
	  wrenchCount *= grasp->getContact(i)->getMate()->numFCWrenches + 1;
	  if (wrenchCount > INT_MAX/6.0) {  //what is a reasonable threshold ?
	    DBGA("Too many contacts to compute the Minkowski sum!");
        return FAILURE;
      }
  }

  coordT *array = new coordT[wrenchCount*6];
  if (!array) {
	  DBGA("Could not allocate wrench array in ComputeLInfHull. wrenchCount: " << wrenchCount);
      return FAILURE;
  }

  Wrench initSum;
  initSum.force = vec3::ZERO;
  initSum.torque = vec3::ZERO;

  int wrenchNum = 0;
  minkowskiSum(grasp, 0, wrenchNum, array, initSum, useDimensions);

  //-----lock qhull access
  qhull_mutex.lock();
  
  int result;
  try {
	result = buildHyperplanesFromWrenches(array, wrenchCount, useDimensions);
  } catch(...) {
	DBGA("Build QHull 3D failed!!!");
	result = FAILURE;
  }
  
  //-----release qhull access
  qhull_mutex.unlock();

  if (result == SUCCESS) {
	  computeHyperplaneMetrics();
  } else{
	  clearGWS();
  }

  DBGP("HULL VOLUME: " << hullVolume);

  delete [] array;
  return result;
}