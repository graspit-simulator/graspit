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
// Author(s): Andrew T. Miller
//
// $Id: lmiOptimizer.cpp,v 1.1 2009/07/21 19:30:41 cmatei Exp $
//
//######################################################################

#include "lmiOptimizer.h"

#include "grasp.h"
#include "matvec3D.h"
#include "robot.h"
#include "contact.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

#include "maxdet.h"

bool errorOccurred = false;

double LMIOptimizer::GFO_WEIGHT_FACTOR = 1.0;

//print out error message and exit
void FatalErrMsg(char* s)
{
  fprintf(stderr,"error: %s\n",s);
  exit(-1);
}

#define errMsg(S_)                 \
{                                  \
  fprintf(stderr,"error: %s\n",S_);\
  errorOccurred = true;            \
  return;                          \
} 

/*!
  Create a diagonal-block identity matrix
*/
double * block_Identity(int numBlocks, int * blockSizes, int pkSize)
{

  int i, j, itemIndex, blockDim;
  double db0=0.0,*identity= (double*) malloc(pkSize*sizeof(double));
  
  dcopy(pkSize,&db0, 0, identity, 1);
  itemIndex=0;
  for(i=0;i<numBlocks; i++){
    blockDim=blockSizes[i];
    for(j=0;j<blockDim;j++){
      identity[itemIndex]=1.0;
      itemIndex+=blockDim-j;
    }
  }
  return(identity);
}

/*!
  Constructs the hand jacobian.  It currently assumes there is no palm motion,
  hence the matrix is \c numDOF x \c numWrenches (\c numWrenches = 3 *
  \c mGrasp->numContacts when each contact is PCWF.  This jacobian relates joint
  velocities to contact velocities and its transpose relates contact forces
  to joint torques.  

  Distances used to be in meters. Now they are in millimeters.
*/
void
LMIOptimizer::buildJacobian()
{
	double db0=0.0;
	int i,f,l,blkOffset,rowOffset;
	transf Tec;
	double *Jee,*Tv_ec,*J;
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;
  
	numDOF = mGrasp->hand->getNumDOF();   // +6 for palm motion

	if (externalTorques) delete [] externalTorques;
	externalTorques = new double[numDOF];
	for (i=0;i<numDOF;i++) {
		externalTorques[i] = mGrasp->hand->getDOF(i)->getExtForce();
	}

	if (Jacobian) delete [] Jacobian;
	Jacobian = new double[numDOF*numWrenches];
	dcopy(numDOF*numWrenches,&db0,0,Jacobian,1);
	blkOffset = 0;

	// J is an individual contact jacobian
	J = new double[6*numDOF];

	for (f=0;f<mGrasp->hand->getNumFingers();f++) {
		for (l=0;l<mGrasp->hand->getFinger(f)->getNumLinks();l++) {
   			contactList = mGrasp->hand->getFinger(f)->getLink(l)->getContacts();
			if (contactList.empty()) continue;

			Jee = mGrasp->getLinkJacobian(f,l);

			for (cp=contactList.begin();cp!=contactList.end();cp++) {
				if ((*cp)->getBody2() != mGrasp->object) continue;
				dcopy(6*numDOF,&db0,0,J,1);

				// Tec: tranform from link base to contact point
				Tec = rotate_transf(M_PI,vec3(1,0,0)) * (*cp)->getContactFrame();
				Tv_ec = new double[36];
				Tec.jacobian(Tv_ec);
				// J = Tv_ec * Jee;
				dgemm("N","N",6,numDOF,6,1.0,Tv_ec,6,Jee,6,0.0,J,6);
   				delete [] Tv_ec;

				// we only want to include the portion of J that is admissible
				// by the contact type
	    
				rowOffset = 0;
	    
				if ((*cp)->getContactDim() >= 3) {
					// copy the first row of J (dx/dtheta)
					dcopy(numDOF,J,6,Jacobian+blkOffset+rowOffset,numWrenches);
					rowOffset++;
      				// copy the second row of J (dy/dtheta)
					dcopy(numDOF,J+1,6,Jacobian+blkOffset+rowOffset,numWrenches);
					rowOffset++;
				}
   
				// copy the third row of J (dz/dtheta)
				dcopy(numDOF,J+2,6,Jacobian+blkOffset+rowOffset,numWrenches);
				rowOffset++;
    
				if ((*cp)->getContactDim() > 3) {
					// copy the sixth row of J (drz/dtheta)
					dcopy(numDOF,J+5,6,Jacobian+blkOffset+rowOffset,numWrenches);
					rowOffset++;
				}
	    		blkOffset += rowOffset;
			}
			delete [] Jee;
		}
	}  
	delete [] J;

#ifdef GRASPDEBUG
	printf("JACOBIAN:\n");
	disp_mat(stdout,Jacobian,numWrenches,numDOF,0);
#endif

}


/*!
  Constructs the grasp map matrix.  This is a 6 x \c numWrenches
  (\c numWrenches = 3 * \c mGrasp->numContacts, when each contact is PCWF) matrix
  that computes the net object wrench given a vector of magnitudes for
  the contact wrenches.  
*/
void
LMIOptimizer::buildGraspMap()
{
  int i,curLoc = 0;

  if (graspMap) delete [] graspMap;
  graspMap = NULL;

  // count the total number of wrenches contributed by all the contacts
  numWrenches = 0;
  for(i=0;i<mGrasp->numContacts;i++)
    numWrenches += mGrasp->contactVec[i]->getContactDim();

  if (mGrasp->numContacts) {
    graspCounter++;
    graspMap = new double[6*numWrenches];

    /*
      This code has not been verified in a long time. Please check that the frictionEdges
      below indeed do what you expect them to
    */
    fprintf(stderr,"Unverified code! Check me first!\n");
    // add the basis wrenches of each contact to the grasp map
    for (i=0;i<mGrasp->numContacts;i++) {
      dcopy(6*mGrasp->contactVec[i]->numFrictionEdges,
	    mGrasp->contactVec[i]->getMate()->frictionEdges,1,graspMap+curLoc,1);
	  curLoc += 6 * mGrasp->contactVec[i]->getContactDim();
    }
#ifdef GRASPDEBUG
    printf("Built GraspMap (6x%d):\n",numWrenches);
    disp_mat(stdout,graspMap,6,numWrenches,1);
#endif

  }
}

/*!
  Computes the null space of the grasp map.
  Adapted from Li Han's code.
*/
void
LMIOptimizer::computeNullSpace() {

  double *tempGraspMap, *work;
  double *leftSV, *singularValues, *rightSV, *checkNull, *checkRSV;
  double db0=0.0;
  int lwork, rank, info;
  int numGElements, numCheck, i;
  int row = 6, col = numWrenches;
  
  rank=row;
  nullDim=col-rank;
  if (nullDim <= 0) return;

  if (nullSpace) free(nullSpace);

  numGElements = row*col;
  tempGraspMap=(double*) malloc(numGElements*sizeof(double));
  dcopy(numGElements,graspMap,1, tempGraspMap,1);
  
  lwork=MAX(MAX(1,3*MIN(row,col)+ MAX(row,col)),5*MIN(row,col));  //-4
  work=(double*) malloc(lwork*sizeof(double));
  singularValues=(double*)malloc(MIN(row,col)*sizeof(double));
  leftSV=(double*)malloc(row*row*sizeof(double));
  rightSV=(double*)malloc(col*col*sizeof(double));

#ifdef GRASPDEBUG  
  printf("--------- tempGraspMap ------------ \n");
  disp_mat(stdout,tempGraspMap,row,col,0);
#endif
  
  dgesvd("A","A", row, col,tempGraspMap, row, singularValues, leftSV, row,
	 rightSV, col, work, lwork, &info);
  if(info==0){

#ifdef GRASPDEBUG  
      printf("--------- singular values ------------ \n");
      disp_mat(stdout,singularValues,1,MIN(row,col),0);
      
      printf("--------- Left Singular Vectors -------- \n");
      disp_mat(stdout, leftSV, row, row,0);
      printf("--------- Right Singular Vectors -------- \n");
      disp_mat(stdout, rightSV, col, col,0);
      
      printf("-------- GraspMap*RightSV ------------ \n");
#endif

      checkRSV=(double*)malloc(col*col*sizeof(double));
      dcopy(numGElements,graspMap,1, tempGraspMap,1);
      dgemm("N","T",row,col,col,1.0,tempGraspMap,row,rightSV,col,0.0,
      checkRSV,col);
#ifdef GRASPDEBUG  
      disp_mat(stdout, checkRSV, col, col,0);
#endif
      free(checkRSV);

    nullSpace=(double *)malloc(col* nullDim *sizeof(double));
    for (i=row;i<col;i++)
      dcopy(col,rightSV+i,col,nullSpace+(i-row)*col,1);
    
#ifdef GRASPDEBUG  
      printf("---------- Null Space -----------------\n");
      disp_mat(stdout, nullSpace, col, nullDim,0);
      
      printf("---------- check null space --------- \n");
#endif
      numCheck=row*nullDim;
      checkNull=(double*)malloc(numCheck*sizeof(double));
      dcopy(numCheck, &db0, 0,checkNull,1);
      dcopy(numGElements,graspMap,1, tempGraspMap,1);
      dgemm("N","N", row, nullDim, col, 1.0, tempGraspMap, row,
      nullSpace,col, 0.0, checkNull,row);
#ifdef GRASPDEBUG  
      disp_mat(stdout, checkNull, row, nullDim, 0);
#endif
      free(checkNull);

    free(tempGraspMap);free(work); 
    free(leftSV); free(singularValues); free(rightSV);
  }
  else {
    free(tempGraspMap);free(work);
    free(leftSV); free(singularValues); free(rightSV);    
    errMsg("error in SVD of grasp map");
  }
}

/*! 
  Find the minimal norm solution to the object wrench equilibirum equation.
  Adapted from Li Han's Code - I changed the sign of the object wrench.

  The solution to the linear equation \f$ G x + F_{ext} = 0 \f$
  is \f$ x = x_0 + V z \f$
  where \f$ x_0 = G^+ * F_{ext} \f$ is the minimal normal solution.

  Grasp map:   \f$G\f$. \n
  minNormSln:  \f$x_0\f$.\n
  nullSpace:   \f$V\f$. \n
*/
void
LMIOptimizer::minimalNorm()
{
  double *tempGraspMap, *work, *incForce;
  int lwork, info;
  int numGElements, ldx0, i;
  int row = 6, col = numWrenches, nrhs = 1;

  if (minNormSln) free(minNormSln);

  incForce=(double*)malloc(row*sizeof(double));
  dcopy(row,mGrasp->object->getExtWrenchAcc(),1,incForce,1);

  // Grasp should generate the _negative_ object wrench
  dscal(row,-1.0,incForce,1);

#ifdef GRASPDEBUG  
  printf("---------- Object Wrench --------------------- \n");
  disp_mat(stdout, mGrasp->object->getExtWrenchAcc(), row, 1,0);
#endif

  minNormSln = (double*) malloc(MAX(row,col)*nrhs*sizeof(double));
  //lwork for dgels_ 
  lwork=MAX(1,MIN(row,col)+MAX(row, MAX(col,nrhs)));
  work= (double*)malloc(lwork*sizeof(double));
  
  numGElements = row*col;
  tempGraspMap=(double*) malloc(numGElements*sizeof(double));
  dcopy(numGElements,graspMap,1, tempGraspMap,1);
  
  ldx0=MAX(row,col);
  for (i=0; i<nrhs;i++)
    dcopy(row,incForce+i*row, 1, minNormSln+i*ldx0, 1);
  
  dgels("N", row, col, nrhs, tempGraspMap, row, minNormSln, ldx0, work,
	 lwork, &info);
  
  if(info==0){
#ifdef GRASPDEBUG
    printf("preset work length: %d, optimal length: %f \n", lwork, work[0]);
    printf("---------- Minimal Norm/Error Solution ---------- \n");
    disp_mat(stdout, minNormSln, nrhs, ldx0,0);
#endif
    free(work); free(tempGraspMap); free(incForce);
  }
  else {
    free(work); free(tempGraspMap); free(minNormSln); free(incForce);
    minNormSln = NULL;
    errMsg(" no exact solution exists");
  }    
}

/*!
  This sets up the contact limits lmi.  It requires the Grasp map null space,
  the minimum norm solution and the lower and upper bounds of each null space
  dimension.
    
  Adapted from Li Han's Code.
*/
double *
lmiContactLimits(int numForces, int nullDim,  double* nullSpace,
			 double * minNormSln, double* lowerBounds,
			 double* upperBounds)
{
  double *LmiLinear, *tempLower, *tempUpper;
  int row, col, nullSpaceSize;
  int i;
  
  tempLower=(double*)malloc(numForces*sizeof(double));
  dcopy(numForces, lowerBounds, 1, tempLower, 1);
  daxpy(numForces, -1.0, minNormSln, 1, tempLower, 1);  //lower-minNormSln
  dscal(numForces,-1.0,tempLower, 1);			 //-(lower-minNormSln)
  
  tempUpper=(double*)malloc(numForces*sizeof(double));	
  dcopy(numForces, upperBounds, 1, tempUpper, 1);
  daxpy(numForces, -1.0, minNormSln, 1, tempUpper, 1);  // upper - minNormSln
  
  row=2*numForces;   col=nullDim+1; 
  LmiLinear=(double*)malloc(row*col*sizeof(double));
  dcopy(numForces, tempLower, 1, LmiLinear, 1);
  dcopy(numForces, tempUpper, 1, LmiLinear+numForces, 1);
  
  for(i=0;i<nullDim; i++)
    dcopy(numForces, nullSpace+i*numForces, 1, LmiLinear+(i+1)*row,1);
  nullSpaceSize=numForces*nullDim;
  dscal(nullSpaceSize,-1.0, nullSpace, 1);
  for(i=0;i<nullDim; i++)
    dcopy(numForces, nullSpace+i*numForces, 1, LmiLinear+(i+1)*row+numForces,1);
  
  dscal(nullSpaceSize,-1.0, nullSpace, 1);
  free(tempLower); free(tempUpper);
  return(LmiLinear);
}

/*! 
  Adapted from Li Han's Code.  
*/
double *
LMIOptimizer::lmiTorqueLimits()
{
  double  db0=0.0,*tildeJ, *tildeExternalTrq;
  int i,tildeJDim=numDOF*nullDim;
  double *lmi,*lowerBounds,*upperBounds;

  /*  L=2*numWrenches;
  F_blkszs= (int *) malloc(L*sizeof(int));
  for (int i=0;i<L;i++) F_blkszs[i]=1;
  */

  L=2*numDOF;
  F_blkszs= (int *) malloc(L*sizeof(int));
  for (i=0;i<L;i++) F_blkszs[i]=1;

  lowerBounds = (double*) malloc(numDOF*sizeof(double));
  upperBounds = (double*) malloc(numDOF*sizeof(double));
  for (i=0;i<mGrasp->hand->getNumDOF();i++) {
    upperBounds[i] = mGrasp->hand->getDOF(i)->getMaxForce()*1.0e-9;
    lowerBounds[i] = - mGrasp->hand->getDOF(i)->getMaxForce()*1.0e-9;
  }

  tildeJ=(double*) malloc(tildeJDim*sizeof(double));
  dcopy(tildeJDim, &db0, 0, tildeJ,1);

  dgemm("T","N", numDOF, nullDim, numWrenches, 1.0, Jacobian, numWrenches,
	 nullSpace, numWrenches, 0.0, tildeJ, numDOF);

  tildeExternalTrq=(double*) malloc(numDOF*sizeof(double));
  dcopy(numDOF, externalTorques, 1, tildeExternalTrq, 1);
  
  dgemv("T", numWrenches, numDOF, 1.0, Jacobian, numWrenches, minNormSln, 1,
	 1.0, tildeExternalTrq, 1);

#ifdef GRASPDEBUG  
  printf("tildeExternalTrq:\n");
  disp_mat(stdout,tildeExternalTrq,1,numDOF,0);
#endif

  lmi = lmiContactLimits(numDOF, nullDim, tildeJ, tildeExternalTrq,
		       lowerBounds, upperBounds);

  free(tildeJ); free(tildeExternalTrq);  free(lowerBounds); free(upperBounds);
  return(lmi);
}

void
LMIOptimizer::lmiFL(double *lmi,int rowInit, int colInit, int totalRow)
{
  int x3_post=colInit*totalRow+rowInit;
  lmi[x3_post]=1.0;
}

void
LMIOptimizer::lmiPCWF(double cof, double *lmi,int rowInit, int colInit,
		    int totalRow)
{
  int x1_post, x2_post, x3_post;

  x1_post=colInit*totalRow+rowInit+2;
  lmi[x1_post]=1.0;
  
  x2_post=(colInit+1)*totalRow+rowInit+4;
  lmi[x2_post]=1.0;
  
  x3_post=(colInit+2)*totalRow+rowInit;
  lmi[x3_post]=cof;
  lmi[x3_post+3]=cof;
  lmi[x3_post+5]=cof;
}

void
LMIOptimizer::lmiSFCE(double cof, double cof_t,double *lmi,int rowInit,
		    int colInit, int totalRow)
{
  int x1_post, x2_post, x3_post, x4_post;
  double alpha, beta;
  
  alpha=1.0/sqrt(cof);
  beta =1.0/sqrt(cof_t);
  
  x1_post=colInit*totalRow+rowInit+3;
  lmi[x1_post]=alpha;
  
  x2_post=(colInit+1)*totalRow+rowInit+6;
  lmi[x2_post]=alpha;
  
  x3_post=(colInit+2)*totalRow+rowInit;
  lmi[x3_post]=1.0;
  lmi[x3_post+4]=1.0;
  lmi[x3_post+7]=1.0;
  lmi[x3_post+9]=1.0;

  x4_post=(colInit+3)*totalRow+rowInit+8;
  lmi[x4_post]=beta;
}

void
LMIOptimizer::lmiSFCL(double cof, double cof_t,double *lmi,int rowInit,
		    int colInit, int totalRow)
{
  int colFirst;
  double ratio = cof/cof_t;

  // The 1 dimensional index for the first item of this LMI block
  // in the x_i column

  //x_1
  colFirst=colInit* totalRow+rowInit;
  lmi[colFirst+9]=1.0;
  lmi[colFirst+24]=1.0;
  
  //x_2
  colFirst=(colInit+1)*totalRow+rowInit;
  lmi[colFirst+14]=1.0;
  lmi[colFirst+26]=1.0;
    
  //x_3
  colFirst=(colInit+2)*totalRow+rowInit;
  lmi[colFirst]   =1.0;
  lmi[colFirst+7] =cof;
  lmi[colFirst+13]=cof;
  lmi[colFirst+18]=cof;
  lmi[colFirst+22]=cof;
  lmi[colFirst+25]=cof;
  lmi[colFirst+27]=cof;
  
  //x_4
  colFirst=(colInit+3)*totalRow+rowInit;
  lmi[colFirst+7] =ratio;
  lmi[colFirst+13]=ratio;
  lmi[colFirst+18]=ratio;
  lmi[colFirst+22]=-ratio;
  lmi[colFirst+25]=-ratio;
  lmi[colFirst+27]=-ratio;
}

// note that LMI is stored in a packed form where the whole lower triangle of
// each P_i is stored as a single column (remember: column-major format)
//
double *
LMIOptimizer::lmiFrictionCones()
{
  double db0=0.0,*initLMI, *finalLMI;
  int lmiDim;
  int row=0, col=0, *blockRowIndex, *blockColIndex;
  int i, initSize, finalSize;

  blockRowIndex = new int[mGrasp->numContacts];
  blockColIndex = new int[mGrasp->numContacts];

  K=mGrasp->numContacts;
  G_blkszs=(int *)malloc(K*sizeof(int));

  for (i=0;i<mGrasp->numContacts;i++) {
    blockRowIndex[i]=row;
    blockColIndex[i]=col;
    lmiDim = mGrasp->contactVec[i]->getLmiDim();
    row += lmiDim*(lmiDim+1)/2;  // size of the lower triangle
    col += mGrasp->contactVec[i]->getContactDim();
    G_blkszs[i]=lmiDim;
  }

  initSize=row*col;
  initLMI=(double*)malloc(initSize*sizeof(double));
  dcopy(initSize,&db0, 0, initLMI, 1);

  for (i=0;i<mGrasp->numContacts;i++) {
    switch(mGrasp->contactVec[i]->getFrictionType()) {
      case FL:   lmiFL(initLMI,blockRowIndex[i],blockColIndex[i], row);
	         break;
      case PCWF: lmiPCWF(mGrasp->contactVec[i]->getCof(),initLMI,blockRowIndex[i],
			 blockColIndex[i], row);
                 break;
      case SFCE: assert(0); //not handled
			 //lmiSFCE(mGrasp->contactVec[i]->getCof(),
			 //mGrasp->contactVec[i]->getTorsionalCof(),initLMI,
			 //blockRowIndex[i], blockColIndex[i], row);
                 break;
      case SFCL: assert(0); //not handled
		     //lmiSFCL(mGrasp->contactVec[i]->getCof(),
			 //mGrasp->contactVec[i]->getTorsionalCof(),initLMI,
			 //blockRowIndex[i], blockColIndex[i], row);
    }
  }
#ifdef GRASPDEBUG  
  printf("------------- Init LMI(%-dx%-d) ----------------------\n",row, col);
  disp_mat(stdout, initLMI, row, col,0);
#endif

  finalSize=row*(nullDim+1);
  
  finalLMI = (double*) malloc(finalSize*sizeof(double));
  dcopy(finalSize,&db0, 0, finalLMI, 1);
  dgemm("N","N",row, nullDim, col, 1.0, initLMI, row, nullSpace, col, 
	 0.0,finalLMI+row, row);
  dgemv("N",row,col, 1.0, initLMI, row, minNormSln, 1, 0.0, finalLMI, 1);
  free(initLMI);
  delete [] blockRowIndex;
  delete [] blockColIndex;
  GPkRow = row;
  return(finalLMI);
}

double *
LMIOptimizer::weightVec()
{
  double  db0=0.0,*initWeights, *finalWeights;
  int i,dim,blockIndex=0;
  
  initWeights=(double*)malloc(numWrenches*sizeof(double));
  for (i=0;i<mGrasp->numContacts;i++) {
    dim = mGrasp->contactVec[i]->getContactDim();
    dcopy(dim,&db0,1,initWeights+blockIndex,1);

    // temporary preset weights
    if (dim==1)
      *(initWeights+blockIndex) = LMIOptimizer::GFO_WEIGHT_FACTOR;
    else
      *(initWeights+blockIndex+2) = LMIOptimizer::GFO_WEIGHT_FACTOR;
    
    blockIndex += dim;
  }

#ifdef GRASPDEBUG
  printf("------------ Init Weight Vector(%d) -------------\n",numWrenches);
  disp_mat(stdout, initWeights,1,numWrenches,0);
  
      printf("---------- Null Space -----------------\n");
      disp_mat(stdout, nullSpace, numWrenches, nullDim,0);
#endif
      
  finalWeights=(double*)malloc(nullDim*sizeof(double));
  dcopy(nullDim,&db0,0,finalWeights,1);
  dgemv("T",numWrenches,nullDim,1.0,nullSpace,numWrenches,initWeights,1,
	0.0,finalWeights, 1);
  
  constOffset = ddot(numWrenches,initWeights,1,minNormSln,1);

#ifdef GRASPDEBUG
  printf("------------ Final Weight Vector(%d) for new variables z & %lf -------------\n",nullDim, constOffset);
  disp_mat(stdout, finalWeights,1,nullDim,0);
#endif

  free(initWeights);
  return(finalWeights);
}

/*!
  Combines LMIs \a lmi1 >0 and \a lmi2 >0 into the LMI \n
  [lmi1    0]\n
  [0    lmi2] > 0
*/
double *
combineLMIs(int numVariables, double* lmi1, int lmi1RowSz, double * lmi2,
	    int lmi2RowSz) {

  double * combinedLMI;
  int numElements, totalRowSz;
  int i;
  
  totalRowSz=lmi1RowSz+lmi2RowSz;
  numElements=(numVariables+1)*totalRowSz;
  combinedLMI=(double*)malloc(numElements*sizeof(double));
  for(i=0;i<=numVariables;i++){
    dcopy(lmi1RowSz,lmi1+i*lmi1RowSz,1,combinedLMI+i*totalRowSz,1);
    dcopy(lmi2RowSz,lmi2+i*lmi2RowSz,1,combinedLMI+i*totalRowSz+lmi1RowSz,1);
  }
  return(combinedLMI);
}

/*!
  Prepare the parameters used by maxdet package and
  use it to solve the maxdet optimization problem.
*/
double * maxdet_wrap(int m, int L, double *F, int *F_blkszs,
                   int K, double *G, int *G_blkszs,
                   double *c, double *z0, double *Z, double *W,
                   double gamma, double abstol, double reltol,
                   int * pNTiters, int negativeFlag, FILE* pRstFile)


{
  register int i;
  int    n, l, max_n, max_l, F_sz, G_sz;  
  int    ptr_size;
  
  int *iwork, lwork;
  double db0=0.0,ul[2], *hist, *truncatedHist, *work;
  int m3, m2, sourceCol, destCol, destSize;
  int info, info2;
  
  
  //  struct tms before,after;
  
  // Gets rid of compiler warning
#ifndef GRASPDBG
  pRstFile = NULL;
#endif
  
  for (i=0; i<L; i++) {
    if (F_blkszs[i] <= 0) {
      pr_error("Elements of F_blkszs must be positive.\n");
      errorOccurred = true;
      return NULL;
    }
  }
  for (i=0; i<K; i++) {
    if (G_blkszs[i] <= 0) {
      pr_error("Elements of G_blkszs must be positive.\n");
      errorOccurred = true;
      return NULL;
    }
  }
  
  /* various dimensions
   * n, l: dimensions
   * F_sz, G_sz: length in packed storage
   * F_upsz, G_upsz: length in unpacked storage
   * max_n, max_l: max block size */
  for (i=0, n=0, F_sz=0, max_n=0; i<L; i++) {
    n += F_blkszs[i];
    F_sz += (F_blkszs[i]*(F_blkszs[i]+1))/2;
    max_n = MAX(max_n, F_blkszs[i]);
  }
  for (i=0, l=0, G_sz=0, max_l=0; i<K; i++) {
    l += G_blkszs[i];
    G_sz += (G_blkszs[i]*(G_blkszs[i]+1))/2;
    max_l = MAX(max_l, G_blkszs[i]);
  }
  
  /* hist (5th output argument) */
  ptr_size =(3+m)*(*pNTiters);
  //    ptr_size =(3+m)*(NTiters);
  hist=(double *) malloc(ptr_size*sizeof(double));
  dcopy(ptr_size,&db0, 0, hist, 1);
  
  /* allocate work space */
  lwork = (2*m+5)*(F_sz+G_sz) + 2*(n+l) +
    MAX(m+(F_sz+G_sz)*NB,MAX(3*(m+SQR(m)+MAX(G_sz,F_sz)),
			     MAX(3*(MAX(max_l,max_n)+MAX(G_sz,F_sz)),
				 MAX(G_sz+3*max_l,F_sz+3*max_n))));
  work = (double *) malloc(lwork*sizeof(double));
  dcopy(lwork,&db0,0,work,1);
  iwork = (int *) malloc(10*m*sizeof(int));
  for(i=0;i<10*m;i++)	iwork[i]=0;
  
  //  times(&before);
  info2=maxdet(m,L, (double*)F,(int *) F_blkszs,K,(double *) G,
	       (int *) G_blkszs,(double *)c,(double *)z0,(double *)Z,
	       (double *)W, (double *)ul, (double*)hist,gamma,abstol,reltol,
	       pNTiters,work,lwork,iwork,&info,negativeFlag);
  
  if (info2) {
      free(work); free(iwork); free(hist);
      errorOccurred = true;
      pr_error("Error in maxdet.\n");
      return NULL;
  }
  
  // times(&after);
  
  //#ifndef REAL_RUN
  //  fprintf(pRstFile,"times:  User time: %f seconds\n",(float) (after.tms_utime-before.tms_utime)/sysconf(CLK_TCK));
  //#endif
  
  // infostr 
#ifdef GRASPDEBUG
  switch (info) {
  case 1:
    fprintf(pRstFile, "  maximum Newton iteration exceeded\n");
    break;
  case 2:
    fprintf(pRstFile, "  absolute tolerance reached\n");
    break;
  case 3:
    fprintf(pRstFile, "  relative tolerance reached\n");
    break;
  case 4:
    fprintf(pRstFile, "  negative objective value reached\n");
    break;
  default:
    printf("error occurred in maxdet\n"); 
    exit(-1);
  }  
#endif

  //  truncate hist
  destCol=0;
  m3=m+3; m2=m3 -1;
  for(sourceCol=0;sourceCol<*pNTiters;sourceCol++) 
    if(hist[sourceCol*m3+m2]!=0.0){
      dcopy(m3,hist+sourceCol*m3,1,hist+destCol*m3,1);
      destCol++;
    }
  //        printf("init Col: %d  final Col: %d \n", sourceCol, destCol);
  destSize=destCol*m3;
  truncatedHist=(double *)malloc(destSize*sizeof(double));
  dcopy(destSize, hist, 1, truncatedHist, 1);
  *pNTiters=destCol;
  
  
  /* free matrices allocated */
  free(work); free(iwork); free(hist);
  
  return(truncatedHist);
}

/*!
  Solves the question of whether there is a feasible set of contact forces
  that will result in equilibrium for the object.
*/
void
LMIOptimizer::feasibilityAnalysis()
{
  double *eigF, minEigF;
  double *eigG, minEigG;
  double *work, *hist;
  
  double *identityF, *identityG;
  double *expandF, *expandG;
  double *newF, *newG;
  int    *newF_blkszs,  newG_blkszs;
  double db0=0.0,t0, *x0,*Z0, W0, *c;
  int    newM, newL, newK, newPkSz,initSize;
  
  int F_pksz=0, G_pksz=0;
  int F_dim=0, G_dim=0;
  int blkSize, maxBlockSize=0;
  int i,m,workSize;
  //int  negativeFlag=TRUE;
    
  eigF = new double[numDOF*2]; // max dimension of torque limit LMI
  eigG = new double[mGrasp->numContacts*7];    // max dimension of friction LMI
  
  m = nullDim;
  newM=m+1;
  newG=(double*)malloc((newM+1)*sizeof(double));
  newG[0]=1.0;
  dcopy(newM,&db0,0,newG+1,1);
  newK=1;
  newG_blkszs=1;
  W0=0.0;
    
  c=(double*)malloc(newM*sizeof(double));
  dcopy(m,&db0,0,c,1);
  c[m]=1.0;
  
  for(i=0;i<L;i++){
    blkSize=F_blkszs[i];
    F_dim+=blkSize;
    F_pksz+=(blkSize+1)*blkSize/2;
    maxBlockSize=MAX(maxBlockSize,blkSize);
  }
  workSize=F_pksz+3*maxBlockSize;
  work=(double*)malloc(workSize*sizeof(double));
  minEigF=eig_val(eigF,F,L,F_blkszs,F_pksz,work);
  free(work);
  
  maxBlockSize=0;
  for(i=0;i<K;i++){
    blkSize=G_blkszs[i];
    G_dim+=blkSize;
    G_pksz+=(blkSize+1)*blkSize/2;
    maxBlockSize=MAX(maxBlockSize,blkSize);
  }
  workSize=G_pksz+3*maxBlockSize;
  work=(double*)malloc(workSize*sizeof(double));
  minEigG=eig_val(eigG,G,K,G_blkszs,G_pksz,work);
  free(work);
  
#ifdef GRASPDEBUG
  fprintf(stderr,"minEigF: %f  minEigG:%f \n",minEigF, minEigG);
  fprintf(stderr," ------- EigF(%d) ------ \n",F_dim);
  disp_mat(stderr,eigF,1,F_dim,0);
  fprintf(stderr," ------- EigG(%d) ------ \n",G_dim);
  disp_mat(stderr,eigG,1,G_dim,0);
#endif
  
  t0=MAX(-1.1*MIN(minEigF,minEigG),1.0);
  x0=(double*)malloc(newM*sizeof(double));
  dcopy(m,&db0,0,x0,1);
  x0[m]=t0;
  
  identityF=(double*)block_Identity(L,F_blkszs,F_pksz);
  identityG=(double*)block_Identity(K,G_blkszs,G_pksz);

#ifdef GRASPDEBUG
  fprintf(stderr,"------------- Identity_F ---------------- \n");
  disp_mat(stderr,identityF,1,F_pksz,0);
  fprintf(stderr,"------------- Identity_G ---------------- \n");
  disp_mat(stderr,identityG,1,G_pksz,0);
#endif
  
  expandF=(double*)malloc((newM+1)*F_pksz*sizeof(double));
  initSize=(m+1)*F_pksz;
  dcopy(initSize,F,1,expandF,1);
  dcopy(F_pksz,identityF,1,expandF+initSize,1);
  free(identityF);
  
  expandG=(double*)malloc((newM+1)*G_pksz*sizeof(double));
  initSize=(m+1)*G_pksz;
  dcopy(initSize,G,1,expandG,1);
  dcopy(G_pksz,identityG,1,expandG+initSize,1);
  free(identityG);
  
  newF=(double*) combineLMIs(newM,expandF,F_pksz,expandG,G_pksz);
  free(expandF); free(expandG);
  
  newL=L+K;
  newF_blkszs=(int*)malloc(newL*sizeof(int));
  for(i=0;i<L;i++)  newF_blkszs[i]=F_blkszs[i];
  for(i=0;i<K;i++)  newF_blkszs[L+i]=G_blkszs[i];
  
  newPkSz=F_pksz+G_pksz;
  Z0=(double*)malloc(newPkSz*sizeof(double));
  dcopy(newPkSz,&db0,0,Z0,1);
  
#ifdef GRASPDEBUG
  fprintf(stderr,"------------- Combined LMI (%d x %d) ---------------- \n", F_pksz+G_pksz, newM+1);
  disp_mat(stderr,newF,F_pksz+G_pksz, newM+1,0);
  fprintf(stderr,"------------- Combined LMI Block Sizes(%d) ---------------- \n", newL);
  disp_imat(stderr,newF_blkszs,1, newL,0);
#endif
  
  fprintf(stderr,"Checking grasping force feasibility...\n");
  hist = maxdet_wrap(newM, newL, newF, newF_blkszs, newK, newG, &newG_blkszs,
		     c, x0,Z0, &W0, gamma, abstol,  reltol, &feasNTiters,
		     negativeFlag,pRstFile);

  if (errorOccurred) {
    free(newF); free(newF_blkszs); free(Z0);
    free(newG); free(c); free(x0);
    delete [] eigF; delete [] eigG;
    return;
  }

  if (initz0) free(initz0);
  initz0=(double *)malloc(nullDim*sizeof(double));
  dcopy(m,x0,1,initz0,1);

  if (feasZHistory) free(feasZHistory);
  feasZHistory=(double*)malloc((m+3)*(feasNTiters)*sizeof(double));
  for(i=0;i<feasNTiters;i++){
    dcopy(newM,hist+i*(newM+3),1,feasZHistory+i*(m+3),1);
    dcopy(2,hist+i*(newM+3)+newM+1, 1, feasZHistory+i*(m+3)+m+1,1);
  }
  if(x0[m]<0) {
    feasible=TRUE;
#ifdef GRASPDEBUG
    printf("\n Feasible! min eig value: %f \n",-x0[m]);
#endif
  }
  else feasible=FALSE;
  
  free(newF); free(newF_blkszs); free(Z0);
  free(newG); free(c); free(x0); free(hist);
  delete [] eigF; delete [] eigG;
}

/*!
  Formulate and solve the optimization 4 problem (See the LMI paper)
*/
void
LMIOptimizer::optm_EffortBarrier()
{
  int    newL, newK;
  double *newF, *newG;
  int    newF_blkszs,  *newG_blkszs;
  double db0=0.0,Z, *W;
  int F_pksz=0, G_pksz=0, newPkSz, i, blkSize;
  int m;
  
  
  m=nullDim;
  if (optmz0) free(optmz0);
  optmz0=(double*)malloc(m*sizeof(double));
  dcopy(m, initz0, 1, optmz0, 1);

  newL=1;
  newF_blkszs=1;
  newF=(double*)malloc((m+1)*sizeof(double));
  newF[0]=1.0;
  dcopy(m,&db0,0,newF+1,1);
  Z=db0;
  
  newK=L+K;
  newG_blkszs=(int*)malloc(newK*sizeof(int));
  for(i=0;i<L;i++){
    blkSize=F_blkszs[i];
    newG_blkszs[i]= blkSize;
    F_pksz+=(blkSize+1)*blkSize/2;
  }
  for(i=0;i<K;i++){
    blkSize=G_blkszs[i];
    newG_blkszs[L+i]= blkSize;
    G_pksz+=(blkSize+1)*blkSize/2;
  }
  newG = (double*) combineLMIs(m,F,F_pksz,G,G_pksz);
  newPkSz=F_pksz+G_pksz;
  W=(double*)malloc(newPkSz*sizeof(double));
  dcopy(newPkSz,&db0,0,W,1);

  if (optmZHistory) free(optmZHistory);
  fprintf(stderr,"Optimizing...\n");
  optmZHistory =  maxdet_wrap(m,newL, newF, &newF_blkszs, newK, newG,
			      newG_blkszs, c, optmz0, &Z, W, gamma, abstol,
			      reltol, &optmNTiters, FALSE, pRstFile);

  if (errorOccurred) {
    free(newF); free(newG_blkszs); free(W);
    free(newG);
    return;
  }
  
  // transform the optimal z value to the optimal x value 
  if (optmx0) free(optmx0);
  optmx0=(double*)malloc(numWrenches*sizeof(double));
  dgemv("N", numWrenches, nullDim, 1.0, nullSpace, numWrenches, 
	optmz0,1, 0.0, optmx0,1);
  daxpy(numWrenches, 1.0, minNormSln, 1, optmx0, 1);

  free(newF); 
  free(newG); free(newG_blkszs); free(W);
}

/*!
  Compute the value of the objective function for each set of z values
  stores the optimal z value and the corresponding objective value
  in \c extendOptmz0.
*/
void
LMIOptimizer::computeObjectives()
{
  double *tempG, *zHistory,objective;
  int    zHistRowDim;
  
  double db0=0.0,minEigG, *eigG, *work;
  int workSize, maxBlockSize=0, lmiDim=0;
  int i,j;

  /* put the initial z value in the first row of the history, then
     copy z history after that */
  zHistRowDim=nullDim+3;
  if (extendOptmZHistory) free(extendOptmZHistory);
  extendOptmZHistory=(double*) malloc(zHistRowDim*(optmNTiters+1)*
					   sizeof(double));
  dcopy(zHistRowDim,initz0,1,extendOptmZHistory,1);
  dcopy(zHistRowDim*optmNTiters,optmZHistory,1,extendOptmZHistory+
	zHistRowDim,1);
  

  zHistory = extendOptmZHistory;

  tempG=(double *)malloc(GPkRow*sizeof(double));
  
  for(i=0;i<K;i++){
    maxBlockSize=MAX(maxBlockSize, G_blkszs[i]);
    lmiDim+=G_blkszs[i];
  }
  workSize=GPkRow+3*maxBlockSize;
  work=(double *)malloc(workSize*sizeof(double));
  eigG=(double *)malloc(lmiDim*sizeof(double));


  for(i=0;i<optmNTiters+1;i++) {
    objective=ddot(nullDim,c,1,zHistory+i*zHistRowDim,1);
    objective+=constOffset;
    
    dcopy(GPkRow,G,1, tempG,1);
    dgemv("N", GPkRow, nullDim, 1.0, G+GPkRow, GPkRow,
	   zHistory+i*zHistRowDim,1,1.0, tempG, 1);
    
    dcopy(workSize,&db0,0,work,1);
    dcopy(lmiDim,&db0,0,eigG,1);
    minEigG=eig_val(eigG,tempG,K,G_blkszs,GPkRow,work);
    if(minEigG<0)	errMsg("G(x) not positive definite");
    for(j=0;j<lmiDim; j++)
      objective -=log(eigG[j]);
    
    zHistory[i*zHistRowDim+nullDim]=objective;
  }
  free(tempG);  free(work); free(eigG);
}


/*!
  Transform z values to values of grasp force x.
*/
double *
LMIOptimizer::xzHistoryTransfrom(double *zHistory,int numIters)
{
  double db0=0.0,*xHistory;
  int xHistSize, xHistRowSize, zHistRowSize, i;


  xHistRowSize=numWrenches+1; 		// X+objective
  xHistSize=xHistRowSize*numIters;
  xHistory=(double*)malloc(xHistSize*sizeof(double));
  dcopy(xHistSize,&db0,0,xHistory, 1);
  
  zHistRowSize=nullDim+3;

  for(i=0;i<numIters;i++){
    dgemv("N", numWrenches, nullDim, 1.0, nullSpace, numWrenches, 
	   zHistory+i*zHistRowSize,1, 0.0, xHistory+i*xHistRowSize,1);
    daxpy(numWrenches, 1.0, minNormSln, 1, xHistory+i*xHistRowSize, 1);
  }
  dcopy(numIters,zHistory+nullDim, zHistRowSize, xHistory+numWrenches,
	xHistRowSize);

  return(xHistory);
}

/*!
  Main GFO routine.
  Adapted from Li Han's code.
*/
int
LMIOptimizer::findOptimalGraspForce()
{
  int i;

  feasible = FALSE;

  if (numWrenches < 6) {
    printf("More contacts are necessary before solving for the optimal grasp force\n");
    return FAILURE;
  }

#ifdef GRASPDEBUG  
  char rstFile[40];
  sprintf(rstFile,"Grasp%02d.rst",graspCounter);
  if ((pRstFile=fopen(rstFile,"w"))==NULL)
    FatalErrMsg("failed to open result file");

  //  graspMap = ReadGraspMap("mytest.G",18);
  printf("---------------- Solve Grasping Force Equation ---------------- \n");
#endif

  minimalNorm();
  // if (optmx0) {free(optmx0); optmx0=NULL;}
  if (errorOccurred) {errorOccurred = false; if (pRstFile) fclose(pRstFile);return FAILURE;}

  computeNullSpace();
  if (errorOccurred) {errorOccurred = false; if (pRstFile) fclose(pRstFile);return FAILURE;}

  if (nullDim == 0) {
    printf("The dimension of the null space is 0. Cannot solve for the optimal grasp force.\n");

    return FAILURE;
  }
 
#ifdef GRASPDEBUG  
  printf("----------- Prepare LMIs for Torque/Contact Limits --------------- \n");
#endif

  if (F) free(F);
  F = lmiTorqueLimits();
 
#ifdef GRASPDEBUG
  printf("----------- Prepare LMIs for Friction Cones --------------- \n");
#endif
  
  if (G) free(G);
  G = lmiFrictionCones();

#ifdef GRASPDEBUG
  printf("--------- Prepare Weight Vectors in the Objective Function ------- \n");
#endif

  if (c) free(c);
  c = weightVec();
  
#ifdef GRAPSDEBUG
  printf("\n -------- The  Feasibility Phase ----------------------- \n");
  fprintf(pRstFile, "\n -------- The  Feasibility Phase ----------------------- \n");
#endif

  feasNTiters=MAX_FEAS_LOOPS;  
  feasibilityAnalysis();
  if (errorOccurred) {errorOccurred = false; if (pRstFile) fclose(pRstFile); return FAILURE;}

  if (feasXHistory) free(feasXHistory);
  feasXHistory=xzHistoryTransfrom(feasZHistory, feasNTiters);

  if(feasible) {
#ifdef GRASPDEBUG
    printf("\n -------- The  Optimization  Phase ----------------------- \n");
    fprintf(pRstFile, "\n -------- The  Optimization  Phase ----------------------- \n");
#endif

    optmNTiters=MAX_OPTM_LOOPS;
    optm_EffortBarrier();
    if (errorOccurred) {feasible = FALSE; errorOccurred = false; if (pRstFile) fclose(pRstFile); return FAILURE;}
    
    computeObjectives();
    if (errorOccurred) {feasible = FALSE; errorOccurred = false; if (pRstFile) fclose(pRstFile); return FAILURE;}

    if (optmXHistory) free(optmXHistory);
    optmXHistory = xzHistoryTransfrom(extendOptmZHistory, optmNTiters+1);
    
    printf("OPTIMAL CONTACT FORCES:\n");
    disp_mat(stdout,optmx0,1,numWrenches,0);

    //    double *testOut = new double[6];
    //    dgemv("N",6,numWrenches,1.0,graspMap,6,optmx0,1,0.0,testOut,1);
    //    printf("CHECK:\n");
    //    for (i=0;i<6;i++)
    //      printf("%15.12le ",testOut[i]);
    //    printf("\n");
    //    delete [] testOut;

    if (optTorques) delete [] optTorques;
    optTorques = new double[numDOF];

    dcopy(numDOF,externalTorques,1,optTorques,1);
    dgemv("T", numWrenches, numDOF, 1.0, Jacobian, numWrenches,
	  optmx0, 1,1.0, optTorques, 1);

    printf("OPTIMAL TORQUES:\n");
    disp_mat(stdout,optTorques,1,numDOF,0);

    int offset = 0;
    for (i=0;i<mGrasp->numContacts;i++) {
      mGrasp->contactVec[i]->getMate()->setContactForce(optmx0+offset);
      offset += mGrasp->contactVec[i]->getContactDim();
    }
  }
  else {
    // prompt that the current grasp force is infeasible.
    printf("not feasible.\n");
#ifdef GRASPDEBUG
    fprintf(pRstFile,"\n --------  Problem Infeasible ----------------------- \n");
#endif
    return FAILURE;
  }


// save the problem parameters, the timing results and the feaibility as well as 
// optimization results at the last simulation step to file argv[1].rst
#ifdef GRASPDEBUG
  fprintf(pRstFile, "-------------- Jacobian (%-dx%-d) -------------- \n",numWrenches,numDOF);
  disp_mat(pRstFile, Jacobian, numWrenches, numDOF, 0);

  fprintf(pRstFile, "-------------- External Torques (%-d)------------------ \n",numDOF);
  disp_mat(pRstFile, externalTorques,1,numDOF,0);
  
  fprintf(pRstFile, "-------------- Grasp Map (6x%-d) -------------- \n", numWrenches);
  disp_mat(pRstFile, graspMap,6, numWrenches,0);
  
  fprintf(pRstFile, "-------------- Object Wrench (6) -------------- \n");
  disp_mat(pRstFile, mGrasp->object->getExtWrenchAcc(), 1, 6, 0);
  
  fprintf(pRstFile, "-------------- Minimal Norm Solution (%-d)------------------ \n",numWrenches);
  disp_mat(pRstFile, minNormSln,1,numWrenches,0);
  
  fprintf(pRstFile, "-------------- Admissible Null Space (%-dx%-d)------------------ \n",numWrenches, nullDim);
  disp_mat(pRstFile, nullSpace, numWrenches, nullDim,0);
  
  fprintf(pRstFile, "-------------- F blkszs (%d)------------- \n",L);
  disp_imat(pRstFile, F_blkszs, 1, L, 0);
  
  fprintf(pRstFile, "-------------- F (%-dx%-d) ------------- \n",L,nullDim+1);
  disp_mat(pRstFile, F, L, nullDim+1,0);
  
  fprintf(pRstFile, "-------------- G blkszs (%d)------------- \n",K);
  disp_imat(pRstFile, G_blkszs, 1, K,0);
  
  fprintf(pRstFile,"-------------- G (%-dx%-d) -------------- \n",GPkRow,
	  nullDim+1);
  disp_mat(pRstFile, G, GPkRow, nullDim+1,0);
  
  fprintf(pRstFile,"-------- Final Weight Vector(%d) and Offset: %f -------\n",
	  nullDim,constOffset);
  disp_mat(pRstFile, c,1,nullDim,0);
  
  fprintf(pRstFile,"------------- Initial Feasible z and History(%dx%d) -------------- \n",
	  3+nullDim,feasNTiters);
  disp_mat(pRstFile, initz0,1,nullDim,0);
  disp_mat(pRstFile, feasZHistory, 3+nullDim, feasNTiters, 0);
  
  fprintf(pRstFile,"-------------  Feasible X History(%dx%d) ---------------- \n",
	  numWrenches+1,feasNTiters);
  disp_mat(pRstFile, feasXHistory, numWrenches+1, feasNTiters, 0);
  
  if(feasible) {
    fprintf(pRstFile,"------------- Optimal z and History(%dx%d)------------------ \n",
	    3+nullDim,optmNTiters+1);
    disp_mat(pRstFile, optmz0,1,nullDim,0);
    disp_mat(pRstFile, extendOptmZHistory, 3+nullDim, optmNTiters+1, 0);
    
    fprintf(pRstFile,"------------- Optimal X and History(%dx%d)------------------ \n",
	    numWrenches+1,optmNTiters+1);
    disp_mat(pRstFile, optmXHistory, numWrenches+1, optmNTiters+1, 0);
  }

  fclose(pRstFile);  
#endif

  return SUCCESS;

}

