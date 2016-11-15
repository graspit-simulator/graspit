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
// $Id: dynamics.cpp,v 1.28 2009/04/02 16:43:25 cmatei Exp $
//
//#####################################################################

/*! \file 
  \brief Implements moveBodies, iterateDynamics, and Lemke's algorithm
 */

#include <stdio.h>
#include <list>
#include <vector>
#include <map>
#include <algorithm>
#ifdef Q_WS_X11
  #include <unistd.h>
#endif
#include "mytools.h"
#include "dynamics.h"
#include "dynJoint.h"
#include "body.h"
#include "robot.h"
#include "contact.h"
#include "world.h"
#include "ivmgr.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

#include "maxdet.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

extern FILE *debugfile;
extern IVmgr *ivmgr;

int myLemke(double *M,int n,double *q,double *z,bool usePrediction,bool ldbg, int &iterations);
//int myNewLemke(double *M,int n,double *q,double *z,bool ldbg);
//int myLemke2(double *M,int n,double *q,double *z,bool ldbg);

void
buildJointConstraints(std::vector<Robot *> &robotVec,
		      std::vector<int> &islandIndices,int numBodies,
		      double* Nu,double *eps,double *H,double *g,double h);

void
assembleLCPPrediction(double *lambda, int Arows, int numDOFLimits, std::list<Contact *> *contactList);

void 
printLCPBasis(double *lambda, int Arows, int numDOFLimits, int numContacts);

/*!
  Copies matrix \a B into a block of matrix \a M. \a ldb is the number of
  rows of \a B, \a r1 and \a r2 are the start and end rows within \a M, and
  \a c1 and \a c2 are the start and end columns in \a M.  \a ldm is the number
  of rows in \a M.
*/
void
fillMatrixBlock(double *B,int ldb,int r1,int c1,int r2,int c2,double *M,
		int ldm)
{
  for (int i=c1;i<=c2;i++)
    for (int j=r1;j<=r2;j++)
      M[i*ldm + j] = B[(i-c1)*ldb + j-r1];
}

/*!
  Creates a 6x6 matrix, \a transformMat, that transforms a wrench expressed
  in one coordinate system to wrench expressed in another.  \a T is the
  transform, and \a p is the new torque origin expressed within the new
  coordinate system.
*/
void buildForceTransform(transf &T,vec3 &p,double *transformMat)
{
  static int j,k;
  static double R[9];
  static double crossMat[9];
  static double Rcross[9];
  static vec3 radius;

  R[0] = T.affine().element(0,0); 
  R[1] = T.affine().element(0,1); 
  R[2] = T.affine().element(0,2); 
  
  R[3] = T.affine().element(1,0); 
  R[4] = T.affine().element(1,1); 
  R[5] = T.affine().element(1,2); 
  
  R[6] = T.affine().element(2,0); 
  R[7] = T.affine().element(2,1); 
  R[8] = T.affine().element(2,2); 
/*
  R[0] = T.affine().element(0,0); 
  R[1] = T.affine().element(1,0); 
  R[2] = T.affine().element(2,0); 
  
  R[3] = T.affine().element(0,1); 
  R[4] = T.affine().element(1,1); 
  R[5] = T.affine().element(2,1); 
  
  R[6] = T.affine().element(0,2); 
  R[7] = T.affine().element(1,2); 
  R[8] = T.affine().element(2,2); 
*/ 

  for (j=0;j<9;j++)
    if (fabs(R[j]) < MACHINE_ZERO) R[j] = 0.0;
    else if (R[j] > 1.0 - MACHINE_ZERO) R[j] = 1.0;
    else if (R[j] < -1.0 + MACHINE_ZERO) R[j] = -1.0;
      
  radius = T.translation() - p;
	
  crossMat[0]=0.0;       crossMat[3]=-radius.z();crossMat[6]=radius.y();
  crossMat[1]=radius.z();crossMat[4]=0.0;        crossMat[7]=-radius.x();
  crossMat[2]=-radius.y();crossMat[5]= radius.x();crossMat[8]=0.0;

  //original graspit
  //dgemm("N","N",3,3,3,1.0,R,3,crossMat,3,0.0,Rcross,3);

  // mtc: new version, I believe this is the correct one
  dgemm("N","N",3,3,3,1.0,crossMat,3,R,3,0.0,Rcross,3);
	
  fillMatrixBlock(R,3,0,0,2,2,transformMat,6);
  for (j=3;j<6;j++)
    for (k=0;k<3;k++) 
      transformMat[6*j+k] = 0.0;
  fillMatrixBlock(Rcross,3,3,0,5,2,transformMat,6);
  fillMatrixBlock(R,3,3,3,5,5,transformMat,6);
}

/*!
  Inverts an n by n matrix.
*/
int
invertMatrix(int n,double *A,double *INVA)
{
  double *work;
  int *ipiv;
  int lwork = n;
  int info;

  if (A && INVA) {
    work = new double[n];  // should we optimize work size?
    ipiv = new int[n];
    dcopy(n*n,A,1,INVA,1);
    dgetrf(n,n,INVA,n,ipiv,&info);
    dgetri(n,INVA,n,ipiv,work,lwork,&info);
    delete [] work;
    delete [] ipiv;
  }
  else {
    printf("One or both matricies in InvertMatrix are NULL\n");
    info = -1;
  }
  return info;
}  

/*! Given a vector of pointers to dynamic bodies, and the number of bodies in
	the vector, this routine will move those bodies in the direction of their
	current velocity for the length of the timestep, \a h. It uses the 
	pre-computed velocities and accelerations computed by iterateDynamics and
	stored for each body. 
*/
int
moveBodies(int numBodies,std::vector<DynamicBody *> bodyVec,double h)
{
  static double V[42];
  static double tmp12[12];
  static double B[12];
  static double R_N_B[9];
  static double newPos[7];
  static mat3 Rot;
  int bn;
  double currq[7];
  double currv[6];
  int errCode=SUCCESS;
  
  for (bn=0;bn<numBodies;bn++) {
    memcpy(currq,bodyVec[bn]->getPos(),7*sizeof(double));
    memcpy(currv,bodyVec[bn]->getVelocity(),6*sizeof(double));;
    
    Quaternion tmpQuat(currq[3],currq[4],currq[5],currq[6]);
    tmpQuat.ToRotationMatrix(Rot);   
    
    // The rotation matrix returned by ToRotationMatrix is expressed as
    // a graphics style rot matrix (new axes are in rows), the R_N_B matrix
    // is a robotics style rot matrix (new axes in columns)
    
    R_N_B[0] = Rot[0];  R_N_B[3] = Rot[1];  R_N_B[6] = Rot[2];
    R_N_B[1] = Rot[3];  R_N_B[4] = Rot[4];  R_N_B[7] = Rot[5];
    R_N_B[2] = Rot[6];  R_N_B[5] = Rot[7];  R_N_B[8] = Rot[8];
    
    // B relates the angular velocity of the body (expressed in
    // the body frame) to the time derivative of the Euler parameters
    B[0] = -currq[4];  B[4] = -currq[5];  B[8] = -currq[6];
    B[1] =  currq[3];  B[5] = -currq[6];  B[9] =  currq[5];
    B[2] =  currq[6];  B[6] =  currq[3];  B[10]= -currq[4];
    B[3] = -currq[5];  B[7] =  currq[4];  B[11]=  currq[3];
    dscal(12,0.5,B,1);
    
    // V is a list of matrices.  Each matrix (V_bn) can be multiplied by
    // body bn's 6x1 velocity vector to get the 7x1 time derivative 
    // of the body's position.
    // V_bn = [ eye(3,3)  zeros(3,3);
    //         zeros(4,3)  B*R_N_B'  ];
    // This list of matrices will be used at the end to compute the new
    // position from the new velocity
    dgemm("N","T",4,3,3,1.0,B,4,R_N_B,3,0.0,tmp12,4);
    V[0] = 1.0;
    V[8] = 1.0;
    V[16]= 1.0;
    fillMatrixBlock(tmp12,4,3,3,6,5,V,7);
    
    dcopy(7,currq,1,newPos,1);
    
    dgemv("N",7,6,h,V,7,currv,1,1.0,newPos,1);
    
#ifdef GRASPITDBG
    fprintf(stdout,"object %s new velocity: \n", bodyVec[bn]->getName().latin1());
    for (int i=0;i<6;i++) fprintf(stdout,"%le   ",currv[i]);
    printf("\n");
    fprintf(stdout,"object %s new position: \n", bodyVec[bn]->getName().latin1());
    disp_mat(stdout,newPos,1,7,0);
#endif
    
    //should we bother to check if the object has moved? (for optimization)
    if (!bodyVec[bn]->setPos(newPos)) {
      DBGP("requested position is out of bounds for this object!");
      errCode = FAILURE;
    }
  }
  return errCode;
}

/*!
  This routine builds a Linear Complementarity Problem (LCP) to solve
  for the velocities of each dynamic body in a given island of connected
  dynamic bodies (connected by joints or contacts).  The robots and bodies
  in the island are passed to the routine in the vectors \a robotVec and
  \a bodyVec, and the integration time step \a h is also provided.  The
  \a useContactEps flag determines whether error correction will be used for
  the contacts to ensure non-interpenetration.  At the end of the
  routine, the velocity of each body is updated, as well as all contact
  and joint forces.
*/

  // From Anitescu and porta 1996 eq 2.13
  // The mixed LCP has the following form:
  // A w + b = z
  //
  //  Our modified version adds some additional constraints they did not
  //  include.  See "Implementation of Multi-rigi-body Dynamics within a
  //  Robotic Grasping Simulator" ICRA 2003 for more details.
  //
  //  [ M  -nu -l  -Wn  -D  0] [v_(l+1)]   [-Mv_(l) - hk]   [  0  ]
  //  [nu'  0   0   0    0  0] [ c_nu  ]   [  -eps_j    ]   [  0  ]
  //  [ l'  0   0   0    0  0] [ c_l   ]   [     0      ]   [ xi  ]
  //  [Wn'  0   0   0    0  0] [ c_n   ] + [  -eps_n    ] = [ rho ]
  //  [ D'  0   0   0    0  E] [ beta  ]   [     0      ]   [sigma]
  //  [ 0   0   0   Mu  -E' 0] [lambda ]   [     0      ]   [zeta ]
  
  // M  is a (6*numBodies x 6*numBodies)  Mass matrix
  // nu is a (6*numBodies x numJointConstraints) Joint constraint matrix
  // l  is a (6*numBodies x numJointLimits) Joint limit constraint matrix
  // Wn is a (6*numBodies x numContacts)  matrix of contact wrenches
  // D  is a (6*numBodies x numContacts*numEdges) friction directions matrix
  // E  is a (numContacts*numEdges x numContacts) matrix of 1's and 0's
  //         relating beta and lambda values to specific contacts
  // MU is a (numContacts x numContacts) coefficient of friction matrix
  
  // To form a pure LCP they eliminate v_(l+1) and c_nu.  They group the
  // mixed LCP into the following blocks (see eq 2.15 and appendix A):
  //
  // [M  -F  -H] [x]   [-k]   [0]
  // [F'  0   0] [y] + [ 0] = [0]
  // [H'  0   N] [l] + [ b] = [s] 
  //
  // eliminating x and y gives the following pure LCP:
  // (G + N) l + g = s    s>=0 l>=0 l's=0
  // where G = H'M_iH - H'M_iF(F'M_iF)_invF'M_iH
  // and   g = H'M_iF(F'M_iF)_invF'M_ik + H'M_ik + b
  
  // In this program, A is the LCP matrix, and N is stored in it, then G
  // is added to it.

  // Note on adding joint spring forces:
  // we assume that some joints have linear springs that extert a force f_k = k * (theta_t - theta_0)
  // the impulse of this force will be c_k = h * f_k
  // at time step t+1 we have 
  // c_k_t+1 = h * k * (theta_t+1 - theta_0) = h * k * ( theta_t+1 - theta_t + theta_t - theta_0)
  // = h * k * delta_theta + c_k_t = J_k * delta_v

int
iterateDynamics(std::vector<Robot *> robotVec,
		std::vector<DynamicBody *> bodyVec,
		DynamicParameters *dp)		
{
  double h = dp->timeStep;
  bool useContactEps = dp->useContactEps;
  static double Jcg_tmp[9],Jcg_B[9],Jcg_N[9],Jcg_N_inv[9],R_N_B[9];
  static double db0=0.0,tmp3[3];
  static mat3 Rot;
  static int info;
  World *myWorld;
  KinematicChain *chain;
  int numBodies = bodyVec.size(),errCode = 0;
  int numRobots = robotVec.size();
  int numJoints=0;
  int numDOF=0;
  int bn,cn,i,j;
  int Mrows,Dcols,Arows,Hrows,Hcols,Nurows,Nucols;
  int numDOFLimits=0;

  std::list<Contact *> contactList;
  std::list<Contact *> objContactList;
  std::list<Contact *>::iterator cp;

  //  unsigned long dmark = dmalloc_mark();

  double *ql = new double[7*numBodies];
  double *qnew = new double[7*numBodies];
  double *vl = new double[6*numBodies];
  double *vlnew = new double[6*numBodies];
  double *M = new double[(6*numBodies)*(6*numBodies)];
  double *M_i = new double[(6*numBodies)*(6*numBodies)];
  double *fext = new double[6*numBodies];

  // LCP matrix
  double *A;

  // LCP vectors
  double *g,*lambda;
  double *predLambda = NULL; //used for debugging the prediction of LCP basis

  // main matrices for contact constraints
  double *H;

  // main matrices for joint constraints
  double *Nu;

  // main vector for contact constraints
  double *k;
  
  // main vectors for joint constraints
  double *eps;

  // intermediate matrices for contact constraints
  double *HtM_i,*v1;

  // intermediate matrices for contact constraints
  double *v2;

  // intermediate matrices for case of both joint and contact constraints
  double *NutM_i,*NutM_iNu,*INVNutM_iNu,*INVNutM_iNuNut;
  double *INVNutM_iNuNutM_i,*INVNutM_iNuNutM_iH;

  // intermediate vectors for case of both joint and contact constraints
  double *NutM_ikminuseps,*INVNutM_iNuNutM_ikminuseps;

  double *currq,*currM;

  Mrows = 6*numBodies;

  myWorld = bodyVec[0]->getWorld();

  std::map<Body*, int> islandIndices;
  for (i=0;i<myWorld->getNumBodies();i++) {
	islandIndices.insert( std::pair<Body*, int>(myWorld->getBody(i), -1) );
  }
  for (i=0;i<numBodies;i++) {
	islandIndices[ bodyVec[i] ] = i;
  }

  // count the joints and DOF, and the joint coupling constraints
  int numCouplingConstraints = 0;
  for (i=0;i<numRobots;i++) {
    numDOF += robotVec[i]->getNumDOF();
    for (j=0;j<robotVec[i]->getNumChains();j++) {
      chain = robotVec[i]->getChain(j);
      numJoints += chain->getNumJoints();
    }
	for (j=0;j<robotVec[i]->getNumDOF();j++) {
	  numCouplingConstraints += robotVec[i]->getDOF(j)->getNumCouplingConstraints();
	  numDOFLimits += robotVec[i]->getDOF(j)->getNumLimitConstraints();
	}
  }

  DBGP("Dynamics time step: " << h);
  DBGP("numJoints: " << numJoints);

  // count the total number of joints and contacts
  int numContacts = 0;
  int numTotalFrictionEdges = 0;
  int numDynJointConstraints=0;
  for (bn=0;bn<numBodies;bn++) {
    //count joints
    if (bodyVec[bn]->getDynJoint()) {
	  int numCon = bodyVec[bn]->getDynJoint()->getNumConstraints();
	  numDynJointConstraints += numCon;
	  DBGP(bodyVec[bn]->getName().latin1() << ": " << numCon << " constraints");
    }
	//count contacts
    objContactList = bodyVec[bn]->getContacts();
    for (cp=objContactList.begin();cp!=objContactList.end();cp++) {
      // check if the mate of this contact is already in the contact list
      if (std::find(contactList.begin(),contactList.end(),(*cp)->getMate()) == contactList.end()) {
		numContacts++;
		numTotalFrictionEdges += (*cp)->numFrictionEdges;
		contactList.push_back(*cp);
      }
	}
  }

  DBGP("Num contacts: " << numContacts);
  DBGP("Num friction edges: " << numTotalFrictionEdges);
  DBGP("Num dynjoint: " << numDynJointConstraints);

  // zero out matrices
  dcopy(Mrows*Mrows,&db0,0,M,1);
  dcopy(Mrows*Mrows,&db0,0,M_i,1);
  dcopy(Mrows,&db0,0,fext,1);

  //allocate the joint constraint matrices
  if (numJoints) {
    Nurows = Mrows;
    Nucols = numDynJointConstraints + numCouplingConstraints;
    DBGP("Nucols: " << Nucols);

    Nu = new double[Nurows * Nucols];
    dcopy(Nurows*Nucols,&db0,0,Nu,1);

    eps = new double[Nucols];
    dcopy(Nucols,&db0,0,eps,1);
    Arows = Mrows+Nucols;
  }
    
  // allocate the LCP matrix
  if (numContacts || numDOFLimits) {
	Dcols = numTotalFrictionEdges;

    DBGP("numContacts " << numContacts);	
    DBGP("Dcols " << Dcols);
    DBGP("numDOFLimits " << numDOFLimits);

    Hrows = Mrows;
    Hcols = Dcols + 2*numContacts + numDOFLimits;
    H = new double[Hrows * Hcols];

    dcopy(Hrows*Hcols,&db0,0,H,1);

    v1 = new double[Hrows * Hcols];
    v2 = new double[Hrows];
    dcopy(Hrows*Hcols,&db0,0,v1,1);
    dcopy(Hrows,&db0,0,v2,1);

    k = new double[Mrows]; //holds mass*previous velocity and external impulses
    Arows = Hcols;
    lambda = new double[Arows];  // the LCP solution    
  } else {
    Dcols = 0;
  }

  // allocate the constraint matrix
  if (numJoints || numContacts) {    
    A = new double[Arows*Arows];
    g = new double[Arows];

    dcopy(Arows*Arows,&db0,0,A,1); 
    dcopy(Arows,&db0,0,g,1); 
  }

  // compute mass matrix and external forces
  for (bn=0;bn<numBodies;bn++) {
	memcpy(vl+6*bn,bodyVec[bn]->getVelocity(),6*sizeof(double));
	memcpy(vlnew+6*bn,bodyVec[bn]->getVelocity(),6*sizeof(double));

    memcpy(ql+7*bn,bodyVec[bn]->getPos(),7*sizeof(double));    
    memcpy(qnew+7*bn,bodyVec[bn]->getPos(),7*sizeof(double));

    currq = qnew + 7*bn;    
    Quaternion tmpQuat(currq[3],currq[4],currq[5],currq[6]);
    tmpQuat.ToRotationMatrix(Rot);   

    // The rotation matrix returned by ToRotationMatrix is expressed as
    // a graphics style rot matrix (new axes are in rows), the R_N_B matrix
    // is a robotics style rot matrix (new axes in columns)
    
    R_N_B[0] = Rot[0];  R_N_B[3] = Rot[1];  R_N_B[6] = Rot[2];
    R_N_B[1] = Rot[3];  R_N_B[4] = Rot[4];  R_N_B[7] = Rot[5];
    R_N_B[2] = Rot[6];  R_N_B[5] = Rot[7];  R_N_B[8] = Rot[8];

    // Jcg_N = R_N_B * Jcg_B * R_N_B'; 
    // where Jcg_B is inertia matrix in body coords
    //       Jcg_N is inertia matrix in world coords ?
    memcpy(Jcg_B,bodyVec[bn]->getInertia(),9*sizeof(double));
	//multiply by mass
	dscal(9, bodyVec[bn]->getMass(), Jcg_B, 1);
    dgemm("N","N",3,3,3,1.0,R_N_B,3,Jcg_B,3,0.0,Jcg_tmp,3);
    dgemm("N","T",3,3,3,1.0,Jcg_tmp,3,R_N_B,3,0.0,Jcg_N,3);

	if ((info = invertMatrix(3,Jcg_N,Jcg_N_inv))) {
      printf("In iterateDynamics, inertia matrix inversion failed (info is %d)\n",info);
	  fprintf(stderr,"%f %f %f\n",Jcg_B[0], Jcg_B[1], Jcg_B[2]);
	  fprintf(stderr,"%f %f %f\n",Jcg_B[3], Jcg_B[4], Jcg_B[5]);
	  fprintf(stderr,"%f %f %f\n",Jcg_B[6], Jcg_B[7], Jcg_B[8]);
	  fprintf(stderr,"Body is %s\n",bodyVec[bn]->getName().latin1());
	}
    
    currM = M+((6*bn)*Mrows + bn*6);  //point to the correct block of M
    
    currM[0]              = bodyVec[bn]->getMass();
    currM[6*numBodies+1]  = bodyVec[bn]->getMass();
    currM[12*numBodies+2] = bodyVec[bn]->getMass();
    fillMatrixBlock(Jcg_N,3,3,3,5,5,currM,Mrows);
  
    currM = M_i+((6*bn)*Mrows + bn*6);//point to correct block of M_i

    currM[0]         = 1.0/bodyVec[bn]->getMass();
    currM[Mrows+1]   = 1.0/bodyVec[bn]->getMass();
    currM[2*Mrows+2] = 1.0/bodyVec[bn]->getMass();
    fillMatrixBlock(Jcg_N_inv,3,3,3,5,5,currM,Mrows);

    // compute external wrench
    // fext = [ 0 0 -9810.0*mass -[ang_vel_N x (Jcg_N * ang_vel_N)] ]
    //based on this, it would appear that graspit force units are N*1.0e6
    //fext[6*bn+2] = -9810.0 * bodyVec[bn]->getMass() * dp->gravityMultiplier;  // force of gravity
    fext[6*bn+2] = 0;  // NO force of gravity

    dgemv("N",3,3,1.0,Jcg_N,3,&vl[6*bn+3],1,0.0,tmp3,1);  // inertial moments
    fext[6*bn+3] = - (vl[6*bn+4]*tmp3[2] - vl[6*bn+5]*tmp3[1]);
    fext[6*bn+4] = - (vl[6*bn+5]*tmp3[0] - vl[6*bn+3]*tmp3[2]);
    fext[6*bn+5] = - (vl[6*bn+3]*tmp3[1] - vl[6*bn+4]*tmp3[0]);

    double ForcesToBodyFrame[36];
    transf invBody = bodyVec[bn]->getTran().inverse();
    vec3 invBodyTransl = invBody.translation();
    buildForceTransform(invBody,invBodyTransl,ForcesToBodyFrame);
	DBGP("fext initial: ");
    DBGST( disp_mat(stdout,&fext[6*bn],1,6,0) );

    // add any other wrenches that have accumulated on the body
    daxpy(6,1.0,bodyVec[bn]->getExtWrenchAcc(),1,&fext[6*bn],1);
	DBGP("fext with accumulated wrench: ");
    DBGST( disp_mat(stdout,&fext[6*bn],1,6,0) );

	if (numContacts||numDOFLimits) {
      // k = Mv_l + hfext
      currM = M+((6*bn)*Mrows + bn*6);  //point to the correct block of M
      dgemv("N",6,6,1.0,currM,Mrows,vl+6*bn,1,0.0,k+6*bn,1);
    }
  }

  if (numJoints) {
    int ncn = 0;
	int hcn = 0;
	for (i=0;i<numBodies;i++) {
	  if (bodyVec[i]->getDynJoint())
		bodyVec[i]->getDynJoint()-> buildConstraints(Nu,eps,numBodies,islandIndices,ncn);
	}
	for (i=0;i<numRobots;i++) {
      robotVec[i]->buildDOFLimitConstraints(islandIndices,numBodies,H,g,hcn);
      robotVec[i]->buildDOFCouplingConstraints(islandIndices,numBodies,Nu,eps,ncn);
	}
	for (i=0;i<Nucols;i++) {
	  eps[i] *= ERP/h;
	}
	for (i=0; i<hcn; i++) {
		g[i] *= ERP/h;
	}
  }

  // add contacts to the LCP
  if (!contactList.empty()) {
    DBGP("processing contacts");
    double Ftform_N_C[36];
    
    // A is square
    double *Wn = &H[numDOFLimits*Hrows];
    double *D  = &H[(numDOFLimits+numContacts)*Hrows];
    
    double *E =		&A[(numDOFLimits+numContacts+Dcols)*Arows + numDOFLimits+numContacts];
    double *negET = &A[(numDOFLimits+numContacts)*Arows + numDOFLimits+numContacts+Dcols]; 
    double *MU    = &A[numDOFLimits*Arows + numDOFLimits+numContacts+Dcols];
    double *contactEps = &g[numDOFLimits];

	int frictionEdgesCount = 0;
    for (cp=contactList.begin(),cn=0; cp!=contactList.end(); cp++,cn++){

      //DBGP("contact " << cn);
      transf cf  = (*cp)->getContactFrame() *  (*cp)->getBody1Tran();
      transf cf2 = (*cp)->getMate()->getContactFrame() * (*cp)->getBody2Tran();

      DBGP("CONTACT DISTANCE: " << (cf.translation() - cf2.translation()).len());
      if (useContactEps) {
            contactEps[cn] = MIN(0.0,-ERP/h *
      			(Contact::THRESHOLD/2.0 - (cf.translation() - cf2.translation()).len()));
	  }
      DBGP(" EPS: " << contactEps[cn]);
      vec3 normal(cf.affine().element(2,0), cf.affine().element(2,1), cf.affine().element(2,2));
        
      // find which body is this contact from
      for (bn=0;bn<numBodies;bn++)
	    if ((*cp)->getBody1() == bodyVec[bn]) break;
      if (bn<numBodies) {
		//????? this doesn't seem correct
       	vec3 radius = cf.translation() - ( bodyVec[bn]->getCoG() * (*cp)->getBody1Tran() - position::ORIGIN );

	    //	radius = radius / 1000.0;  // convert to meters

		vec3 RcrossN = radius * normal;
		DBGP("body1 normal: " << normal);
		DBGP("body1 radius: " << radius);

		Wn[cn*Hrows+6*bn]   = normal.x();
		Wn[cn*Hrows+6*bn+1] = normal.y();
		Wn[cn*Hrows+6*bn+2] = normal.z();
		Wn[cn*Hrows+6*bn+3] = RcrossN.x();
		Wn[cn*Hrows+6*bn+4] = RcrossN.y();
		Wn[cn*Hrows+6*bn+5] = RcrossN.z();
	
		vec3 bodyOrigin = bodyVec[bn]->getCoG() * (*cp)->getBody1Tran() - position::ORIGIN;
		buildForceTransform(cf,bodyOrigin,Ftform_N_C);

		/* dgemm("N","N", 6,Contact::numFrictionEdges,6, 1.0,Ftform_N_C,6, Contact::frictionEdges,6,
			    0.0,&D[Contact::numFrictionEdges*cn*Hrows+6*bn],Hrows); */
				
		dgemm("N","N",
				6,(*cp)->numFrictionEdges,6,  //m, n, k
				1.0,Ftform_N_C,6,			 //alfa, A, lda
				(*cp)->frictionEdges,6,		//B, ldb
			    0.0,&D[ frictionEdgesCount*Hrows+6*bn],Hrows);	//beta, C, ldc
	  }

      //find the other body
      for(bn=0;bn<numBodies;bn++)
		if ((*cp)->getBody2() == bodyVec[bn]) break;
      if (bn<numBodies) {

        //normal = vec3(cf2.affine().element(2,0), cf2.affine().element(2,1),cf2.affine().element(2,2));
		normal = -normal;

		//vec3 radius = cf2.translation() - (bodyVec[bn]->getCoG() * (*cp)->getBody2Tran() - position::ORIGIN);
		vec3 radius = cf.translation() - (bodyVec[bn]->getCoG() * (*cp)->getBody2Tran() - position::ORIGIN);
		vec3 RcrossN = radius * normal;
		DBGP("body2 normal: " << normal);
		DBGP("body2 radius: " << radius);

		Wn[cn*Hrows+6*bn]   = normal.x();
		Wn[cn*Hrows+6*bn+1] = normal.y();
		Wn[cn*Hrows+6*bn+2] = normal.z();
		Wn[cn*Hrows+6*bn+3] = RcrossN.x();
		Wn[cn*Hrows+6*bn+4] = RcrossN.y();
		Wn[cn*Hrows+6*bn+5] = RcrossN.z();
	
		vec3 bodyOrigin = bodyVec[bn]->getCoG()*(*cp)->getBody2Tran() - position::ORIGIN;
		buildForceTransform(cf,bodyOrigin,Ftform_N_C);
		//buildForceTransform(cf2,bodyOrigin,Ftform_N_C);

/*		dgemm("N","N",6,Contact::numFrictionEdges,6,-1.0,Ftform_N_C,6, Contact::frictionEdges,6,
			  0.0,&D[Contact::numFrictionEdges*cn*Hrows+6*bn],Hrows);*/
		//original graspit had a -1.0 here in front of Ftform_N_C
		dgemm("N","N",
				6,(*cp)->numFrictionEdges,6,
				-1.0,Ftform_N_C,6,
				(*cp)->frictionEdges,6,
				0.0,&D[ frictionEdgesCount*Hrows+6*bn ],Hrows);
      }

      //for (i=cn*Contact::numFrictionEdges; i<(cn+1)*Contact::numFrictionEdges; i++) {
	  for (i=frictionEdgesCount; i<frictionEdgesCount+(*cp)->numFrictionEdges; i++) {
		E[cn*Arows+i] = 1.0;
		negET[i*Arows+cn] = -1.0;
      }      
      MU[cn*Arows + cn] = (*cp)->getCof();
	  frictionEdgesCount += (*cp)->numFrictionEdges;
    }
  }
  
  if (numContacts || numDOFLimits)
    daxpy(Mrows,h,fext,1,k,1);

  if (numJoints && (numContacts || numDOFLimits)) {
    // Cnu1 = INV(Nu'M_iNu)Nu'M_iH
    // Cnu2 = INV(Nu'M_iNu)(Nu'M_ik-eps)
    // v1 = -NuCnu1
    // v2 = -NuCnu2
    
    NutM_i = new double[Nucols*Mrows];
    NutM_iNu = new double[Nucols*Nucols];
    INVNutM_iNu = new double[Nucols*Nucols];
    INVNutM_iNuNut = new double[Nucols*Nurows];
    INVNutM_iNuNutM_i = new double[Nucols*Mrows];
    INVNutM_iNuNutM_iH = new double[Nucols*Hcols];
    

    NutM_ikminuseps = new double[Nucols];
    INVNutM_iNuNutM_ikminuseps = new double[Nucols];
    
    dgemm("T","N",Nucols,Mrows,Mrows,1.0,Nu,Nurows,M_i,Mrows, 0.0,NutM_i,Nucols);
    dgemm("N","N",Nucols,Nucols,Mrows,1.0,NutM_i,Nucols,Nu,Nurows, 0.0,NutM_iNu,Nucols);
    if ((info = invertMatrix(Nucols,NutM_iNu,INVNutM_iNu)))
      printf("In iterateDynamics, NutM_iNu matrix inversion failed (info is %d)\n",info);
    
    dgemm("N","T",Nucols,Nurows,Nucols,1.0,INVNutM_iNu,Nucols,Nu,Nurows,
	  0.0,INVNutM_iNuNut,Nucols);
    dgemm("N","N",Nucols,Mrows,Mrows,1.0,INVNutM_iNuNut,Nucols,M_i,Mrows,
	  0.0,INVNutM_iNuNutM_i,Nucols);
    dgemm("N","N",Nucols,Hcols,Mrows,1.0,INVNutM_iNuNutM_i,Nucols,H,Hrows,
	  0.0,INVNutM_iNuNutM_iH,Nucols);
    dgemm("N","N",Nurows,Hcols,Nucols,-1.0,Nu,Nurows,INVNutM_iNuNutM_iH,Nucols,
	  0.0,v1,Nurows);

    dgemv("N",Nucols,Mrows,1.0,NutM_i,Nucols,k,1,0.0,NutM_ikminuseps,1);
    daxpy(Nucols,-1.0,eps,1,NutM_ikminuseps,1);

    dgemv("N",Nucols,Nucols,1.0,INVNutM_iNu,Nucols,NutM_ikminuseps,1,
	  0.0,INVNutM_iNuNutM_ikminuseps,1);

    dgemv("N",Nurows,Nucols,-1.0,Nu,Nurows,INVNutM_iNuNutM_ikminuseps,1,
	  0.0,v2,1);
  }

  if (numContacts || numDOFLimits) {
    // in the simple case without joint constraints
    // A = H'M_iv1 + N
    // g = H'M_iv2
    // where N is already stored in A
    // v1 is the first term of v_(l+1) and v2 is the second term
    // v_l+1 = M_i(v1 lambda + v2) = M_i(H lambda + k)
    // k is (Mv_l + hfext)

    //add H to v1
    //add k to v2
    DBGP("k:");
    DBGST( disp_mat(stdout,k,1,Mrows,0) );
    DBGP("first g:");
    DBGST( disp_mat(stdout,g,1,Arows,0) );

	daxpy(Mrows*Hcols,1.0,H,1,v1,1);
    daxpy(Mrows,1.0,k,1,v2,1);

    // build A and g
    HtM_i = new double[Hcols*Mrows];
    dgemm("T","N",Hcols,Mrows,Hrows,1.0,H,Hrows,M_i,Mrows,0.0,HtM_i,Hcols);

    dgemm("N","N",Hcols,Hcols,Mrows,1.0,HtM_i,Hcols,v1,Mrows,1.0,A,Arows);
    //    dgemv("N",Hcols,Mrows,1.0,HtM_i,Hcols,v2,1,0.0,g,1);
    dgemv("N",Hcols,Mrows,1.0,HtM_i,Hcols,v2,1,1.0,g,1);
  }

	int frictionEdgesCount;
	//debug information; can be removed

	if (numContacts || numDOFLimits) {
		bool lemkePredict = false;
		if (lemkePredict) {
			//try to use information from previous time steps to guess a good starting basis for Lemke's algorithm
			assembleLCPPrediction(lambda, Arows, numDOFLimits, &contactList);
	        predLambda = new double[Arows];  // keep a copy of the prediction so we can check it later
			dcopy(Arows, lambda, 1, predLambda, 1);
//			fprintf(stderr,"Prediction: \n");
//			printLCPBasis(predLambda, Arows, numDOFLimits, numContacts);
		}

	    //    double startTime;   
	    //    startTime = getTime();
   
		DBGP("g:");
		DBGST( for (i=0;i<Arows;i++) printf("%le ",g[i]); );
		DBGP("\n");

		double CFM = 0.0;
		int lcperr = 1, lcpIter;
		while ( CFM < 1.0e-7) {
			lcperr = myLemke(A, Arows, g, lambda, lemkePredict, false, lcpIter);
			if (!lcperr) break;
			if (CFM == 0) CFM = 1.0e-11;
			else CFM *= 10.0;
			for (i=0;i<Arows;i++) A[i*Arows+i]+= CFM;
			DBGP("Lemke failed, re-run with CFM " << CFM);
		}
/*
		// debug for lemke predictions
		fprintf(stderr,"Standard result: \n");
		printLCPBasis(lambda, Arows, numDOFLimits, numContacts);      
		fprintf(stderr,"Prediction result: \n");
		printLCPBasis(predLambda, Arows, numDOFLimits, numContacts);      

		int predScore = 0;
		int disagreements = 0;
		for (i=0; i<Arows; i++) {
			if ( (lambda[i] > 0) != (predLambda[i] > 0) ) disagreements++;
			if ( predLambda[i] > 0 ) {
				if (lambda[i] > 0)
					predScore ++;
				else
					predScore --;
			}
		}
		fprintf(stderr,"Disagreements: %d / Prediction score: %d\n\n", disagreements, predScore);
*/

		if (!lcperr) {
			// Once we have solved for lambda it can be plugged back into
			// the elimated equations to get the new velocity vector
      
			double contactForce[6];
			double invh = 1.0/h;
			dgemv("N",Hrows,Hcols,1.0,v1,Hrows,lambda,1,1.0,v2,1);
			dgemv("N",Hrows,Hrows,1.0,M_i,Hrows,v2,1,0.0,vlnew,1);
      
			// retrieve contact wrench - only used for visualization purposes
			//----------------careful now - adding the components back
			frictionEdgesCount = 0;
			for (cp=contactList.begin(),cn=0;cp!=contactList.end();cp++,cn++){

				//save LCP basis results for this contact for future use				
				(*cp)->setLCPInfo(lambda[numDOFLimits+cn], 
								  lambda[Arows - numContacts + cn],
								  &lambda[numDOFLimits + numContacts + frictionEdgesCount] );

				/*dgemv("N", 6,Contact::numFrictionEdges, invh,Contact::frictionEdges,6,
				&lambda[numDOFLimits+numContacts+cn*Contact::numFrictionEdges],1, 0.0,contactForce,1); */
				dgemv("N",
					6,(*cp)->numFrictionEdges, // m,n
					invh,(*cp)->frictionEdges,6,  //alpha, A, lda
					&lambda[ numDOFLimits + numContacts + frictionEdgesCount ], 1, // x, incx
					0.0,contactForce,1); //beta, y, incy
	
				contactForce[2] += lambda[numDOFLimits+cn]/h;

				dscal(3,1.0e-6,contactForce,1);    // convert to newtons
				dscal(3,1.0e-9,contactForce+3,1);  // convert to newton meters
	
				(*cp)->setDynamicContactWrench(contactForce);
				//cnl-updating friction edges (eg contact size, given new Dynamic force
				//cnl-hack-save old dynamic force in body so that new
				//contact area can be calculated for new contact
				//(*cp)->getBody1()->setDynCForce( contactForce,  );
				(*cp)->setUpFrictionEdges(true);
	
				dscal(6,-1.0,contactForce,1); 
				transf cf = (*cp)->getContactFrame() *  (*cp)->getBody1Tran();
				transf cf2 = (*cp)->getMate()->getContactFrame() * (*cp)->getBody2Tran();
				vec3 cvec(contactForce);
				cvec = cvec * cf * cf2.inverse();	
				contactForce[0] = cvec[0];
				contactForce[1] = cvec[1];
				contactForce[2] = cvec[2];
				(*cp)->getMate()->setDynamicContactWrench(contactForce);
				//cnl-updating friction edges (eg contact size, given new Dynamic force
				(*cp)->getMate()->setUpFrictionEdges(true);

				frictionEdgesCount += (*cp)->numFrictionEdges;
			}
      
#ifdef GRASPITDBG
			printf("Lambda is:\n");
			for (i=0;i<Arows;i++) {
				printf("%le ",lambda[i]);
				if (lambda[i] < -MACHINE_ZERO) {
					printf ("NEGATIVE LAMBDA!!!\n");
				}
			}
			printf("\n");
		    printf("Alambda+g = \n");
			dgemv("N",Arows,Arows,1.0,A,Arows,lambda,1,1.0,g,1);
			for (i=0;i<Arows;i++)
				printf("%le ",g[i]);
			printf("\n");
			if (numJoints){
				double *cnu= new double[Nucols];
				printf("Cnu:\n");
				dgemv("N",Nucols,Hcols,-1.0,INVNutM_iNuNutM_iH,Nucols,lambda,1,0.0,cnu,1);
				daxpy(Nucols,-1.0,INVNutM_iNuNutM_ikminuseps,1,cnu,1);
				disp_mat(stdout,cnu,1,Nucols,0);
				delete [] cnu;
			}
#endif

		}
		else errCode = 1;  // lcperr
	}
  else if (numJoints) {
    DBGP("solving with inversion");
    // A x = g
    // [M -Nu] [v_l+1]     [Mv_l + hk]
    // [Nu' 0] [c_nu ]  =  [  eps    ]

    fillMatrixBlock(M,Mrows,0,0,Mrows-1,Mrows-1,A,Arows);
    for (i=0;i<Mrows;i++)
      for (j=Mrows;j<Arows;j++) {
	A[i*Arows + j] = Nu[(j-Mrows)*Nurows + i];
	A[j*Arows + i] = -Nu[(j-Mrows)*Nurows + i];
      }

    for (i=0;i<Arows;i++)
      for(j=0;j<Arows;j++)
	if (fabs(A[i*Arows+j]) < MACHINE_ZERO) A[i*Arows+j] = 0.0;
    
    dgemv("N",Mrows,Mrows,1.0,M,Mrows,vl,1,0.0,g,1); //why was it M_inv??
    daxpy(Mrows,h,fext,1,g,1);
    dcopy(Nucols,eps,1,g+Mrows,1);

#ifdef GRASPITDBG
    printf("g:\n");
    disp_mat(stdout,g,1,Arows,0); 
    printf("h: %le\n",h);
#endif
    //Invert A
    int *ipiv = new int[Arows];
    dgesv(Arows,1,A,Arows,ipiv,g,Arows,&info);
    if (info != 0){
      printf("In iterateDynamics, A matrix inversion failed (info is %d)\n",info);
      std::cerr << "matrix solve failed"<< std::endl;
    }

    delete [] ipiv;
#ifdef GRASPITDBG
    printf("lambda:\n");
    disp_mat(stdout,g,1,Arows,0);
#endif
    dcopy(Mrows,g,1,vlnew,1);

  }
  // if we have no constraints, just use euler step to get next velocity
  else{
    // vl = h*M_i*k + vl;
    dgemv("N",Mrows,Mrows,h,M_i,Mrows,fext,1,1.0,vlnew,1);
  }

  //compute acceleration
  daxpy(6*numBodies,-1,vlnew,1,vl,1);
  dscal(6*numBodies,-1.0/h,vl,1);

  for (i=0;i<numBodies;i++) {    
    bodyVec[i]->setAccel(&vl[6*i]);
    bodyVec[i]->setVelocity(&vlnew[6*i]);    
  }
  
  
  if (numJoints || numContacts) {
    delete [] A;
    delete [] g;
  }
  
  if (numJoints) {
    delete [] Nu;
    delete [] eps;
    
    if (numContacts || numDOFLimits) {
      delete [] NutM_i;
      delete [] NutM_iNu;
      delete [] INVNutM_iNu;
      delete [] INVNutM_iNuNut;    
      delete [] INVNutM_iNuNutM_i;
      delete [] INVNutM_iNuNutM_iH;
      
      delete [] NutM_ikminuseps;
      delete [] INVNutM_iNuNutM_ikminuseps;

    }
  }
  
  if (numContacts || numDOFLimits) {

    delete [] H;
    delete [] HtM_i;
    delete [] v1;
    delete [] v2;
    delete [] lambda;
	if (predLambda) delete [] predLambda;
    delete [] k;
  }

  delete [] ql;
  delete [] qnew;
  delete [] vl;
  delete [] vlnew;
  delete [] M;
  delete [] M_i;
  delete [] fext;

  return errCode;  
  }

  /*! Attempts to use information from the solution at the previous time step,
	stored in each contact, to bootstrap the LCP at the current step. In theory,
	this could help a lot and all the framework is here, we just never got it
	to actually do much.
*/
void
assembleLCPPrediction(double *lambda, int Arows, int numDOFLimits, std::list<Contact *> *contactList) 
{
	double db0 = 0.0;
	dcopy(Arows, &db0, 0, lambda, 1);
	  
    std::list<Contact *>::iterator cp;
	int numContacts = contactList->size();
	int frictionEdgesCount = 0,cn;

	for (cp=contactList->begin(),cn=0;cp!=contactList->end();cp++,cn++){
		if ( (*cp)->inherits() ) {
			lambda[numDOFLimits+cn] = (*cp)->getPrevCn();
			lambda[Arows - numContacts + cn] = (*cp)->getPrevLambda();
			memcpy(&lambda[numDOFLimits + numContacts + frictionEdgesCount],
				(*cp)->getPrevBetas(), (*cp)->numFrictionEdges * sizeof(double) );
		}
		frictionEdgesCount += (*cp)->numFrictionEdges;
	}
}

void 
printLCPBasis(double *lambda, int Arows, int numDOFLimits, int numContacts)
{
	int i,mz;
	fprintf(stderr,"[");
	for (i=0; i < Arows; i++) {
		mz = 0;
		if (lambda[i] > 0) mz = 1;
		if (i == numDOFLimits || i == numDOFLimits + numContacts || i == Arows - numContacts)
			fprintf(stderr," |");
		fprintf(stderr," %d",mz);
	}
	fprintf(stderr," ]\n");
}

/* simple bubble sort for a vector*/
void sortVector(int *v, int n)
{
	bool done = false;
	while (!done) {
		done = true;
		for (int i=1; i<n; i++) {
			if (v[i] < v[i-1]) {
				double tmp = v[i];
				v[i] = v[i-1];
				v[i-1] = tmp;
				done = false;
			}
		}
	}
}

/*!
  Implementation of Lemke's algorithm for solving pure LCP's.  \a M and \a q
  are the matrix and vector that define the LCP.  \a n is the number of rows
  in \a M and \a q, and \a z is the result.  \a usePrediction is a flag that 
  tells the algorithm to use as a starting basis the initial value of \a z. 
  If it is set to false, the initial value of \a z is ignored. \a ldbg is a 
  debug flag.
*/
int
myLemke(double *M,int n,double *q,double *z, bool usePrediction, bool ldbg, int &iterations)
{
  //     double zer_tol = 1.0e-5;
  //    double piv_tol = 1.0e-8;
	double zer_tol = 1.0e-11;
    double piv_tol = 1.0e-13;

	int maxiter = MIN(1000, 25*n);
	int i,k,info,iter,err;
	int *ipiv,*bas,*j;
	double db0=0.0;
	double *tmpZ,*B,*tmpB,*Be,*x,*d;
	double rowSum,theta,tval,ratio;
	int t,entering,leaving,lvindex;

	iterations = 0;

	for (i=0;i<n && q[i]>=0.0;i++)
		z[i] = 0.0;
	if (i==n) return 0;  // Trivial solution (z=0)

	int *possibleLVindex = new int[n];
	tmpZ = new double[2*n];
	dcopy(2*n,&db0,0,tmpZ,1);

	j = new int[n];
	for (i=0;i<n;i++) j[i] = 0;

	bas = new int[n];
	B = new double[n*n];
	tmpB = new double[n*n];
	x = new double[n];
	ipiv = new int[n];

	// Determine initial values
	if (!usePrediction) {
		//bas = ( n+1 : 2*n )
		for (i=n;i<2*n;i++) bas[i-n] = i;
		// B = -I
		dcopy(n*n,&db0,0,B,1);
		for (i=0;i<n;i++) 
			B[i*n+i] = -1.0;
		//x = q
		dcopy(n,q,1,x,1);
	} else {
		// let B = -I but pick columns from M where z[i] > 0

		// bas = [ find(z0>0) ; n+find(z0<=0) ]
		for (i=0;i<n;i++)  {
			if ( z[i] > 0 )
				bas[i] = i;
			else 
				bas[i] = n+i;
		}
		sortVector(bas, n);
/*
		printf("bas:\n");
		for (i=0;i<n;i++) 
			printf("%d ",bas[i]);
		printf("\n");
*/
		// B = [M -I]; B = (:,bas)
		dcopy(n*n, &db0, 0, B, 1);
		for (i=0; i<n; i++) {
			if ( bas[i] >= n ) {
				B[ i*n + bas[i]-n ] = -1;
			} else {
				for (k=0; k<n; k++)
					B[ i*n + k ] = M[ bas[i]*n + k];			
			}
		}

		// x = - (B\q)
		dcopy(n,q,1,x,1);
		dcopy(n*n, B, 1, tmpB, 1);
		dgesv(n, 1, tmpB, n, ipiv, x, n, &info);
		for (i=0; i<n; i++)
			x[i] = -1.0 * x[i];
		if (info!=0)
			fprintf(stderr,"Inversion failed when computing initial basis from user-supplied starting point!\n");
	}
/*
	fprintf(stderr,"M is:\n");
	disp_mat(stderr,M,n,n,0);
	fprintf(stderr,"B is:\n");
	disp_mat(stderr,B,n,n,0);
*/
	// Check if initial basis provides solution
	for (i=0;i<n && x[i] >= 0.0;i++)
		tmpZ[bas[i]] = x[i];
    
	if (i==n) {    
		dcopy(n,tmpZ,1,z,1);
		delete [] tmpZ; delete [] j; delete [] bas; delete [] B; delete [] x;
		delete [] tmpB; delete [] ipiv; delete [] possibleLVindex;
		return 0;
	}

	// Artificial variable is the first entering variable
	t = 2*n;
	entering = t;

	// Determine initial leaving variable
	tval = x[0];
	lvindex = 0;
	for (i=1;i<n;i++) {
		if (x[i] < tval) {
			tval = x[i];
			lvindex = i;
		}
	}
	tval = -tval;
	leaving = bas[lvindex];

  // pivot in the artificial variable
	bas[lvindex] = t;
	for (i=0;i<n;i++) {
		x[i] = x[i] + tval; 
		rowSum = 0.0;
		for (k=0;k<n;k++)
			rowSum += B[k*n+i];
	    B[lvindex*n+i] = -rowSum;
	}
	x[lvindex] = tval;

	Be = new double[n];
	d = new double[n];

#ifdef LEMKE_DBG
	if (ldbg) {
		printf("bas:\n");
		for (i=0;i<n;i++) 
			printf("%d ",bas[i]);
		printf("\n");
	}
#endif

	// Main iterations starts here
	for (iter=1;iter<maxiter;iter++) {
		iterations = iter;
		//fprintf(stderr,"Iteration: %d\n",iter);
		//    getchar();
		if (leaving == t) break;
		if (leaving < n) {
			entering = n+leaving;
			dcopy(n,&db0,0,Be,1);
			Be[leaving] = -1.0;
		}
		else {
			entering = leaving - n;
			dcopy(n,&M[n*entering],1,Be,1);
		}

#ifdef LEMKE_DBG
		if (ldbg) {
			printf("entering: %d\n",entering);
			//    printf("Be:\n");
			//    disp_mat(stdout,Be,1,n,0);
		}
#endif

		dcopy(n,Be,1,d,1);
		dcopy(n*n,B,1,tmpB,1);
		dgesv(n,1,tmpB,n,ipiv,d,n,&info);

		//if (info != 0)
		//	printf("!!!!!!!!!!!!!!!! 2-INFO is %d iter is %d!!!!!!!!!!!!!\n",info,iter);

#ifdef LEMKE_DBG
		if (ldbg) {
			//    printf("x:\n");
			//    disp_mat(stdout,x,1,n,0);
			printf("d:\n");
			disp_mat(stdout,d,1,n,0);
		}
#endif

		int definite=-1;
		double rat;
		// Find new leaving variable
		err = 2;
		theta = HUGE_VAL;
		for (i=0;i<n;i++) {
			if (ldbg) 
				printf("i=%d x: %15.12le d: %15.12le",i,x[i],d[i]);
			if (d[i] > piv_tol) {
				//	j[i] = i;
				err=0;
				rat = (x[i]+zer_tol)/d[i];
				if (rat < theta) {
					theta=rat;
					definite = i;
					if (ldbg) printf("theta: %15.12le",theta);
				}
			}
			if (ldbg) printf("\n");
			//      else j[i] = -1;
			j[i] = -1;
		}
    
		if (err) {
			//printf("ray termination\n");
			delete [] tmpZ; delete [] j; delete [] bas; delete [] B; delete [] Be;
			delete [] x; delete [] d; delete [] ipiv; delete [] tmpB;
			delete [] possibleLVindex;
			return err;  // no new pivots - ray termination
		}
    
		for (i=0;i<n;i++) {
			if (d[i] > piv_tol) {
				if (theta - x[i]/d[i] > -zer_tol) j[i] = i;
			}
		}

		j[definite] = definite;
    
#ifdef LEMKE_DBG    
		if (ldbg) {
			printf("theta: %lf\n",theta);
			printf("j: ");
			for (i=0;i<n;i++) printf("%d ",j[i]);
			printf("\n");
	    }
#endif

		// check if artificial among these
		lvindex = -1;
		for (i=0;i<n;i++)
			if (j[i] >= 0 && bas[i] == t) lvindex = i;

		// Always use artificial if possible
		if (lvindex > -1) {
			lvindex = j[lvindex];
			if (ldbg) printf("artificial found\n");
		}
		// ATM: This is the strangest part of the matlab algorithm
		// basically it finds how many d[i]'s match the max theta
		// but then it doesn't use the index of that element, it	
		// uses the first valid j value or if there is more than one match
		// it chooses one of the j values at random from the among the
		// first "count" valid j values.  For now I'll just always choose
		// the first valid j value.
		//
		// matlab statements:
		//  else                           % otherwise pick among set of max d
		//    theta=max(d(j))
		//    lvindex=find(d(j)==theta)
		//    lvindex=j(ceil(length(lvindex)*rand))% if multiple choose randomly
		//  end
    
		else {     // otherwise pick among set of max d
			theta = -HUGE_VAL;
			for (i=0;i<n;i++) {
				if (j[i] >= 0 && d[i] > theta) theta = d[i];
			}

			int count=0;
			for (i=0;i<n;i++) {
				if (j[i] >= 0 && fabs(d[i] - theta) < zer_tol) {
					possibleLVindex[count++] = j[i];
				}
			}
      
			if (count > 1) {
				// if multiple choose randomly
				count = (int)((double)rand()/(RAND_MAX+1.0)*count);	
				lvindex = possibleLVindex[count];
			}
			else lvindex = possibleLVindex[0];
		}
		leaving = bas[lvindex];

#ifdef LEMKE_DBG
		if (ldbg) {
			printf("lvindex: %d\n",lvindex);
			printf("theta: %lf\n",theta);
			printf("leaving: %d\n",leaving);
		}
#endif
    
		// perform pivot
		ratio = x[lvindex]/d[lvindex];
		daxpy(n,-ratio,d,1,x,1);
		x[lvindex] = ratio;
		dcopy(n,Be,1,&B[lvindex*n],1);
		bas[lvindex] = entering;

#ifdef LEMKE_DBG
		if (ldbg) {
			printf("ratio: %lf\n",ratio);
      //    printf("x:\n");
      //    disp_mat(stdout,x,1,n,0);
      //    printf("B:\n");
      //    disp_mat(stdout,B,n,n,0);
			printf("bas:\n");
			for (i=0;i<n;i++) printf("%d ",bas[i]);
			printf("\n");
		}
#endif    
	}

	if (iter==maxiter) {
		DBGP("Max iterations reached in myLemke");
		delete [] tmpZ; delete [] j; delete [] bas; delete [] B; delete [] Be;
		delete [] x; delete [] d; delete [] ipiv; delete [] tmpB;
		delete [] possibleLVindex;
		return 1;
	}
  
	for (i=0;i<n;i++) {
		if (bas[i] < n) {
			tmpZ[bas[i]] = x[i];
			// if (x[i] < -MACHINE_ZERO) printf("z[%d] = x[%d] is %le!!!\n",bas[i],i,x[i]);
		}
	}

	dcopy(n,tmpZ,1,z,1);

	dcopy(n,q,1,tmpZ,1);
#ifdef LEMKEDBG
	printf("q:\n");
	disp_mat(stdout,q,1,n,0);
#endif
	dgemv("N",n,n,1.0,M,n,z,1,1.0,tmpZ,1);
#ifdef LEMKEDBG
	printf("Mz+q:\n");
	disp_mat(stdout,tmpZ,1,n,0);
#endif

	for (i=0;i<n;i++)
		if (tmpZ[i] < -100.0*MACHINE_ZERO) {
			// printf("Alambda+g[%d] is %le!!!\n",i,tmpZ[i]);
			delete [] tmpZ; delete [] j; delete [] bas; delete [] B; delete [] Be;
			delete [] x; delete [] d; delete [] ipiv; delete [] tmpB;
			delete [] possibleLVindex;
			return 1;
		}

  
	delete [] tmpZ; delete [] j; delete [] bas; delete [] B; delete [] Be;
	delete [] x; delete [] d; delete [] ipiv; delete [] tmpB;
	delete [] possibleLVindex;
	return 0;
}

