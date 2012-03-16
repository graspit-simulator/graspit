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
// Author(s):  Matei Ciocarlie 
//
// $Id: mcGrip.cpp,v 1.8 2009/09/12 00:14:37 cmatei Exp $
//
//######################################################################

#include "mcGrip.h"

#include <limits>

#include "matrix.h"
#include "debug.h"

/*! Also sets the construction parameters which for now are fixed 
	and hard-coded in.

	Initializes instance of Grap class to the specialized version
	McGripGrasp
*/
McGrip::McGrip(World *w,const char *name) : HumanHand(w,name) 
{
	mLinkLength = 20;
	mJointRadius = 5;

	delete grasp;
	grasp = new McGripGrasp(this);
}

/*! The tendons that we model here are not connected to spring-like
	muscles and we assume there is no passive force.
*/
int 
McGrip::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = HumanHand::loadFromXml(root, rootPath);
	for (size_t i=0; i<(unsigned int)mTendonVec.size(); i++) {
		mTendonVec[i]->setApplyPassiveForce(false);
	}
	return result;
}

/*	This function adds up the contributions of the insertion points on the 
	link after \a thisJoint to the joint \a refJoint. The joint that follows 
	\a thisJoint in the chain is \a nextJoint, or -1 if \a thisJoint is the 
	last in the	chain.

	\a thisJoint is the key to the computation. All we need from \a refJoint is
	the translation to get to \a thisJoint(\a tx and \a ty), and all we need 
	from \a nextJoint is its angle (\a theta_n).

	All the math in here is derived in the notes, see there for more details.
*/
	
void 
McGrip::assembleTorqueMatrices(int refJointNum, int thisJointNum, int nextJointNum,
							   double tx, double ty,
							   double theta, double theta_n,
							   Matrix &B, Matrix &a)
{
	DBGP("\nRef: " << refJointNum << " Proximal: " 
				   << thisJointNum << " Distal: " << nextJointNum);

	if (thisJointNum == refJointNum) {
		DBGP(">>> - cos(theta/2) = " << - cos(theta/2.0));
	}

	//translation tx ty from refJoint to thisJoint
	Matrix t(2,1);
	t.elem(0,0) = tx; t.elem(1,0) = ty;

	//joint angles theta and theta_n
	//remember in our math convention flexion rotation is negative
	Matrix Rtheta(Matrix::ROTATION2D(-theta));
	//we will flip the sign of everything later to make it agree with graspit

	//this is the first term in the chain
	//actually contains the resultant force direction
	Matrix m1(1,2);

	//proximal insertion point
	if (nextJointNum >= 0) {
		m1.elem(0,0) = cos(theta) - cos(theta/2.0);
		m1.elem(0,1) = -sin(theta) + sin(theta/2.0);
	} else {
		m1.elem(0,0) = -cos(theta/2.0);
		m1.elem(0,1) =  sin(theta/2.0);
	}

	Matrix free_term(1,1);
	matrixMultiply(m1, t, free_term);

	Matrix b_terms(1,2);
	matrixMultiply(m1, Rtheta, b_terms);

	B.elem(refJointNum, thisJointNum) += b_terms.elem(0,0);
	DBGP("Proximal effect: " << b_terms.elem(0,0));
	B.elem(refJointNum, 6) += b_terms.elem(0,1);
	a.elem(refJointNum, 0) += free_term.elem(0,0);

	//distal insertion point
	if (nextJointNum >= 0) {
		m1.elem(0,0) =  cos(theta + theta_n/2.0) - cos(theta);
		m1.elem(0,1) = -sin(theta + theta_n/2.0) + sin(theta);

		Matrix S(Matrix::ZEROES<Matrix>(2,3));
		S.elem(0,0) = S.elem(1,1) = S.elem(1,2) = 1.0;

		matrixMultiply(m1, t, free_term);

		matrixMultiply(m1, Rtheta, b_terms);
		Matrix b_terms_ext(1,3);
		matrixMultiply(b_terms, S, b_terms_ext);

		B.elem(refJointNum, nextJointNum) += b_terms_ext.elem(0,0);
		DBGP("Distal effect: " << b_terms_ext.elem(0,0));
		B.elem(refJointNum, 6) += b_terms_ext.elem(0,1);
		B.elem(refJointNum, 7) += b_terms_ext.elem(0,2);
		a.elem(refJointNum, 0) += free_term.elem(0,0);
	}

	//mid-link change
	if (nextJointNum >= 0) {
		DBGP("Proximal -1.0 and distal +1.0");
		B.elem(refJointNum, thisJointNum) += -1.0;
		B.elem(refJointNum, nextJointNum) +=  1.0;
	}
}

/*! The overall matrices B and a relate hand construction parameters and 
	tendon force to joint torques:

	(B [l r d]^t + a) * f = tau

	By definition, the matrix of unknowns is thus

	[l0 l1 l2 l3 l4 l5 r d]^T

	This takes into account the influence of all insertion points on *all* 
	joints that come before them (like a Jacobian for example).

	The contribution of each individual insertion point on all joints is 
	computed in the reference system of the link that the insertion point
	is on, and considered on all proximal joints. The contributions are
	computed separately, in the \a assembleTorqueMatrices(...) function.

	With the current assumptions, it turns out that only l_i influences
	the torque at joint i, not any l_j with j > i. Therefore, the part of 
	the B matrix that corresponds to the l entries is diagonal.
*/
void
McGrip::getRoutingMatrices(Matrix **B, Matrix **a)
{
	*B = new Matrix(Matrix::ZEROES<Matrix>(6,8));
	*a = new Matrix(Matrix::ZEROES<Matrix>(6,1));

	//compute the world locations of all the joints of the robot
	//this is overkill, as we might not need all of them
	std::vector< std::vector<transf> > jointTransf(getNumChains());
	for (int c=0; c<getNumChains(); c++) {
		jointTransf[c].resize(getChain(c)->getNumJoints(), transf::IDENTITY);
		getChain(c)->getJointLocations(NULL, jointTransf[c]);
	}

	assert(numChains==2);
	for(int c=0; c<2; c++) {
		assert (getChain(c)->getNumJoints()==3);
		for (int j=0; j<3; j++) {
			int refJointNum = c*3 + j;
			transf refTran = jointTransf.at(c).at(j);
			for (int k=j; k<3; k++) {
				int thisJointNum = c*3 + k;
				int nextJointNum = -1;
				if (k!=2) {
					nextJointNum = c*3 + k + 1;
				}
				//compute the transform from one joint to the other
				transf thisTran = jointTransf.at(c).at(k);
				//relative transform in thisJoint's coordinate system
				transf relTran = refTran * thisTran.inverse();
				vec3 translation = relTran.translation();
				//for this hand, the z component should always be 0
				if (translation.z() > 1.0e-3) {
					DBGA("Z translation in McGrip routing matrix");
				}
				double tx, ty;
				//in the math, I have used a different convention than link
				//axes in hand geometry. 
				tx = translation.y();
				ty = translation.x();
				//this obviously means we will need to also flip the sign of the result
				//since we changed the "handed-ness" of the coordinate system
				DBGP("Joint translation: " << tx << " " << ty);

				//get the joint angles
				double theta = getChain(c)->getJoint(k)->getVal() + 
							   getChain(c)->getJoint(k)->getOffset();
				double theta_n = 0.0;
				if (k!=2) {
					theta_n = getChain(c)->getJoint(k+1)->getVal() + 
							  getChain(c)->getJoint(k+1)->getOffset();
				}
				//compute the contributions
				//negate the translation, as we want the vector from refJoint to thisJoint
				//but still in thisJoint's system
				assembleTorqueMatrices(refJointNum, thisJointNum, nextJointNum,
									   -tx, -ty, 
									   theta, theta_n,
									   **B, **a);
			}
		}
	}
	//flip the signs; in the math a positive torque is an extension
	//in the model, a positive torque is flexion
	(*B)->multiply(-1.0);
	(*a)->multiply(-1.0);
	DBGP("B matrix:\n" << **B);
	DBGP("a vector:\n" << **a);
}

int
McGrip::jointTorqueEquilibrium()
{
	Matrix *a, *B;
	getRoutingMatrices(&B, &a);
	Matrix p(8, 1);
	//tendon insertion points
	p.elem(0,0) = 5;
	p.elem(1,0) = 5;
	p.elem(2,0) = 1.65;
	//--------------
	p.elem(3,0) = 5;
	p.elem(4,0) = 5;
	p.elem(5,0) = 1.65;

	//link length and joint radius
	p.elem(6,0) = getJointRadius();
	p.elem(7,0) = getLinkLength();

	//compute joint torques
	Matrix tau(6, 1);
	matrixMultiply(*B, p, tau);
	matrixAdd(tau, *a, tau);

	//multiply by tendon force
	assert(mTendonVec.size()==2);
	assert(mTendonVec[0]->getName()=="Finger 0");
	assert(mTendonVec[1]->getName()=="Finger 1");
	double f = mTendonVec[0]->getActiveForce();
	for (int j=0; j<3; j++) {
		tau.elem(j,0) *= f;
	}
	f = mTendonVec[1]->getActiveForce();
	for (int j=0; j<3; j++) {
		tau.elem(3+j,0) *= f;
	}

	DBGA("Recovered joint forces:\n" << tau);

	//compute joint spring values
	assert(numChains==2);
	Matrix k(6, 1);
	for(int c=0; c<2; c++) {
		assert (getChain(c)->getNumJoints()==3);
		for(int j=0; j<3; j++) {
			k.elem(3*c+j,0) = getChain(c)->getJoint(j)->getSpringForce();
		}
	}
	DBGA("Recovered spring forces:\n" << k);

	//compute the difference
	Matrix delta(6,1);
	k.multiply(-1.0);
	matrixAdd(tau, k, delta);
	f = delta.fnorm();
	int result;
	if ( f >= 1.0e3) {
		DBGA("McGrip joint equilibrium failed; error norm: " << f);
		result = 1;
	} else {
		DBGA("McGrip joint equilibrium success");
		result = 0;
	}

	return result;
}

void
McGrip::getJointDisplacementMatrix(Matrix **J)
{
	*J = new Matrix(Matrix::ZEROES<Matrix>(6,6));
	assert(numChains==2);
	for(int c=0; c<2; c++) {
		assert (getChain(c)->getNumJoints()==3);
		for(int j=0; j<3; j++) {
			(*J)->elem(3*c+j, 3*c+j) = getChain(c)->getJoint(j)->getDisplacement();
		}
	}
}

int
McGripGrasp::computeQuasistaticForces(double tendonForce)
{
	//routing matrices
	Matrix *B, *a;
	static_cast<McGrip*>(hand)->getRoutingMatrices(&B, &a);

	//parameter matrix
	Matrix p(8, 1);
	//tendon insertion points
	p.elem(0,0) = 5;
	p.elem(1,0) = 5;
	p.elem(2,0) = 1.65;
	//--------------
	p.elem(3,0) = 5;
	p.elem(4,0) = 5;
	p.elem(5,0) = 1.65;

	//link length and joint radius
	p.elem(6,0) = static_cast<McGrip*>(hand)->getJointRadius();
	p.elem(7,0) = static_cast<McGrip*>(hand)->getLinkLength();

	//compute joint torques
	Matrix tau(6, 1);
	matrixMultiply(*B, p, tau);
	matrixAdd(tau, *a, tau);
	delete a;
	delete B;

	//multiply by tendon force
	tau.multiply(tendonForce);

	//call super
	return Grasp::computeQuasistaticForces(tau);
}

/*! Optimizes contact forces AND tendon route (defined by the insertion point 
	"arms" l_i) and	some construction parameters (joint radius r and link 
	length l) to get 0 resultant wrench on object, with all contact forces 
	legally inside their friction cones.

	The optimization objective is the equilibrium criterion, where joint
	torques should be balanced by contact forces. However, joint torques
	are expressed as a function of tendon force (which is assumed known since
	there is just one tendon) and tendon and hand parameters.

	JTD * beta = tau = f * (B * p + a)
	p = [l r d]

	To get rid of the free term, the relationship becomes:

	[JTD -B -I] * [beta p a] = 0

	with the variables in a fixed to their known values.

	We also have to constrain the sum contact amplitudes so at least some force
	is applied to the object. If we don't, it might find solutions where tendon
	forces just cancel out in this particular configuration. This is not ideal, 
	as the value that we constrain it to is somewhat arbitrary. However, it should 
	be compensated since the equilibrium constraint is not hard, but rather the 
	optimization objective.

	Alternatively, we can multiply the B and a matrices by the desired tendon
	force which does the same thing a lot more elegantly.

	Here, the constraints themselves are nothing but the force-closure criterion.
	Therefore, they should be satisfiable anytime the grasp has f-c. The addition
	is the equilibrium, which is hand specific, but is the objective rather than 
	a constraint.

	The overall optimization problem has the form:

	minimize
	[beta p a] [JTD -B -I]^T [JTD -B -I] [beta p a]^t      (joint equilibrium)
	constrained by:
	[G 0 0] [beta p a]^T  = |0|							   (0 resultant wrench)
	[S 0 0]					|1|							   (some force applied on object)
	[F 0 0] [beta p a]^T <= 0							   (all forces inside cone)
	beta >= 0											   (legal contact forces)
	p_min <= p <= p_max									   (limits on the parameters)
	a = a_known											   (free term is known)

	Return results: we report that the problem is unfeasible if either the
	optimization itself if unfeasible OR the objective is not 0. However, if
	the optimization is successful but with non-0 objective, we accumulate and
	display contact forces as usual, and return the optimized parameters.
	Codes: 0 is success; >0 means problem unfeasible; <0 means error in 
	computation
*/
int
McGripGrasp::tendonAndHandOptimization(Matrix *parameters, double &objValRet)
{
	//use the pre-set list of contacts. This includes contacts on the palm, but
	//not contacts with other objects or obstacles
	std::list<Contact*> contacts;
	contacts.insert(contacts.begin(),contactVec.begin(), contactVec.end());
	//if there are no contacts we are done
	if (contacts.empty()) return 0;

	//retrieve all the joints of the robot. for now we get all the joints and in
	//the specific order by chain. keep in mind that saved grasps do not have
	//self-contacts so that will bias optimizations badly
	//RECOMMENDED to use this for now only for grasps where both chains are
	//touching the object and there are no self-contacts
	std::list<Joint*> joints;
	for (int c=0; c<hand->getNumChains(); c++) {
		std::list<Joint*> chainJoints = hand->getChain(c)->getJoints();
		joints.insert(joints.end(), chainJoints.begin(), chainJoints.end());
	}

	//build the Jacobian and the other matrices that are needed.
	//this is the same as in other equilibrium functions.
	Matrix J(contactJacobian(joints, contacts));
	Matrix D(Contact::frictionForceBlockMatrix(contacts));
	Matrix F(Contact::frictionConstraintsBlockMatrix(contacts));
	Matrix R(Contact::localToWorldWrenchBlockMatrix(contacts));
	//grasp map that relates contact amplitudes to object wrench G = S*R*D
	Matrix G(graspMapMatrix(R,D));
	//matrix that relates contact forces to joint torques JTD = JTran * D
	Matrix JTran(J.transposed());
	Matrix JTD(JTran.rows(), D.cols());
	matrixMultiply(JTran, D, JTD);

	//get the routing equation matrices
	Matrix *a, *negB;
	static_cast<McGrip*>(hand)->getRoutingMatrices(&negB, &a);
	negB->multiply(-1.0);
	assert(JTD.rows() == negB->rows());
	assert(JTD.rows() == a->rows());
	//scale B and a by tendon force
	double tendonForce = 1.0e6;
	negB->multiply(tendonForce);
	a->multiply(tendonForce);

	//int numBetas = JTD.cols();
	int numParameters = negB->cols();
	//int numFree = a->rows();

	//matrix of unknowns
	Matrix p(JTD.cols() + negB->cols() + a->rows(), 1);

	//optimization objective
	Matrix Q( JTD.rows(), p.rows() );
	Q.copySubMatrix( 0, 0, JTD );
	Q.copySubMatrix( 0, JTD.cols(), *negB );
	Q.copySubMatrix( 0, JTD.cols() + negB->cols(), Matrix::NEGEYE(a->rows(), a->rows()) );

	//friction matrix padded with zeroes
	Matrix FO( Matrix::ZEROES<Matrix>(F.rows(), p.rows()) );
	FO.copySubMatrix(0, 0, F);

	//equality constraint is just grasp map padded with zeroes
	Matrix EqLeft( Matrix::ZEROES<Matrix>(G.rows(), p.rows()) );
	EqLeft.copySubMatrix(0, 0, G);
	//and right hand side is just zeroes
	Matrix eqRight( Matrix::ZEROES<Matrix>(G.rows(), 1) );
		
	/*
	//let's add the summation to the equality constraint
	Matrix EqLeft( Matrix::ZEROES<Matrix>(G.rows() + 1, p.rows()) );
	EqLeft.copySubMatrix(0, 0, G);
	Matrix sum(1, G.cols());
	sum.setAllElements(1.0);
	EqLeft.copySubMatrix(G.rows(), 0, sum);
	//and the right side of the equality constraint
	Matrix eqRight( Matrix::ZEROES<Matrix>(G.rows()+1, 1) );
	//betas must sum to one
	eqRight.elem(G.rows(), 0) = 1.0;
	*/

	//lower and upper bounds
	Matrix lowerBounds( Matrix::MIN_VECTOR(p.rows()) );
	Matrix upperBounds( Matrix::MAX_VECTOR(p.rows()) );
	//betas are >= 0
	for (int i=0; i<JTD.cols(); i++) {
		lowerBounds.elem(i,0) = 0.0;
	}
	//l's have hard-coded upper and lower limits
	for (int i=0; i<6; i++) {
		lowerBounds.elem( JTD.cols() + i, 0) = -5.0;
		upperBounds.elem( JTD.cols() + i, 0) =  5.0;
	}

	//use this if you already know the hand parameters and are just using this to check
	//how close this grasp is to equilibrium
	//tendon insertion points
	lowerBounds.elem(JTD.cols() + 0,0) = 5;
	lowerBounds.elem(JTD.cols() + 1,0) = 5;
	lowerBounds.elem(JTD.cols() + 2,0) = 1.65;
	upperBounds.elem(JTD.cols() + 0,0) = 5;
	upperBounds.elem(JTD.cols() + 1,0) = 5;
	upperBounds.elem(JTD.cols() + 2,0) = 1.65;
	//--------------
	lowerBounds.elem(JTD.cols() + 3,0) = 5;
	lowerBounds.elem(JTD.cols() + 4,0) = 5;
	lowerBounds.elem(JTD.cols() + 5,0) = 1.65;
	upperBounds.elem(JTD.cols() + 3,0) = 5;
	upperBounds.elem(JTD.cols() + 4,0) = 5;
	upperBounds.elem(JTD.cols() + 5,0) = 1.65;

	//r and d have their own hard-coded limits, fixed for now
	lowerBounds.elem(JTD.cols() + 6, 0) = static_cast<McGrip*>(hand)->getJointRadius();
	upperBounds.elem(JTD.cols() + 6, 0) = static_cast<McGrip*>(hand)->getJointRadius();
	lowerBounds.elem(JTD.cols() + 7, 0) = static_cast<McGrip*>(hand)->getLinkLength();
	upperBounds.elem(JTD.cols() + 7, 0) = static_cast<McGrip*>(hand)->getLinkLength();
	//the "fake" variables in a are fixed
	for (int i=0; i<a->rows(); i++) {
		lowerBounds.elem(JTD.cols() + 8 + i, 0) = a->elem(i,0);
		upperBounds.elem(JTD.cols() + 8 + i, 0) = a->elem(i,0);
	}

	//we don't need the routing matrices any more
	delete negB;
	delete a;

	//solve the whole thing
	double objVal;
	int result = factorizedQPSolver(Q,
									EqLeft, eqRight,
									FO, Matrix::ZEROES<Matrix>(FO.rows(), 1),
									lowerBounds, upperBounds,
									p, &objVal);
	objValRet = objVal;

	if (result) {
		if( result > 0) {
			DBGA("McGrip constr optimization: problem unfeasible");
		} else {
			DBGA("McGrip constr optimization: QP solver error");
		}
		return result;
	}
	DBGA("Construction optimization objective: " << objVal);
	DBGP("Result:\n" << p);

	//get the contact forces and the parameters; display contacts as usual
	Matrix beta(JTD.cols(), 1);
	beta.copySubBlock(0, 0, JTD.cols(), 1, p, 0, 0);
	//scale betas by tendon foce
	beta.multiply(1.0e7);
	parameters->copySubBlock(0, 0, numParameters, 1, p, JTD.cols(), 0);

	//retrieve contact wrenches in local contact coordinate systems
	Matrix cWrenches(D.rows(), 1);
	matrixMultiply(D, beta, cWrenches);
	DBGP("Contact forces:\n " << cWrenches);

	//compute object wrenches relative to object origin and expressed in world coordinates
	Matrix objectWrenches(R.rows(), cWrenches.cols());
	matrixMultiply(R, cWrenches, objectWrenches);
	DBGP("Object wrenches:\n" << objectWrenches);

	//display them on the contacts and accumulate them on the object
	displayContactWrenches(&contacts, cWrenches);
	accumulateAndDisplayObjectWrenches(&contacts, objectWrenches);

	//we actually say the problem is also unfeasible if objective is not 0
	//this is magnitude squared of contact wrench, where force units are N*1.0e6
	//acceptable force is 1mN -> (1.0e3)^2 magnitude 
	if (objVal > 1.0e6) {
		return 1;
	}
	return 0;
}
