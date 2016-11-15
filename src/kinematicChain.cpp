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
// $Id: kinematicChain.cpp,v 1.33 2009/09/11 19:06:32 jweisz Exp $
//
//######################################################################

#include "kinematicChain.h"

#include "matvec3D.h"
#include "body.h"
#include "robot.h"
#include "dof.h"
#include "joint.h"
#include "world.h"
#include "matvecIO.h"
#include "dynJoint.h"
#include "humanHand.h"
#include "math/matrix.h"
#include "tinyxml.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

//for disp_mat, who needs a new home
#include "maxdet.h"

//#define GRASPITDBG
#include "debug.h"

KinematicChain::KinematicChain(Robot *r,int chainNumber, int jointNum) : owner(r),chainNum(chainNumber), firstJointNum(jointNum),
									 numJoints(0),numLinks(0),lastJoint(NULL), IVRoot(NULL),numChildren(0)
{
}

/*!
  The destructor deletes each of the joints in this chain, and asks the
  world to delete each of the links in the chain.  It also detaches any
  connected robots.
*/
KinematicChain::~KinematicChain()
{
  int i;
  IVTran->unref();

  delete [] lastJoint;

  for (i=0;i<numJoints;i++)
    if (jointVec[i]) delete jointVec[i];

    for (i=0;i<numLinks;i++)
      if (linkVec[i]) owner->getWorld()->destroyElement(linkVec[i]);
	

  // go in reverse order becase these operations will delete elements from
  // the children vector
  for (i=numChildren-1;i>=0;i--)
    owner->detachRobot(children[i]);
}

/*! Creates dynamic joints for each of the links in the chain. A dynamic joint
	can be a collection of one or more regular joints, so dynamic joints are
	only created when all the links and the regular joints are already in place.
	The vector of dynamic joint types tells us what kind of dynamic joint
	each link is connected to, and the dynamic joint is then constructed
	based on the appropriate number of regular joints.

	See the DynJoint class for details.
*/
int
KinematicChain::createDynamicJoints(const std::vector<int> &dynJointTypes)
{
  Link* prevLink = owner->getBase();
  for (int l=0; l<numLinks; l++){
    transf dynJointTran = transf::IDENTITY;
    if(l==0) dynJointTran = tran;
    
    if (dynJointTypes[l] == DynJoint::BALL) {
      linkVec[l]->setDynJoint(new BallDynJoint(
                                               jointVec[lastJoint[l]-2],
                                               jointVec[lastJoint[l]-1],jointVec[lastJoint[l]],
                                               prevLink,linkVec[l], 
                                               dynJointTran, jointVec[lastJoint[l]]->getTran().inverse()));
      jointVec[lastJoint[l]-2]->dynJoint = linkVec[l]->getDynJoint();
      jointVec[lastJoint[l]-1]->dynJoint = linkVec[l]->getDynJoint();
      jointVec[lastJoint[l]-0]->dynJoint = linkVec[l]->getDynJoint();
    } else if (dynJointTypes[l] == DynJoint::UNIVERSAL) {
      linkVec[l]->setDynJoint(new UniversalDynJoint(
                                                    jointVec[lastJoint[l]-1], jointVec[lastJoint[l]],
                                                    prevLink, linkVec[l], 
                                                    dynJointTran, jointVec[lastJoint[l]]->getTran().inverse()));
      jointVec[lastJoint[l]-1]->dynJoint = linkVec[l]->getDynJoint();
      jointVec[lastJoint[l]-0]->dynJoint = linkVec[l]->getDynJoint();
    } else if (dynJointTypes[l] == DynJoint::REVOLUTE) {
      linkVec[l]->setDynJoint(new RevoluteDynJoint(jointVec[lastJoint[l]],
                                                   prevLink,linkVec[l],dynJointTran));
      jointVec[lastJoint[l]]->dynJoint = linkVec[l]->getDynJoint();
    } else if (dynJointTypes[l] == DynJoint::PRISMATIC) {
      linkVec[l]->setDynJoint(new RevoluteDynJoint(jointVec[lastJoint[l]],
                                                   prevLink,linkVec[l],dynJointTran));
      jointVec[lastJoint[l]]->dynJoint = linkVec[l]->getDynJoint();
    } else if (dynJointTypes[l] == DynJoint::FIXED) {
      DBGA("FIXED dynamic joints not yet fully supported");
      return FAILURE;
      //linkVec[l]->setDynJoint(new FixedDynJoint(prevLink, linkVec[l], dynJointTran));
    } else {
      DBGA("Unknown joint type requested");
      return FAILURE;
    }
    prevLink = linkVec[l];
  }
  return SUCCESS;
}

/*!
  Sets up the chain given a XML DOM from a currently open robot
  configuration file.  It reads the number of joints and the number of links
  and allocates space for those vectors, then it reads in the base transform
  of the chain.  Next, it reads a node for each joint and creates a
  prismatic or revolute joint which is initialized with the kinematic data
  in that node. \a linkDir should be the path to the directory where the link
  body files are kept (usually rootPath/iv).
*/ 
int
KinematicChain::initChainFromXml(const TiXmlElement* root,QString &linkDir)
{
  numJoints = countXmlElements(root,"joint");
  if (numJoints < 1) {
    DBGA("number of joints < 1");
    return FAILURE;
  }
  
  numLinks = countXmlElements(root,"link");
  if (numLinks < 1) {
    DBGA("Number of links < 1");
    return FAILURE;
  }
  
  jointVec.resize(numJoints, NULL);
  linkVec.resize(numLinks, NULL);  
  
  lastJoint = new int[numLinks];
  
  IVRoot = new SoSeparator;
  IVTran = new SoTransform;
  IVTran->ref();
  
  /* read in the finger transformation */
  const TiXmlElement* element = findXmlElement(root,"transform");
  if(element){
    if(!getTransform(element,tran)){
      QTWARNING("Fail to perform transformation");
      return FAILURE;
    }
  }
  tran.toSoTransform(IVTran);
  
  DBGA("  Creating joints");
  numDOF = 0;
  std::list<const TiXmlElement*> elementList = findAllXmlElements(root, "joint");
  std::list<const TiXmlElement*>::iterator p;
  int j;
  for(p = elementList.begin(), j=0; p!=elementList.end(); p++,j++){
    DBGA("   Joint " << j);
    QString type = (*p)->Attribute("type");
    if(type.isNull()){
      QTWARNING("No Joint Type Specified");
      return FAILURE;
    }
    if(type == "Revolute"){
      jointVec[j] = new RevoluteJoint(this);
    } else if(type == "Prismatic"){
      jointVec[j] = new PrismaticJoint (this);
    } else {
      DBGA("Unknown joint type requested");
      return FAILURE;
    }
    if (jointVec[j]->initJointFromXml(*p, firstJointNum+j) == FAILURE) {
      DBGA("Failed to initialize joint");
      return FAILURE;
    }
  }
  
  DBGA("  Creating links");
  std::vector<int> dynJointTypes;
  int lastJointNum = -1;
  elementList = findAllXmlElements(root, "link");
  int l;
  for (l=0, p=elementList.begin(); p!=elementList.end(); p++,l++){
    DBGA("   Link " << l);
    QString jointType=(*p)->Attribute("dynamicJointType");
    if(jointType.isNull()){
      QTWARNING("No Dynamic Joint Type Specified");
      return FAILURE;
    }
    jointType = jointType.stripWhiteSpace();
    if (jointType == "Revolute") {
      dynJointTypes.push_back(DynJoint::REVOLUTE);
      lastJointNum += 1;
    } else if (jointType == "Ball") {
      dynJointTypes.push_back(DynJoint::BALL);
      lastJointNum += 3;
    } else if (jointType == "Universal") {
      dynJointTypes.push_back(DynJoint::UNIVERSAL);
      lastJointNum += 2;
    } else if (jointType == "Prismatic") {
      dynJointTypes.push_back(DynJoint::PRISMATIC);
      lastJointNum += 1;
    } else if (jointType == "Fixed") {
      dynJointTypes.push_back(DynJoint::FIXED);
    } else {
      DBGA("Unknown joint type requested");
      return FAILURE;
    }
    
    QString linkFilename = (*p)->GetText();
    linkFilename = linkFilename.stripWhiteSpace();
    QString linkName = QString(owner->name()) + QString("_chain%1_link%2").arg(chainNum).arg(l);
    linkVec[l] = new Link(owner,chainNum,l,owner->getWorld(),linkName.latin1());
    if (linkVec[l]->load(linkDir + linkFilename)==FAILURE) {
      delete linkVec[l]; linkVec[l] = NULL;
      DBGA("Failed to load file for link " << l);
      return FAILURE;
    }
    /*Handle collision rule settings:
     *Common case is NULL - collisions are on except for adjacent links.
     *Off - turn off collisions for this link globally, do not enter this body
     *into the collision engine at all.
     *OverlappingPair - turn off collisions between two particular links
     */
    QString collisionRule=(*p)->Attribute("collisionRule");
    
    if(!collisionRule.isNull()){
      collisionRule = collisionRule.stripWhiteSpace();
      if (collisionRule == "Off"){
	linkVec[l]->addToIvc(true);
	linkVec[l]->myWorld->toggleCollisions(false, linkVec[l],NULL);
	DBGA("Collisions off for link " << l);
      }else if (collisionRule == "OverlappingPair"){
	/*targetChain - specifies the chain of the target link to disable
	 *collisions for.  No attribute implies current chain.  
	 *Base implies robot base.
	 *targetLink - specifies link number in target chain
	 */
	linkVec[l]->addToIvc();
	QString targetChain=(*p)->Attribute("targetChain");
	targetChain = targetChain.stripWhiteSpace();
	if (targetChain == "base")
	  linkVec[l]->myWorld->toggleCollisions(false, linkVec[l],owner->getBase());
	else{
	  
	  QString targetLink = (*p)->Attribute("targetLink");
	  if (!targetLink.isNull()){
	    bool ok = TRUE;
	    int linkNum = targetLink.toInt(&ok);
	    if (!ok){
	      DBGA("targetLink not a valid input");
	      return FAILURE;
	    }
	    
	    if(targetChain.isNull())
	      linkVec[l]->myWorld->toggleCollisions(false, linkVec[l],linkVec[linkNum]);
	    else{
	      int chainNum = targetChain.toInt(&ok);
	      if (!ok){
		DBGA("targetChain not a valid input");
		return FAILURE;
	      }
	      linkVec[l]->myWorld->toggleCollisions(false, linkVec[l],owner->getChain(chainNum)->linkVec[linkNum]);	
	    }
	  }
	}				
      }
      else{
	DBGA("Unknown Collision Rule");
	return FAILURE;
      }
    }else{
      linkVec[l]->addToIvc();
    }
    
    
    lastJoint[l] = lastJointNum;
    if (lastJoint[l] >= numJoints) {
      DBGA("Wrong last joint value: " << lastJoint[l]);
      return FAILURE;
    }
    
    IVRoot->addChild(linkVec[l]->getIVRoot());
  }
  
  DBGA("  Creating dynamic joints");
  if (createDynamicJoints(dynJointTypes) == FAILURE) {
    DBGA("Failed to create dynamic joints");
    return FAILURE;
  }
  
  jointsMoved = true;
  updateLinkPoses();
  owner->getWorld()->tendonChange();
  owner->getIVRoot()->addChild(IVRoot);
  
  return SUCCESS;
}


/*! Copies this chain structure from an existing chain. All joints are
	set to independent copies of the original chain. All links are created
	as clones of the links from the original chain.
*/
void
KinematicChain::cloneFrom(const KinematicChain *original)
{
	IVRoot = new SoSeparator;
	IVTran = new SoTransform;
	IVTran->ref();
	tran = original->getTran();
	tran.toSoTransform(IVTran);
	
	numJoints = original->getNumJoints();
	numLinks = original->getNumLinks();
	jointVec.resize(numJoints,NULL);
	linkVec.resize(numLinks,NULL);  
	lastJoint = new int[numLinks];

	numDOF = 0;
	for (int j=0; j<numJoints; j++){
		if (original->getJoint(j)->getType() == REVOLUTE) {
			jointVec[j] = new RevoluteJoint(this);
		}else if (original->getJoint(j)->getType() == PRISMATIC) {
			jointVec[j] = new PrismaticJoint(this);
		}
		jointVec[j]->cloneFrom( original->getJoint(j) );
	}
	
	std::vector<int> dynJointTypes;
	for (int l=0; l<numLinks; l++){
		lastJoint[l] = original->getLastJoint(l);
		QString linkName =  QString(owner->name())+QString("_chain%1_link%2").arg(chainNum).arg(l);
		linkVec[l] = new Link(owner,chainNum,l,owner->getWorld(),linkName);
		linkVec[l]->cloneFrom( original->getLink(l) );
		//linkVec[l]->setTransparency(0.5);
		IVRoot->addChild(linkVec[l]->getIVRoot());
		dynJointTypes.push_back( original->getLink(l)->getDynJoint()->getType() );
	}
	createDynamicJoints(dynJointTypes);

	jointsMoved = true;
	owner->getIVRoot()->addChild(IVRoot);
}

/*!
  Given a pointer to another robot and an offset transform of the base
  frame of the robot with respect to the end transform of the link of
  this chain.
*/
void
KinematicChain::attachRobot(Robot *r,const transf &offsetTr)
{
  children.push_back(r);
  childOffsetTran.push_back(offsetTr);
  numChildren++;

  if (r->getMountPiece())
    r->getMountPiece()->
      setDynJoint(new FixedDynJoint(linkVec[numLinks-1],r->getMountPiece(), offsetTr));
  else
    r->getBase()->
      setDynJoint(new FixedDynJoint(linkVec[numLinks-1],r->getBase(), offsetTr));
}

/*!
  This separates the robot pointed to by r from this kinematic chain,
  allowing them to move independently.
*/
void
KinematicChain::detachRobot(Robot *r)
{
  int i,j;
  std::vector<transf>::iterator tp;
  std::vector<Robot *>::iterator rp;

  for (i=0,rp=children.begin();rp!=children.end();i++,rp++)
    if (*rp==r) {children.erase(rp); break;}

  for (j=0,tp=childOffsetTran.begin();tp!=childOffsetTran.end();j++,tp++)
    if (j==i) {childOffsetTran.erase(tp); break;}

  numChildren--;
}

std::list<Joint*> 
KinematicChain::getJoints()
{
	std::list<Joint*> joints;
	for (int j=0; j<numJoints; j++) {
		joints.push_back(jointVec[j]);
	}
	return joints;
}
/*! Filters a collision report and keeps only collisions of a particular chain
	For now it also throws out collisions between robot parts, as those are 
	giving us problems.
*/
void KinematicChain::filterCollisionReport(CollisionReport &colReport)
{
	//only keep in the collision report those collision that interest this chain
	CollisionReport::iterator it = colReport.begin();
	bool keep;
	while ( it != colReport.end() ) {

		if ( (*it).first->getOwner() != owner ) {
			if ( (*it).second->getOwner() != owner ) {
				keep = false;
			} else {
				if ( ((Link*)(*it).second)->getChainNum() == chainNum )
					keep = true;
				else
					keep = false;
			}
		} else if ( (*it).second->getOwner() != owner ) {
			if ( ((Link*)(*it).first)->getChainNum() == chainNum )
				keep = true;
			else
				keep = false;			
		} else {
			keep = false;
		}
		if (!keep) {
			it = colReport.erase(it);
		} else {
			it++;
		}
	}
}

/*! Reads in the current values of the joints in this chain, and uses
	them to populate the vector \a jointVals. This vector is ordered to
	contain all the joints of the robot (not only of this chain) and
	joints are indexed by their number in the robot structure, not in
	this chain's struture.
*/
void
KinematicChain::getJointValues(double *jointVals) const
{
	for (int j=0; j<numJoints; j++) {
		jointVals[firstJointNum + j] = jointVec[j]->getVal();
	}
}

/*! Sets the current joint values from the vector \a jointVals. This 
	vector is ordered to contain all the joints of the robot (not only 
	of this chain) and joints are indexed by their number in the robot 
	structure, not in this chain's struture.
*/
void
KinematicChain::setJointValues(const double *jointVals)
{
	for (int j=0; j<numJoints; j++) {
		jointVec[j]->setVal( jointVals[firstJointNum + j] );
	}
}

/*! Computes forward kinematics for the current joint values, then
	sets the link transforms accordingly. This is the only way that 
	robot links should ever be moved (the robot asks the dof' for
	joint values, tells the chains to set those values then tells
	the chains to update link poses).
*/
void
KinematicChain::updateLinkPoses()
{
	std::vector<transf> newLinkTranVec;
	newLinkTranVec.resize(numLinks, transf::IDENTITY);
	fwdKinematics(NULL, newLinkTranVec);

	for (int l=0;l<numLinks;l++) {
		linkVec[l]->setTran(newLinkTranVec[l]);
	}

	for (int j=0;j<numChildren;j++) {
		children[j]->simpleSetTran(childOffsetTran[j]*newLinkTranVec[numLinks-1]);
	}

	if ( owner->inherits("HumanHand") ){
		((HumanHand*)owner)->updateTendonGeometry();
		owner->getWorld()->tendonChange();
	}
}

/*! Given a array of joint values for each joint in the chain, this method
	will compute the transforms for each link in the chain with respect to
	world coordinates.  It does not affect the chain itself.

	The array of joint values is assumed to contain all the joints in the
	robot, numbered as in the robot's numbering scheme. Alternatively,
	you can pass NULL instead, in which case the current joint values are
	used.
*/
void
KinematicChain::fwdKinematics(const double *jointVals, std::vector<transf> &newLinkTranVec) const
{
	transf total = tran * owner->getTran();
	int l=0;
	for (int j=0;j<numJoints;j++) {    
		if (!jointVals) {
			total = jointVec[j]->getTran( jointVec[j]->getVal() ) * total;
		} else {
			total = jointVec[j]->getTran( jointVals[firstJointNum + j] ) * total;
		}
		if (l<numLinks && lastJoint[l]==j) {
			newLinkTranVec[l] = total;
			l++;
		}
	}
}

/*! Given an array of joint values for each joint in the chain, it will compute
	the location and orientations of all the joints in the chain, w.r.t. world
	coordinates. This is somewhat similar to fwd kinematics, but instead of 
	computing link transforms it computes joint transforms. This is different 
	for two reasons. First, a joint's coordinate system is the chained transform 
	*before* that joint, not after the joint (as it would be for a link that 
	comes after the joint). Also, we can have multiple joints between two links.

	Also remember two GraspIt conventions: for any joint, the joint axis (for 
	rotation or translation) is the z axis of the joint transform returned here.
	Also, the origin of a link is the same as the transform *after* the last joint
	that comes before it.

	The array of joint values is assumed to contain all the joints in the
	robot, numbered as in the robot's numbering scheme. Alternatively,
	you can pass NULL instead, in which case the current joint values are
	used.
*/
void
KinematicChain::getJointLocations(const double *jointVals, std::vector<transf> &jointTranVec) const
{
	transf total = tran * owner->getTran();
	for (int j=0;j<numJoints;j++) {    
		jointTranVec[j] = total;
		if (!jointVals) {
			total = jointVec[j]->getTran( jointVec[j]->getVal() ) * total;
		} else {
			total = jointVec[j]->getTran( jointVals[firstJointNum + j] ) * total;
		}
	}	
}

/*! Given an array of desired joint values, this computed an infinitesimal motion
	of each link as motion *from the current values* towards the desired values is 
	started. Used mainly to	see if any contacts prevent this motion.
*/
void
KinematicChain::infinitesimalMotion(const double *jointVals, std::vector<transf> &newLinkTranVec) const
{
	//start with the link jacobian in local link coordinates
	//but keep just the actuated part
	Matrix J(actuatedJacobian(linkJacobian(false)));
	//joint values matrix
	Matrix theta(numJoints, 1);
	//a very small motion
	//either 0.1 radians or 0.1 mm, should be small enough
	double inf = 0.1;
	//a very small threshold
	double eps = 1.0e-6;
	for(int j=0; j<numJoints; j++) {
		int sign;
		if ( jointVals[firstJointNum + j] + eps < jointVec[j]->getVal() ) {
			sign = -1;
		} else if ( jointVals[firstJointNum + j] > jointVec[j]->getVal() + eps ) {
			sign = 1;
		} else {
			sign = 0;
		}
		theta.elem(j,0) = sign * inf;
	}
	//compute infinitesimal motion
	Matrix dm(6*numLinks, 1);
	matrixMultiply(J, theta, dm);
	//and convert it to transforms
	for (int l=0; l<numLinks; l++) {
		transf tr = rotXYZ( dm.elem(6*l+3, 0), dm.elem(6*l+4, 0), dm.elem(6*l+5, 0) ) * 
					translate_transf( vec3( dm.elem(6*l+0, 0), dm.elem(6*l+1, 0), dm.elem(6*l+2, 0) ) );
		newLinkTranVec[l] = tr;
	}
}

void 
KinematicChain::getDynamicJoints(std::vector<DynJoint*> *dj) const
{
	DynJoint *lastDynJoint = NULL;
	for(int j=0; j<numJoints; j++) {
		if (jointVec[j]->dynJoint == lastDynJoint) continue;
		lastDynJoint = jointVec[j]->dynJoint;
		dj->push_back(lastDynJoint);
	}
}

/*! The link jacobian relates joint angle changes to link movement.
	Its transpose relates forces applied to link to forces applied
	to joints. If \a worldCoords is false, the jacobian is computed
	in each link's coordinate system.
*/
Matrix
KinematicChain::linkJacobian(bool worldCoords) const
{
	Matrix J(Matrix::ZEROES<Matrix>(numLinks * 6, numLinks * 6));
	Matrix indJ(6,6);
	for (int l=0; l<numLinks; l++) {
		for (int dl=0; dl<=l; dl++) {
			DynJoint *dynJoint = linkVec[dl]->getDynJoint();
			dynJoint->jacobian(linkVec[l]->getTran(), &indJ, worldCoords);
			J.copySubMatrix(6*l, 6*dl, indJ);
		}
	}
	return J;
}

/*! The active link jacobian will build the regular link Jacobian, then
	only keep the rows that correspond to links that have at least one
	contact (as links with no contacts can not balance actuation forces 
	applied	to them).
*/
Matrix
KinematicChain::activeLinkJacobian(bool worldCoords)
{
	Matrix J(linkJacobian(worldCoords));
	if (!J.rows() || !J.cols()) return Matrix(0,0);
	int activeLinks = 0;
	for (int l=0; l<numLinks; l++) {
		if (linkVec[l]->getNumContacts()) {
			activeLinks++;
		}
	}
	if (!activeLinks) {
		DBGA("Active link Jac requested, but no active links!");
		return Matrix::ZEROES<Matrix>(0,0);
	}
	int activeRows = 6*activeLinks;
	Matrix activeJ(activeRows, J.cols());
	int linkCount = 0;
	//only keep rows that correspond to links that have at least one contact
	for (int l=0; l<numLinks; l++) {
		if (!linkVec[l]->getNumContacts()) continue;
		activeJ.copySubBlock(6*linkCount, 0, 6, J.cols(), J, 6*l, 0);
		linkCount++;
	}
	return activeJ;
}

/*! This function takes a Jacobian with all columns (6dof per joint
	and returns a version that has only those columns that correspond
	to the actuated dofs in each joint (e.g. the z axis for revolute 
	joints) 
*/
Matrix
KinematicChain::actuatedJacobian(const Matrix &fullColumnJ) const
{
	std::vector<DynJoint*> dynJoints;
	getDynamicJoints(&dynJoints);
	int activeRows = fullColumnJ.rows();
	if (!activeRows) return Matrix(0,0);
	//first count the constraints
	int numConstrained = 0;
	int numActuated = 0;
	for(int dj=0; dj<(int)dynJoints.size(); dj++) {
		numConstrained += dynJoints[dj]->getNumConstraints();
		numActuated += 6 - dynJoints[dj]->getNumConstraints();
	}
	assert(numActuated + numConstrained == 6*numLinks);
	assert(fullColumnJ.cols() == 6*numLinks);
	Matrix blockJ(activeRows, numActuated);
	//then copy only actuated columns in the block jacobian.
	int actIndex = 0;
	for(int dj=0; dj<(int)dynJoints.size(); dj++) {
		char constraints[6];
		dynJoints[dj]->getConstraints(constraints);
		for (int c=0; c<6; c++) {
			if (!constraints[c]) {
				//actuated direction
				blockJ.copySubBlock(0, actIndex, activeRows, 1, fullColumnJ, 0, 6*dj+c);
				actIndex++;
			}
		}
	}
	assert(actIndex == numActuated);
	return blockJ;
}

/*! Given a matrix with all the joint torques of the robot, numbered as in the 
	the robot scheme, this extracts a vector of chain joint torques, in the order
	in which they appear in this chain
*/
Matrix 
KinematicChain::jointTorquesVector(Matrix fullRobotTorques)
{
	assert(fullRobotTorques.cols() == 1);
	Matrix tau(numJoints,1);
	for (int j=0; j<numJoints; j++) {
		tau.elem(j,0) = fullRobotTorques.elem(jointVec[j]->getNum(), 0);
	}
	return tau;
}

/*! Returns the number of contacts between the links of this chain and the object
	\a body. If \a body == NULL, returns the total number of contacts, regardless
	of what object they are against.
	Todo: what about self-colision?
*/
int  
KinematicChain::getNumContacts(Body *body)
{
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		numContacts += linkVec[l]->getNumContacts(body);
	}
	return numContacts;
}

/*! Returns a list of the contacts between the links of this chain and the object
	\a body. If \a body == NULL, returns all the contacts, regardless of what object 
	they are against.
	Todo: what about self-colision?
*/
std::list<Contact*>
KinematicChain::getContacts(Body *body)
{
	std::list<Contact*> contacts;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> linkContacts = linkVec[l]->getContacts(body);
		contacts.insert(contacts.end(), linkContacts.begin(), linkContacts.end());
	}
	return contacts;
}
