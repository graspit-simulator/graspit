#include "Arizona_Test.h"
#include "Arizona_Raw_Exp.h"
#include "contact.h"
#include "grasp.h"
//#include <VCollide.H>
//#include <PQP.H>
#include "gws.h"
#include "gwsprojection.h"
#include "world.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "quality.h"
#include "body.h"

//#define GRASPITDBG
#include "debug.h"


#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>

void ArizonaTest::alignAxis(GWS * gws){
	double temp;
	DBGP("in alignAxis" << std::endl);
	int indices[] = {2,3,4};
	if(!mIsBuildIn3D){
		DBGP("not in 3d");
		indices[0] = 2;
		indices[1] = 3;
		indices[2] = 4;
	}
	for(int i = 0; i < gws->numHyperPlanes; i ++){
		temp = gws->hyperPlanes[i][indices[0]];
		gws->hyperPlanes[i][indices[0]] = gws->hyperPlanes[i][indices[1]];
		gws->hyperPlanes[i][indices[1]] = gws->hyperPlanes[i][indices[2]];
		gws->hyperPlanes[i][indices[2]] = temp;
	}
}

ArizonaTest::ArizonaTest(){
	mGrasp = NULL;
	mQual = NULL;
	mObject = NULL;
	mIsBuildIn3D = false;
}

ArizonaTest::~ArizonaTest()
{
	if (mGrasp) delete mGrasp;
	if (mQual) delete mQual;
	coords.clear();
	indices.clear();
}

QString ArizonaTest::getCurrentType()
{
	if(mCondition == 0)	return QString("random");
	else if (mCondition == 1) return QString("blocked");
	else return QString("test");
}

QString ArizonaTest::getCMPattern()
{
	QString q;
	q.setNum(mCMPattern);
	return q;
}

void ArizonaTest::setObject(GraspableBody *b)
{
	if (mObject == b) return;
	mObject = b;

	if(mGrasp) delete mGrasp;
	mGrasp = new Grasp(NULL);
	mGrasp->setObject(mObject);

	if (mQual) delete mQual;
	mQual = new QualEpsilon( mGrasp, QString("Virtual_grasp_qm"),"L1 Norm");

	DBGP("Arizona test: grasp and quality measure created");
}

void ArizonaTest::setTestData(ArizonaRawExp *are, QString ellipsoidFile, bool isBuildIn3D, bool isFlipped)
{
	if (!mObject) {
		DBGA("Object not set");
		return;
	}
	mObject->breakVirtualContacts();

	//memory leaks here?
	mContacts.clear();

	mCMPattern = are->getCMPattern();
	mCondition = are->getType();
	mIsBuildIn3D = isBuildIn3D;
	mIsFlipped = isFlipped;

	for (int i=0; i<are->getNumContacts(); i++) {
		VirtualContactOnObject *newContact = new VirtualContactOnObject();
		newContact->readFromRawData(are,ellipsoidFile,i,isFlipped);

		newContact->setBody( mObject );

		if (newContact->getNormal().len() == 0) {
			delete newContact;
			continue;
		}
		((Contact*)newContact)->computeWrenches();
		mObject->addVirtualContact(newContact);
		mContacts.push_back(newContact);
	}
	DBGP("AT data set; pattern is " << mCMPattern << "; condition is " << mCondition <<
		"; 3D is " << mIsBuildIn3D << "; contacts: " << mContacts.size());
}

void ArizonaTest::initializeTest()
{
	if(!mObject){
		DBGA("ArizonaTest: object not present");
		return;
	}
	if (mContacts.empty()) {
		DBGA("ArizonaTest: no contacts");
		return;
	}

	std::vector<int> arizonaTestD(6,0);
	if (mIsBuildIn3D) {
		//fx, tx and ty
		arizonaTestD[2] = arizonaTestD[3] = arizonaTestD[4] = 1;
	} else {
		arizonaTestD = Grasp::ALL_DIMENSIONS;
	}
	DBGP("AT: updating grasp");
	mGrasp->update(arizonaTestD); //  arizonaTestD is used when isBuildIn3D = true;

	//create the 3D hull that we are interested in 
	DBGP("AT: creating 3D hull");
	createGWSProjection();

	if(mQual->evaluate() < 0){
		DBGA("Non-ForceClosure");
		return;
	}

	for(int i = 1; i <= NUMCMPATTERN; i ++){
		forceScales[i-1] = getMinimunForce(i);
	}
	DBGP("AT: all tests initialized");
}

double ArizonaTest::getQuality()
{
	DBGP("AT: evaluating quality");
	double q = mQual->evaluate();
	DBGP("Quality: " << q);
	if (q < 0) q = -1;
	return q;
}

void ArizonaTest::writeForce2File(FILE* fp){

	if(mQual->evaluate() < 0 ){
		fprintf(fp, "N/A\n");
		return;
	}

	for(int i = 0; i < NUMCMPATTERN; i ++){
		fprintf(fp,"%f\t",forceScales[i]);
	}
	fprintf(fp,"\n");
}

void ArizonaTest::writeQuality2File(FILE* fp){
	double qual = mQual->evaluate();
	if(qual < 0 ){
		fprintf(fp, "N/A\n");
		return;
	}
	fprintf(fp,"%f\n",qual);
}

void ArizonaTest::writeContact2File(FILE* fp){
	for(int i = 0; i < (int)mContacts.size(); i ++){
		fprintf(fp,"%f\t%f\t%f\t",mContacts[i]->getPosition().x(),mContacts[i]->getPosition().y(),mContacts[i]->getPosition().z());
	}
	fprintf(fp,"\n");
}

void ArizonaTest::createGWSProjection()
{
	DBGP("AT: createGWSProjection. isBuildIn3D: " << mIsBuildIn3D);

	GWS *gws = mGrasp->getGWS("L1 Norm");
	assert(gws);
	coords.clear();
	indices.clear();
	//zeroes and whichFixed are not used if mIsBuildIn3D is true
	double zeroes[] = {0,0,0,0,0,0};
	std::set<int> whichFixed;
	whichFixed.insert(0); whichFixed.insert(1);	whichFixed.insert(5);
	alignAxis(gws);
	gws->projectTo3D(zeroes, whichFixed, coords, indices);

	DBGP("Step out of GWSprojection");
}

void ArizonaTest::showGWSProjection()
{
	//This is a hack. It should not really create another gws projection, but rather display the existing
	//hull. Maybe I'll fix that at some point.
	GWS *gws = mGrasp->getGWS("L1 Norm");
	assert(gws);
	//zeroes and whichFixed are not used if mIsBuildIn3D is true
	double zeroes[] = {0,0,0,0,0,0};
	std::set<int> whichFixed;
	whichFixed.insert(0); whichFixed.insert(1);	whichFixed.insert(5);
	GWSprojection *gp = new GWSprojection(graspItGUI->getIVmgr()->getViewer(),gws,
										  zeroes,whichFixed);
	//the gp is left hanging in the air... really bad.
	addDisturbance2GWS(gp);
}


double ArizonaTest::getScale2Surface(double *dis)
{
	if (coords.empty() || indices.empty()) {
		DBGA("Unable to compute minimum force; hull not set");
		return 0;
	}
	if(mIsFlipped){
		dis[0] = - dis[0];
		dis[1] = - dis[1];
		dis[2] = - dis[2];
	}

	// compute the necessary force
	PQP_REAL v1[3],v2[3],v3[3];
	PQP_Model volume = PQP_Model();

	//model for the volumn
	volume.BeginModel();
	int triInd = 0;
	int numIndices = indices.size();
	for (int k = 0; k < numIndices - 3; k++) {
		if(indices[k+3] == -1) //  dealing with meaningful vertices.
		{
			v1[0]=coords[indices[k]].x();
			v1[1]=coords[indices[k]].y();
			v1[2]=coords[indices[k]].z();

			v2[0]=coords[indices[k+1]].x();
			v2[1]=coords[indices[k+1]].y();
			v2[2]=coords[indices[k+1]].z();

			v3[0]=coords[indices[k+2]].x();
			v3[1]=coords[indices[k+2]].y();
			v3[2]=coords[indices[k+2]].z();

			volume.AddTri(v1,v2,v3,triInd++);
		}
	}
	volume.EndModel();

	PQP_REAL pt[3];
	pt[0] = (double)(dis[0]*mGrasp->getMaxRadius());
	pt[1] = (double)(dis[1]*mGrasp->getMaxRadius());
	pt[2] = (double)(dis[2]*mGrasp->getMaxRadius());

	vec3 disturbance(pt[0], pt[1], pt[2]);
	double originalLength = disturbance.len();
	double startValue = 0, endValue = originalLength;

	int loops = 0;
	while (!isOutside(&volume, pt) ) {
		pt[0] *= 2; pt[1] *= 2; pt[2] *= 2;
		startValue = endValue;
		endValue *= 2;
		loops ++;
		if (loops > 20) {
			DBGA("Failed to find point outside of volume!!!!!");
			break;
		}
	}

	disturbance = normalise(disturbance);
	double length = binarySearch(disturbance, volume, startValue, endValue);

	return originalLength / length;
}

double ArizonaTest::binarySearch(const vec3 &direction, PQP_Model &volume, double startValue, double endValue)
{
	//we assume that startValue is inside the model and endValue is outside the model
	//we find the value in between that is exactly on the model

	double currentValue = startValue;
	double currentInterval = (endValue - startValue)/2.0;

	PQP_REAL pt[3];
	PQP_REAL R[3][3]={{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0}};
	PQP_REAL T[3]={0.0,0.0,0.0};
	double closest_dist = 1.0e9, thresh = 0.0, distance;
	PQP_REAL closest_pt[3], closest_normal[3];

	bool success = false;
	while (1) {
		pt[0] = currentValue * direction.x();
		pt[1] = currentValue * direction.y();
		pt[2] = currentValue * direction.z();

		distance = GetShortestDist(pt, &volume, R, T,  closest_dist, closest_pt, closest_normal, thresh);
		DBGP("disturbance: " << pt[0] << " " << pt[1] << " " << pt[2]);
		DBGP("Closest point: " << closest_pt[0] << " " << closest_pt[1] << " " << closest_pt[2] << "; Distance is: " << distance);
		DBGP("Current interval: " << currentInterval);

		if(distance < EPSILON){
			success = true;
			break;
		}

		if(isOutside(&volume, pt)){ 
			DBGP("outside" << std::endl);
			if (currentValue <= startValue) {
				DBGP("Error: startValue is outside of model!");
				break;
			}
			currentValue -= currentInterval;
		} else {
			DBGP("inside" << std::endl);
			if (currentValue >= endValue) {
				DBGP("Error: endValue is inside model!");
				break;
			}
			currentValue += currentInterval;
		}
		currentInterval /= 2;
		if ( fabs(currentInterval) < 1.0e-10 ) {
			DBGP("Max loops exceeded!");
			break;
		}
	}
	if (distance > 10e-5) {
		DBGA("Binary search failed! distance is " << distance);
	} else {
		DBGP("Binary search success!");
	}
	return currentValue;
}

double ArizonaTest::getMinimunForce(int i){
	return getScale2Surface(getDisturbance(i,1.0,mObject->getMaxRadius()));
}

bool ArizonaTest::isOutside(PQP_Model* model, double* pt){
	PQP_REAL closest_pt[3], closest_normal[3];
	PQP_REAL R[3][3]={{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0}};
	PQP_REAL T[3]={0.0,0.0,0.0};
	double closest_dist = 1.0e9, thresh = 0.0;
	// compute the original distance from the model
	double originalDistance = GetShortestDist(pt, model, R, T, 
											closest_dist, closest_pt, closest_normal, thresh);

	vec3 point(pt);
	vec3 closest(closest_pt);
	std::cerr.precision(10);
	DBGP("Original point: " << pt[0] << " " << pt[1] << " " << pt[2] << " Len: " << point.len());
	DBGP("Closest point: " << closest_pt[0] << " " << closest_pt[1] << " " << closest_pt[2] << " Len: " << closest.len());
	if (closest.len() > point.len()) {
		return false;
	} else {
		return true;
	}

	double offset = originalDistance > point.len() ? point.len() : originalDistance;
	// let's go closer to the origin
	double newPointLength = point.len() - offset/2.0; 

	// create a new reference point
	point.set(normalise(point) * newPointLength);
	pt[0] = point.x();pt[1] = point.y();pt[2] = point.z();
	double newDistance = GetShortestDist(pt, model, R, T, 
									   closest_dist, closest_pt, closest_normal, thresh);

	if(newDistance < originalDistance){ // must be outside the model
		return true;
	}
	else{
		return false;
	}
}


void ArizonaTest::addDisturbance2GWS(GWSprojection* gp){
	SoSeparator *disturbanceSphere = new SoSeparator;
	double scale = mObject->getMaxRadius();
	for(int i = 1; i <= NUMCMPATTERN; i ++){
		if(i == mCMPattern)
			createDisturbanceSeparator(i,disturbanceSphere,scale,1.0,(i == mCMPattern));
	}
/*
	for(int i = 1; i <= NUMCMPATTERN; i ++)
	  createDisturbanceSeparator(i,disturbanceSphere,scale,forceScales[i-1],(i == mCMPattern));
*/
	gp->getGWSRoot()->addChild(disturbanceSphere);
}

void ArizonaTest::createDisturbanceSeparator(int trialNumber, SoSeparator * disturbanceSphere, double scale, double forceScale, bool isOriginal){
//scale is for the visualization, forceScale is the sum of normal force to be applied to the object, it also scales the volumn
// isOriginal distinguish the color
	if(mIsFlipped){
		scale = - scale;
	}
	double* disturbance = getDisturbance(trialNumber,forceScale, scale);
	double fz = *(disturbance+2);
	double Xtx = *(disturbance);
	double Xty = *(disturbance+1);
	free(disturbance);

//  scale /= ((-fz) * 5.0); // 1N - object; 1N - normal sum
//  printf("%d:%f,%f,%f\n",trialNumber,fz,Xtx,Xty);
  SoSeparator * ss = new SoSeparator();
  SoSphere *sp = new SoSphere();
  sp->radius = 5;
  SoMaterial *material = new SoMaterial;
  if(isOriginal){
	  material->diffuseColor.setValue(.9f,.2f,.2f);
	  material->ambientColor.setValue(.9f,.2f,.2f);
	  material->shininess = 0.8f;
	  material->transparency = 0.0f;
	  DBGP(scale * fz << " " << scale * Xtx << " " << scale * Xty);
  }
  else {
	  material->diffuseColor.setValue(.78f,.57f,.11f);
	  material->ambientColor.setValue(.2f,.6f,.2f);
	  material->shininess = .28f;
	  material->transparency = 0.4f;
  }

  SoTransform *transform = new SoTransform;
  //fprintf(stdout,"\nscale:%lf\n",scale);
  transform->translation = SbVec3f(scale * Xtx, scale * Xty, fz * scale);//tx,ty,fz
//  printf("here is the scaler, only for the axes %lf,%lf\n",scale,HULLAXES_SCALE);

  ss->addChild(material);
  ss->addChild(transform);
  ss->addChild(sp);
  disturbanceSphere->addChild(ss);
}

double* ArizonaTest::getDisturbance(int trialNumber, double forceScale, double radius)
{
	double fz, tx, ty, pi = M_PI;
	tx = ty = 0;
	fz = - 2.15;
	float scalex = 1/((-fz) * forceScale); // 1N - object; 1N - normal sum

	switch (trialNumber) {
	  case 1:
		  tx = fz * 70.0 * cos(18 * pi / 180);
		  break;
	  case 2:
		  tx = - fz * 0.5 * 70 * sin(36 * pi / 180); //(70 * sin(36 * pi / 180.0)) / sin(72 * pi / 180.0) * cos(18 * pi / 180.0) * 0.5;
		  ty = fz * 0.5 * 70 * (cos(36 * pi /180) + 1.0); //( 70 - (70 * sin(36 * pi / 180.0)) / sin(72 * pi / 180.0) * sin(18 * pi / 180.0) * 0.5 );
		  break;
	  case 3:
		  tx = - fz * 0.5 * 70 * sin(36 * pi / 180); //(70 * sin(36 * pi / 180.0)) / sin(72 * pi / 180.0) * cos(18 * pi / 180.0) * 0.5;
		  ty = - fz * 0.5 * 70 * (cos(36 * pi /180) + 1.0); //( 70 - (70 * sin(36 * pi / 180.0)) / sin(72 * pi / 180.0) * sin(18 * pi / 180.0) * 0.5 );
		  break;
	  case 4:
		  tx = ty = 0;
		  break;
	}

/*
	double x1 = 70, x2 = 70 * cos(36 * pi / 180), x3 = 70 * cos(72 * pi / 180);
	double y1 = 0, y2 = 70 * sin(36 * pi / 180), y3 = 70 * sin(72 * pi / 180);
	double xc1 = 0.5 * (x1 + x2), xc2 = 0.5 * (x2 + x3);
	double yc1 = 0.5 * (y1 + y2), yc2 = 0.5 * (y2 + y3);
	switch (trialNumber) {
	  case 1:
		  tx = fz * y3;
		  break;
	  case 2:
		  tx = - fz * y3;
		  break;
	  case 3:
		  tx = fz * yc1;
		  ty = fz * xc1;
		  break;
	  case 4:
		  tx = - fz * yc2;
		  ty = fz * xc2;
		  break;
	  case 5:
		  tx = fz * yc1;
		  ty = - fz * xc1;
		  break;
	  case 6:
		  tx = - fz * yc2;
		  ty = - fz * xc2;
		  break;
	  case 7:
		  tx = ty = 0.0;
		  break;
	}
*/
	double Xtx = tx/radius;
	double Xty = ty/radius;
	double *result = new double[3];
	result[0]= Xtx*scalex;
	result[1]= Xty*scalex;
	result[2]= fz*scalex;
	return result;
}


