#include "graspAnalyzingTask.h"
#include "db_manager.h"
#include <vector>
#include "graspit_db_grasp.h"
#include "searchEnergy.h"
#include <QTextStream>
#include <QFile>
#include "debug.h"
#include <algorithm>
#include "searchState.h"
#include "matvecIO.h"
#include <QProcess>
#include "eigenGrasp.h"
#include "grasp.h"

using namespace std;

GraspAnalyzingTask::GraspAnalyzingTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec)
  :Task(disp, mgr, rec), waitLength(0){
  if(!  getGrasps())
    {
      DBGA("GraspAnalyzingService: Fatal error.  Not able to connect to get grasps from CGDB");
      exit(1);
    };
 }


//helper function for ranking grasps
bool compareGraspEpsilonQM(db_planner::Grasp* g1, db_planner::Grasp* g2)
{
	return g1->EpsilonQuality() > g2->EpsilonQuality();
}

/*Load the appropriate hand and graspable body into the world from the task record
*/
bool GraspAnalyzingTask::getGrasps(){

  //put the hand in the world if necessary:
  World *world = graspItGUI->getIVmgr()->getWorld();
  
  //check if the currently selected hand is the same as the one we need
  //if not, load the hand we need
  if (world->getCurrentHand() && 
      GraspitDBGrasp::getHandDBName(world->getCurrentHand()) == QString(mRecord.handName.c_str())) {
    DBGA("Grasp Planning Task: using currently loaded hand");
    mHand = world->getCurrentHand();
  } else {
    mHand = GraspitDBGrasp::loadHandFromDBName(QString(mRecord.handName.c_str()));
    if ( !mHand || !mDBMgr->setHand(mHand)) {
      DBGA("Failed to load hand");
      mStatus = ERROR;
      return false;
    }
  }
  /* FIXME: 
     if there is anything in contact with the hand, 
     we should probably remove it here
  */
  
  mDBMgr->setHand(mHand);
  vector<string> taskNameList;
  mDBMgr->TaskTypeList(&taskNameList);
  //see if this task is performed only on a single grasp -- assume grasps are sorted by epsilon quality
  unsigned int graspPos = taskNameList[mRecord.taskType-1].find("GRASP_NUM_");
  
  //get grasps from dbmgr

    mDBMgr->GetGrasps(*mRecord.model, mRecord.handName, &mGraspList);
    //sort by epsilon quality
    sort(mGraspList.begin(), mGraspList.end(), compareGraspEpsilonQM);
    //if we are only using one grasp, kick all of the others out of the list
  if( graspPos!= string::npos){
    unsigned int graspNum = QString(taskNameList[mRecord.taskType-1].substr(graspPos+10).c_str()).toInt();
    for(unsigned int i = 0; i < mGraspList.size(); ++i){
      if (i == graspNum) continue;
      delete mGraspList[i];
      
    }
   mGraspList[0] = mGraspList[graspNum];
      mGraspList.resize(1);
  }
  //get the approach type if the grasp type specifies it
  unsigned int aTypePos = taskNameList[mRecord.taskType-1].find("APPROACH_TYPE_");
  if( aTypePos!= string::npos){
    mApproachType = ApproachType(QString(taskNameList[mRecord.taskType-1].substr(aTypePos+14).c_str()).toInt());
  }
  else mApproachType = BY_MAGIC;
  
  //load the model into graspit
  GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(mRecord.model);
  //check that this model is already loaded into Graspit, if not, load it
  if (!model->geometryLoaded()) {
    //this loads the actual geometry in the scene graph of the object
    if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
      DBGA("GraspAnalyzingTask::getGrasp: Fatal Error Model load failed");
      return false;
    }
  }
  mObject = model->getGraspableBody();
  mObject->addToIvc();
  graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());  
  if(getenv("GRASPIT_WAIT_DISPLAY_LEN"))
    waitLength = QString(getenv("GRASPIT_WAIT_DISPLAY_LEN")).toInt();
  mHand->setRenderGeometry(waitLength);
  mObject->setRenderGeometry(waitLength);
  return true;
}



void GraspAnalyzingTask::run(){
  //set status to running
  mStatus = RUNNING;
  //get the grasps;
  //start the qthread
  threadLoop();
}

void GraspAnalyzingTask::threadLoop()
{

  while(mStatus == RUNNING){
    mainLoop();
  }
	DBGP("Thread is done!");
}

void GraspAnalyzingTask::mainLoop(){
  static unsigned int graspCounter = 0;
  if (graspCounter < mGraspList.size() && mStatus == RUNNING){
    analyzeGrasp(mGraspList[graspCounter++]);
  }else if(mStatus == RUNNING)
    mStatus = DONE;
}

class VectorPosCompare{
  vec3 v;

public:
  VectorPosCompare();
  VectorPosCompare(const vec3 & vec):v(vec){};
  bool operator ()(const position & p1, const position& p2){
    return (p1%v<p2%v);
  }
};


class VectorPerpendicularInBodyCoordinatesPosCompare{
  vec3 v;
  transf bodyTrans;  
public:
  VectorPerpendicularInBodyCoordinatesPosCompare(const vec3 & vec, const transf & t):v(vec), bodyTrans(t){};  
  bool operator()(const position & p1, const position & p2){
    return perpendicularDistInBTrans(p1) < perpendicularDistInBTrans(p2);
  }
  double perpendicularDistInBTrans(const position & p){
    position pInB = p*bodyTrans.inverse();
    //positions cannot be dotted with themselves because they aren't really meant to be used as vectors
    //    return sqrt(pInB%pInB - pInB%v);
    return sqrt(pInB.x()*pInB.x() + pInB.y()*pInB.y() + pInB.z()*pInB.z()- pow(pInB%v,2));
    
  }
};


inline double moveHandObjectOutOfWorkSpace(Hand * h, Body * b){
  vector<position> vertices;
  vec3 approachVec = vec3::Z * h->getApproachTran() * h->getTran();
  //the fingertip is the last link on each chain
  for (int k = 0; k < h->getNumChains(); ++k)
    h->getChain(k)->getLink(h->getChain(k)->getNumLinks()-1)->getGeometryVerticesInWorldCoordinates(&vertices);
  VectorPosCompare vcPlus(approachVec);
  //find the furthest point on the links along the approach direction
  position furthestPoint= *std::max_element(vertices.begin(), vertices.end(), vcPlus);
  double distanceToFurthestLinkPoint = furthestPoint%approachVec;
  vertices.clear();
  b->getGeometryVerticesInWorldCoordinates(&vertices);
  // find the closest point on the body along the approach direction
  position ClosestBodyPoint = *std::min_element(vertices.begin(), vertices.end(), vcPlus);
  double distanceToClosestBodyPoint = ClosestBodyPoint%approachVec;
  //find the distance to move the hand backwards along the approach direction
  double moveDist = distanceToClosestBodyPoint-distanceToFurthestLinkPoint;
  //FIXME something is broken about b->getGeometryVertices
 

  if(moveDist > 0)
    moveDist = 0;
    //now move the hand forward along the approach direction by 5 millimeters (arbitrarily) further than 
  //movedist (movedist is negative, so this will actually move backwards)
  h->setTran(translate_transf(vec3(0,0,moveDist - 5)*h->getApproachTran())*h->getTran());
  return moveDist;
}


inline void getFurthestDistanceOnChainPerpendicularToV(Hand * h, const vec3 & v, std::vector<double> & targetDistance){
  
  VectorPerpendicularInBodyCoordinatesPosCompare vc(v, h->getTran());
  for(int k = 0; k < h->getNumChains(); k++){
    std::vector<position> vertices;
    KinematicChain * kc = h->getChain(k);
    for(int i = 0; i < kc->getNumLinks()-1; ++i){
      kc->getLink(i)->getGeometryVerticesInWorldCoordinates(&vertices);
    }
    position maxPoint = *std::max_element(vertices.begin(), vertices.end() , vc);    
    targetDistance.push_back(vc.perpendicularDistInBTrans(maxPoint));
  }
}

inline double minFingerTipDistanceAlongPerpendicularToV(Hand *h, Body * b, const vec3 & v){
    VectorPerpendicularInBodyCoordinatesPosCompare vc(v, h->getTran());
    std::vector<position> vertices;
    b->getGeometryVerticesInWorldCoordinates(&vertices);
    //point on the finger closest furthest from the palm
    VectorPosCompare vcPlus(vec3::Z * h->getApproachTran()*h->getTran());
    position fingertipPoint = *std::max_element(vertices.begin(), vertices.end(), vcPlus);
   
    return vc.perpendicularDistInBTrans(fingertipPoint);
}

inline bool openFingertipsPastGraspOutline(Hand * h){
  std::vector<double> targetDistance;
  vec3 approachVec = vec3::Z;
  getFurthestDistanceOnChainPerpendicularToV(h, approachVec,targetDistance);
  for (int k = 0; k < h->getNumChains(); ++k){
    KinematicChain * kc = h->getChain(k);
    double d = minFingerTipDistanceAlongPerpendicularToV(h, kc->getLink(kc->getNumLinks()-1), approachVec);
    while(  d < targetDistance[k])
      {
	
	h->quickOpen(.05);
	double newd = minFingerTipDistanceAlongPerpendicularToV(h, kc->getLink(kc->getNumLinks()-1), approachVec);
	if (fabs(newd - d) < 0.01)
	  return true;
	d = newd;
	
      }
  }
  h->quickOpen(.1);
  return true;
}

int cgid;
inline bool GraspAnalyzingTask::getPregraspPlanningState(GraspPlanningState * initialPreGrasp, GraspPlanningState * finalPreGrasp, transf newObjectTrans)
{
  // Assume you approach the object in the pregrasp position
  finalPreGrasp->copyFrom(initialPreGrasp);

  //put the hand in this position
  finalPreGrasp->execute();
  sleep(waitLength);
  switch(mApproachType){
  case BY_MAGIC: // fall through for now
  case IN_PREGRASP:
    {
      //the hand is already in pregrasp, do nothing
      break;
    }
  case COMPLETELY_OPEN:
    {

      //if this is allows to render, you may need to wait for the complete signal before continuing
      mHand->autoGrasp(false, -1, false);
      sleep(waitLength);    
      break;
    }
  case FINGER_APERATURE_TO_FINALGRASP_KNUCKLE:{
    openFingertipsPastGraspOutline(finalPreGrasp->getHand());
    //    DBGA("Fingers opened to knuckle");
    QFile f(mHand->getName() +"_" +  QString::number(cgid) + ".txt");
    f.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream os(&f);
    os   <<"Hand Transform " <<mHand->getTran() << " , DOF: " ;
    mHand->writeDOFVals(os);
    sleep(waitLength);    
    break;
  }
  }
  

  //now set the object to its perturbed position
  mObject->setTran(newObjectTrans);
  double approachDist = 0.0;
  //if we there exists a strategy that approaches along a straight line and puts the object in the pregrasp pose by 
  //magic, don't back object out of workspace
  if (mApproachType != BY_MAGIC &&  mApproachType !=COMPLETELY_OPEN){
  //now withdraw the hand so the object is no longer in the capture volume
    approachDist = moveHandObjectOutOfWorkSpace(finalPreGrasp->getHand(), finalPreGrasp->getObject());
    //    DBGA("Hand moved back: " <<approachDist );
    //  os   <<"Hand Transform " <<mHand->getTran() << " , DOF: " ;
    //  mHand->writeDOFVals(os);
    sleep(waitLength);    
  }

  //if in collision, find first contact from here
    finalPreGrasp->getHand()->findInitialContact(-approachDist);
    //  DBGA("Moved back");
    sleep(waitLength);         
  
 //Move DOF to pregrasp pose if possible
  int numDOF = finalPreGrasp->getHand()->getNumDOF();
  double * handDOF = new double[numDOF];
  double * handDOFSpeed = new double[numDOF];
  for (int i=0;i<numDOF;i++) {
   handDOFSpeed[i] = .01;
  }
  initialPreGrasp->readPosture()->getHandDOF(handDOF);
  if(!finalPreGrasp->getHand()->moveDOFToContacts( handDOF, handDOFSpeed, false, false)){
    //    DBGA("moveToContact failed");
    //    sleep(5*waitLength);
  }
  //  DBGA("Fingers moved to contact");
  //os   <<"Hand Transform " <<mHand->getTran() << " , DOF: " ;
  //mHand->writeDOFVals(os);
    sleep(waitLength);  
  // now save this position has the real pregrasp position
  //where we are now is where we want to analyze the energy from
  //save the new reftran and hand state
  finalPreGrasp->setRefTran(finalPreGrasp->getObject()->getTran());
  finalPreGrasp->saveCurrentHandState();
  //wait the wait length to let user interact if necessary

  delete [] handDOF;
  delete [] handDOFSpeed;
  return finalPreGrasp->getHand()->getNumContacts();
}



//Perturb the hand as though the object has been moved by transf perturbation - then try to execute grasp
inline bool GraspAnalyzingTask::analyzePerturbedState(GraspPlanningState * p,  const transf &perturbation, 
				  SearchEnergy * se, QTextStream & os){
  GraspPlanningState final(p);
  
  //pretend the object has moved in the opposite direction as we want to simulate it having actually moved
  getPregraspPlanningState(p, &final, p->getObject()->getTran()*perturbation);
  bool legal;
  double energy;
  se->disableRendering(waitLength==0);
  se->analyzeState(legal, energy, &final, false);
  DBGA("stateAnalyzed");
  sleep(waitLength);    

  transf handTrans(final.getHand()->getTran()), objTrans(final.getObject()->getTran());
  double volumeQual(0.0), epsilonQual(0.0);
  if(mHand->getNumContacts()>2){
    epsilonQual = se->getEpsQual();
    volumeQual = se->getVolQual();
  }
  DBGA( "Epsilon Quality:" <<  epsilonQual << " Volume Quality " << volumeQual); 
    //output the perturbation and final grasp to os
  os << objTrans << " , " 
     << handTrans << " , "
     << perturbation  <<" , " ;
  final.getHand()->writeDOFVals(os);
  os << " , " <<p->getEnergy() << " , " 
     << epsilonQual << " , " 
     << volumeQual << " , ";
  RobotTools::writeContactLinks(mHand, mObject, os);
  os << ", ";
  RobotTools::writeContactsBody(mHand, mObject, os);
  os << "\n";
  return true;
    
}

bool zipOutputFile(QString fname, bool deleteOldFile){
  QProcess *zipProc=new QProcess ();
  QStringList arglist;
  arglist.push_back(fname + ".zip");
  arglist.push_back(fname);
  zipProc->start("zip",arglist);
  if (!zipProc->waitForStarted())
    {   
      DBGA("unzip process could not be started");
      return false;
    } 
    // zip could be started
  if(!zipProc->waitForFinished())
    {
      //zip proc failed to finish?
      DBGA("unzip process could not finish");
      return false;
    } 
  if (zipProc->exitStatus() == QProcess::CrashExit)
    {
      DBGA("unzip process failed");
      return false;
    }  
  delete zipProc;
  
  if(!deleteOldFile)
    return true;
  return QFile::remove (fname) ;

}


bool GraspAnalyzingTask::analyzeGrasp(db_planner::Grasp* currentGrasp){
  QFile file("./" + mObject->getName() +'_' + GraspitDBGrasp::getHandDBName(mHand) +'_' + QString::number(mRecord.taskId));
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){
    mStatus = ERROR;
    return false;
  }
  QTextStream out(&file);
  SearchEnergy se;
   GraspPlanningState pregrasp(static_cast<GraspitDBGrasp*>(currentGrasp)->getPreGraspPlanningState());
  transf originalModelTrans = mObject->getTran();
  pregrasp.setObject(mObject);
  pregrasp.getHand()->getGrasp()->setObject(pregrasp.getObject());
  pregrasp.setRefTran(mObject->getTran());
  /*the original final pose is found using energy_strict_autograsp which only goes 
  3 cm forward and then moves back if it makes no palm contact
  */
  se.setType(ENERGY_STRICT_AUTOGRASP); 
  string  object_name(mRecord.model->ModelName());
  //this first line is always by magic - its the baseline preplanned grasp
  
  //export dbase identifying stuff
  out << object_name.c_str()  << " ,  " 
      << GraspitDBGrasp::getHandDBName(mHand) << " , " << -1 << " , " 
     <<mRecord.taskId << " , " << currentGrasp->GraspId() << " , ";
  //fixme delete this later -hack to get initial dofs
  cgid = currentGrasp->GraspId();
  ApproachType currentType=   mApproachType;
  mApproachType = BY_MAGIC;
  analyzePerturbedState(&pregrasp, transf::IDENTITY, &se,  out);
  mApproachType = currentType;
  
  sleep(waitLength);
  //Generate the perturbations
  /* Assume object is in its canonical direction aligned with Y
   * move object in x +/- 10, move object in y +/- 10
   * and rotate in around the Y axis of the center of the grip
   */
 
  //get mean contact location for of unperturbed grasp in mObjects 
  //coordinates.  Ignore contacts with objects that are not the hand
  // instead of iterating over each link and filtering it out
  // this gets all of the contacts on the hand that are on the object
  //and gets the mate of each contact

  list<Contact*> clist = mHand->getContacts();
  position contactCenter(0.0, 0.0, 0.0);
  int contactNum = 0;
  for (list<Contact *>::iterator citer = clist.begin(); citer != clist.end(); ++citer){
    contactCenter = contactCenter + (*citer)->getMate()->getPosition();
    contactNum ++;
  }
  transf contactCenterTrans( mat3::IDENTITY, vec3(contactCenter[0],contactCenter[1],contactCenter[2])/contactNum);
  transf invContactCenterTrans(mat3::IDENTITY, -contactCenterTrans.translation());
  /*In this test, we will move forward as much as 5 cm attempting to make contact, and close there if we dont.*/
  se.setType(ENERGY_CONTACT_AUTOGRASP);
  for (double x = -10.0; x <=10.0; x+=1.0){
    for (double z = -10.0; z <=10.0; z+=1.0){
      for (double theta = -20.0; theta <= 20.0; theta +=1.0){
	transf rotateAroundZ(invContactCenterTrans*rotXYZ(0.0, theta/180.0*M_PI, 0.0)*contactCenterTrans);
	transf posMove(mat3::IDENTITY, vec3(x,0,z));
	//export dbase identifying stuff
	out << mRecord.model->ModelName() .c_str() << " ,  " << GraspitDBGrasp::getHandDBName(mHand) << " , " << mRecord.taskType << " , "
	   <<mRecord.taskId << " , " << currentGrasp->GraspId() << " , ";
	mObject->setTran(originalModelTrans);
	analyzePerturbedState(&pregrasp, rotateAroundZ*posMove, &se,  out);
      }
  out.flush();
    }
  }
  out.flush();
  file.flush();
  file.close();
  zipOutputFile(file.fileName(), true);
  return true;	
}
  


Task* GraspAnalyzerFactory::getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
		db_planner::TaskRecord rec){
  // if the task type is not one which this Planner can handle, return null
  GraspAnalyzingTask *  ga = new GraspAnalyzingTask(disp, mgr, rec);
  //  if(!ga->getGrasps()){
  //DBGA("GraspAnalyzerFactory::getGrasp failed!");
  //return static_cast<Task *>(NULL);
  //}
  return ga;
};



  

GraspAnalyzingTask::~GraspAnalyzingTask()
{
  //delete all remaining grasp pointers
  for (vector<db_planner::Grasp*>::iterator gd = mGraspList.begin();
       gd != mGraspList.end(); ++gd){
    delete (*gd);
  }
  //  delete  mRecord.model;
  //mObject=NULL;
}
