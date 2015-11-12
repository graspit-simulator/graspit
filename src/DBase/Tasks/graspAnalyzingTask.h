#ifndef _GRASPANALYZINGTASK_H_
#define _GRASPANALYZINGTASK_H_

#include <QObject>
#include "robot.h"
#include "taskDispatcher.h"
#include <QThread>
class QTextStream;
class GraspitDBGrasp;
class GraspPlanningState;
class SearchEnergy;

class GraspAnalyzingTask :public QThread, public Task {
  enum ApproachType{BY_MAGIC = 0, IN_PREGRASP = 1, COMPLETELY_OPEN = 2, FINGER_APERATURE_TO_FINALGRASP_KNUCKLE= 3};
  ApproachType mApproachType;
  Q_OBJECT
  protected: 
  Hand * mHand;
  GraspableBody * mObject;
  std::vector<db_planner::Grasp*> mGraspList;
  virtual bool analyzeGrasp(db_planner::Grasp* currentGrasp);
  virtual void mainLoop();
  virtual void start(){mStatus = RUNNING; QThread::start();}
  void threadLoop();
  virtual void run();
  //amount of time to wait for after doing a grasp
  //useful for visual debugging

  int waitLength;
  inline bool analyzePerturbedState(GraspPlanningState * p,  const transf &perturbation, 
				    SearchEnergy * se, QTextStream & os);
  inline bool getPregraspPlanningState(GraspPlanningState * initialPregrasp, GraspPlanningState * finalPregrasp, transf newObjectTrans);
 public:
    //This task is a qthread - it inherits a start function
    //that will spawn a new thread and enter the "run" function from within it
    //Any objects added to the world cannot be added from inside the start and
    //so might want to be added from the constructor -JW
    GraspAnalyzingTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec);
    virtual ~GraspAnalyzingTask();
    virtual bool getGrasps();
    //  ~GraspAnalyzingTask();
};

class GraspAnalyzerFactory :public TaskFactory{
 public:
  GraspAnalyzerFactory(){};
  virtual Task* getTask(TaskDispatcher *disp, db_planner::DatabaseManager *mgr, 
			      db_planner::TaskRecord rec);
};




#endif
