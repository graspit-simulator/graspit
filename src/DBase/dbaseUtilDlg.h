#ifndef DBASEUTILDLG_H_
#define DBASEUTILDLG_H_

#include <QDialog>
#include "ui_dbaseUtilDlg.h"
#include "matvec3D.h"


namespace db_planner
{
	class DatabaseManager;
	class Model;
}
class EGPlanner;
class GraspPlanningState;
class Hand;
class GraspitDBModel;
class GraspitDBGrasp;
class SoTimerSensor;
class SoSensor;

class DBaseUtilDlg : public QDialog, public Ui::DBaseUtilDlgUI
{
	Q_OBJECT	
private:
	db_planner::DatabaseManager* mDBMgr;
	GraspPlanningState* mHandObjectState;
	EGPlanner* mPlanner;
	//! The list of models available in the dbase, as retrieved by the DBMgr
	std::vector<db_planner::Model*> mAllModel, mModelList;
	Hand * mHand;
	GraspitDBModel * mCurrentModel;
	int mNumGrasps;
	//whether the current model is done
	bool mNext;
	//the index of the model being planned
	int mCurrentIndex;
	//used to exit and kill the current planner
	SoTimerSensor *mTimerSensor;
	//log file
	FILE* mLog;
	transf mRefTran;

	void init();
	void destroy();
	//do the necessary cleaning
	void cleanUp();
	void loadAnotherHand();
	void loadNewObject();
	void plan();
	void storeGrasps();
	//after complete is received, this is on to start a new planning
	static void sensorCB(void *data,SoSensor*);
	//synthesize a CGDB grasp based on pre-fin-grasp
	GraspitDBGrasp* synthesize(GraspPlanningState* pre, GraspPlanningState* fin);

public:
	DBaseUtilDlg(QWidget *parent = 0, db_planner::DatabaseManager* dbm = NULL) : QDialog(parent),
		mDBMgr(dbm), mPlanner(NULL), mHandObjectState(NULL), mNext(true), mCurrentIndex(4),
	mCurrentModel(NULL), mTimerSensor(NULL){
		setupUi(this);
		QObject::connect(runPlannerButton, SIGNAL(clicked()), this, SLOT(runPlannerButton_clicked()));
		QObject::connect(runPlannerOnCGDBButton, SIGNAL(clicked()), this, SLOT(runPlannerOnCGDBButton_clicked()));
		QObject::connect(takeScansButton, SIGNAL(clicked()), this, SLOT(takeScansButton_clicked()));
		init();
	}
	~DBaseUtilDlg(){destroy();}
	//whether we can start a new planning
	bool readyForNext(){return mNext;}

public slots:
	void runPlannerButton_clicked();
	void runPlannerOnCGDBButton_clicked();
	void update();
	void complete();
	void nextObject(){cleanUp(); loadAnotherHand(); loadNewObject(); plan(); mCurrentIndex++;}
	void takeScansButton_clicked();

};
#endif //DBASEUTILDLG_H_