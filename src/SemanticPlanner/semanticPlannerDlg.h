#ifndef SEMANTICPLANNERDLG_H
#define SEMANTICPLANNERDLG_H

#include <cv.h>

#include "ui_semanticPlannerDlg.h"

#include <QDialog>

#include "SemanticMap.h"
#include "matvec3D.h"
#include "DBase/scan_manager.h"
#include "SoArrow.h"
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoTranslation.h>
#include "UncertaintyPlanner/uncertaintyPlannerUtil.h"
#include "shapeCorrespondence.h"

class GraspitDBModel;
class EGPlanner;
class GraspPlanningState;
class SoSensor;
class Hand;
class GraspViewer;
//class SoTimerSensor;

namespace Ui {
    class SemanticPlannerDlg;
}

struct augmentedInt
{
int value;
double intensity;
};

class SemanticPlannerDlg : public QDialog
{
    Q_OBJECT

public:
    explicit SemanticPlannerDlg(QWidget *parent = 0);
    ~SemanticPlannerDlg();
	void setCurrentLoadedModel(GraspitDBModel *m){ mCurrentLoadedModel = m; }

private:
    Ui::SemanticPlannerDlg *ui;
	SemanticMap mSemanticMap;

	double mTheta, mPhi;
	GraspitDBModel *mCurrentLoadedModel;
	Hand* mHand;
	std::vector<double> mScanDepth;
	ScanManager sm;
	std::vector<compareItem> mNeighbors;
	//! index for visualize the sorted neighbors
	int mNNIdx;
	//!
	compareItem mVotingWinner;
	std::vector<compareItem> mDistList;
	int mCurrentSolution;


	//! grasp planner
	EGPlanner* mPlanner;
	GraspPlanningState* mHandObjectState;
	//used to exit and kill the current planner
	//SoTimerSensor *mTimerSensor;

	//visualize the arrow
	SoSeparator *aSep;
	SoRotation *aRot;
	SoRotation *bRot;
	SoTranslation *aTransl;
	SoArrow *arrow;

	//grasp viewer
	GraspViewer* mGraspViewer;

	//correspondence part
	//cv::flann::Index* mIndexFLANN;//flann object for knn search
	//std::vector<int> mCorrespondence;//the mapping between two shapes
	//std::vector<position> mOriginalVertices;
	ShapeCorrespondence mShapeCorrespondence;
	UncertaintySpaceVisualizer mVisualizer;

	void scan1();
	void scan2();
	//bool checkWithin(vec3 origin, vec3 corner, double vertical, double horizontal, vec3 up, vec3 approach);
	void displayArrow(int idx);
	void collectTactileFeedback();
	
	//for grasp planning
	void plan();
	void storeGrasps();
	//after complete is received, this is started
	static void sensorCB(void *data,SoSensor*);

	//correspondence
	void paintCorrespondence(std::vector<augmentedVec3> vertices);

public slots:
	void loadScanbookButton_clicked();
	void scanButton_clicked();
	//////void moveHandButton_clicked();
	void encodeButton_clicked();
	void getNNButton_clicked();
	void bestButton_clicked();
	void previousNeighborButton_clicked();
	void nextNeighborButton_clicked();
	void setHandButton_clicked();
	void planButton_clicked();
	void voteButton_clicked();
	void tactileButton_clicked();
	void adjustHandButton_clicked();
	void loadSemanticGraspsButton_clicked();
	void pauseButton_clicked();
	void rankButton_clicked();
	void previousButton_clicked();
	void nextButton_clicked();
	void refineButton_clicked();
	void storeButton_clicked();
	void showExamplePreGraspButton_clicked();
	void similarityMeasureButton_clicked();
	void executeButton_clicked();
	void showCorrespondenceButton_clicked();
	void loadCorrespondenceButton_clicked();
	void loadSGCorrButton_clicked();
	void computeTransformButton_clicked();

	void update();
	void complete();

};

#endif // SEMANTICPLANNERDLG_H
