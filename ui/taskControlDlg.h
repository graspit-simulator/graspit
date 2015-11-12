#ifndef _TASKCONTROLDLF_H_
#define _TASKCONTROLDLF_H_
#include "ui_taskControlDlg.h"
#include <QTextStream>

#include "matvec3D.h"

class GraspableBody;
class QualEpsilon;
class QualVolume;

class TaskControlDlg : public QDialog, public Ui::TaskControlDlgUI
{
	Q_OBJECT

private:
	transf mPose;
	static int mExpId;
	//! Epsilon quality measure
	QualEpsilon* mEpsQual;
	//! Volume quality measure
	QualVolume* mVolQual;
	//! the list of poses generated
	std::vector<transf> mAp;

	void writeOutGEDFromWorldFiles();
	void writeOutGEDFromPoses_deprecated();


	void writeOutExpResults();
	void writeOutTactileInfo(QTextStream & qts);
	void intelligentWriteOutPoses();
	void writeOutCurrentPoses();
	void init();
	//void resetDOFs();

public:
	TaskControlDlg(QWidget *parent = 0) : QDialog(parent), mEpsQual(NULL), mVolQual(NULL) {
		setupUi(this);
		QObject::connect(goButton, SIGNAL(clicked()), this, SLOT(goButton_clicked()));
		QObject::connect(testButton, SIGNAL(clicked()), this, SLOT(testButton_clicked()));
		init();
	}

public slots:
	void goButton_clicked();
	void testButton_clicked();
};
#endif
