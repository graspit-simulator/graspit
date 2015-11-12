
#ifndef _ARIZONAPROJECTDLG_H_
#define _ARIZONAPROJECTDLG_H_

#include "ui_arizonaProjectDlg.h"
#include <QDialog>

class ArizonaProcess;
class ArizonaProjectDlg : public QDialog, public Ui::ArizonaProjectDlgUI
{
	Q_OBJECT
private:
    ArizonaProcess* mAp;
	void updateDlg();
public:
	ArizonaProjectDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButton_clicked()));
		QObject::connect(processButton, SIGNAL(clicked()), this, SLOT(processButton_clicked()));
		QObject::connect(EllipsoidFileButton, SIGNAL(clicked()), this, SLOT(EllipsoidFileButton_clicked()));
		QObject::connect(loadObjectFileButton, SIGNAL(clicked()), this, SLOT(loadObjectFileButton_clicked()));
		QObject::connect(destinationFolderButton, SIGNAL(clicked()), this, SLOT(destinationFolderButton_clicked()));
		QObject::connect(saveVgrButton, SIGNAL(clicked()), this, SLOT(saveVgrButton_clicked()));
		QObject::connect(prevExpButton, SIGNAL(clicked()), this, SLOT(prevExpButton_clicked()));
		QObject::connect(nextExpButton, SIGNAL(clicked()), this, SLOT(nextExpButton_clicked()));
		QObject::connect(testAllButton, SIGNAL(clicked()), this, SLOT(testAllButton_clicked()));
		QObject::connect(showGWSButton, SIGNAL(clicked()), this, SLOT(showGWSButton_clicked()));
		QObject::connect(selectRawFileButton, SIGNAL(clicked()), this, SLOT(selectRawFileButton_clicked()));

		QObject::connect(isBuildIn3DCheckBox, SIGNAL(stateChanged(int)), this, SLOT(isBuildIn3DCheckBox_clicked()));
		QObject::connect(isFlippedCheckBox, SIGNAL(stateChanged(int)), this, SLOT(isFlippedCheckBox_clicked()));


		init();
	}
	~ArizonaProjectDlg(){destroy();}
public slots:
	void init();
	void destroy(){}
	void exitButton_clicked();
	void processButton_clicked();
    void EllipsoidFileButton_clicked();
	void loadObjectFileButton_clicked();
	void destinationFolderButton_clicked();
	void saveVgrButton_clicked();
	void prevExpButton_clicked();
	void nextExpButton_clicked();
	void is3DCheckBox_clicked();
	void testAllButton_clicked();
	
	void showGWSButton_clicked();
	void isFlippedCheckBox_clicked();
	void selectRawFileButton_clicked();
};
#endif