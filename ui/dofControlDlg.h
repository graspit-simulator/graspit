#ifndef _DOFCONTROLDLG_H_
#define  _DOFCONTROLDLG_H_
#include "ui_dofControlDlg.h"

#include <vector>

#include <QLayout>
#include <QLabel>
#include <QScrollBar>
#include <QCheckBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialog>

class World;
class Hand;
class DOFControlDlg : public QDialog, public Ui::DOFControlDlgUI
{
	Q_OBJECT

private:
	//! The resolution of the sliders, set through Qt
    int SLIDER_STEPS;

	World *mWorld;
	Hand *mHand;

	//! Holds all the dof sliders and any controls that is controlable
	QDialog *mSubLayout;
    std::vector<QLabel*> mValueList;
    std::vector<QScrollBar*> mBarList;
    //std::vector<QCheckBox*> mCheckList;
	std::vector<int> mDofMap;
    QVBoxLayout *mainLayout;

	void init();
	void destroy(){};
	int getNumDOFs();
	void readRobotDOF();
public:
	DOFControlDlg(QWidget *parent = 0, World* w = NULL) : QDialog(parent), mWorld(w){
			setupUi(this);
			/*
			QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitButton_clicked()));
			QObject::connect(connectToStaubliButton, SIGNAL(clicked()), this, SLOT(connectToStaubliButton_clicked()));
			QObject::connect(getStaubliStatusButton, SIGNAL(clicked()), this, SLOT(getStaubliStatusButton_clicked()));
			QObject::connect(staubliGoButton, SIGNAL(clicked()), this, SLOT(staubliGoButton_clicked()));
			QObject::connect(initializeBarrettButton, SIGNAL(clicked()), this, SLOT(initializeBarrettButton_clicked()));
			QObject::connect(barrettGoButton, SIGNAL(clicked()), this, SLOT(barrettGoButton_clicked()));
			QObject::connect(getBarrettStatusButton, SIGNAL(clicked()), this, SLOT(getBarrettStatusButton_clicked()));
			QObject::connect(getLocButton, SIGNAL(clicked()), this, SLOT(getLocButton_clicked()));
			QObject::connect(barrettCloseButton, SIGNAL(clicked()), this, SLOT(barrettCloseButton_clicked()));
			QObject::connect(barrettOpenButton, SIGNAL(clicked()), this, SLOT(barrettOpenButton_clicked()));
			QObject::connect(getBarrettButton, SIGNAL(clicked()), this, SLOT(getBarrettButton_clicked()));*/
			init();
	}
	void setLayout(int num);

public slots:
	void dofChanged();
	void fixBoxChanged();
};
#endif // _DOFCONTROLDLG_H_