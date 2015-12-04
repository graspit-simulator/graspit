/********************************************************************************
** Form generated from reading UI file 'egPlannerDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EGPLANNERDLG_H
#define UI_EGPLANNERDLG_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <vector>

QT_BEGIN_NAMESPACE

class Ui_EigenGraspPlannerDlgUI
{
public:
    QPushButton *exitButton;
    Q3GroupBox *groupBox3;
    QLabel *textLabel1;
    QPushButton *plannerResetButton;
    QPushButton *plannerInitButton;
    QPushButton *plannerStartButton;
    QComboBox *plannerTypeBox;
    QPushButton *instantEnergyButton;
    Q3GroupBox *groupBox8;
    QPushButton *nextGraspButton;
    QPushButton *bestGraspButton;
    QPushButton *prevGraspButton;
    QLabel *textLabel2_3;
    QComboBox *energyBox;
    QLabel *textLabel1_2;
    QCheckBox *setContactsBox;
    QLabel *textLabel1_5;
    QLabel *textLabel1_4;
    QLabel *currentStepLabel;
    QLabel *timeLabel;
    QFrame *line1_2;
    QFrame *line1_2_2_2;
    QLabel *rankLabel;
    QLabel *energyLabel;
    QLabel *iterationLabel;
    QLineEdit *annStepsEdit;
    Q3GroupBox *onlineDetailsGroup;
    QLabel *objDistLabel;
    QLabel *solDistLabel;
    QLabel *onlineStatusLabel;
    QLabel *fcBufferLabel;
    QLabel *saBufferLabel;
    QPushButton *onlineGraspButton;
    QCheckBox *autoGraspBox;
    QPushButton *onlineReleaseButton;
    QPushButton *onlinePlanButton;
    QFrame *line3;
    QCheckBox *showCloneBox;
    QCheckBox *showSolutionBox;
    QFrame *line4;
    QCheckBox *useVirtualHandBox;
    QCheckBox *useRealBarrettBox;
    Q3GroupBox *inputBox;
    QPushButton *inputLoadButton;
    QCheckBox *inputGloveBox;
    Q3GroupBox *variableBox;
    QComboBox *spaceSearchBox;
    QLabel *spaceSearchLabel;

    void setupUi(QDialog *EigenGraspPlannerDlgUI)
    {
        if (EigenGraspPlannerDlgUI->objectName().isEmpty())
            EigenGraspPlannerDlgUI->setObjectName(QString::fromUtf8("EigenGraspPlannerDlgUI"));
        EigenGraspPlannerDlgUI->resize(561, 552);
        exitButton = new QPushButton(EigenGraspPlannerDlgUI);
        exitButton->setObjectName(QString::fromUtf8("exitButton"));
        exitButton->setGeometry(QRect(10, 514, 60, 31));
        groupBox3 = new Q3GroupBox(EigenGraspPlannerDlgUI);
        groupBox3->setObjectName(QString::fromUtf8("groupBox3"));
        groupBox3->setGeometry(QRect(270, 220, 280, 90));
        groupBox3->setOrientation(Qt::Vertical);
        textLabel1 = new QLabel(groupBox3);
        textLabel1->setObjectName(QString::fromUtf8("textLabel1"));
        textLabel1->setGeometry(QRect(16, 21, 30, 20));
        textLabel1->setWordWrap(false);
        plannerResetButton = new QPushButton(groupBox3);
        plannerResetButton->setObjectName(QString::fromUtf8("plannerResetButton"));
        plannerResetButton->setGeometry(QRect(60, 50, 40, 30));
        plannerInitButton = new QPushButton(groupBox3);
        plannerInitButton->setObjectName(QString::fromUtf8("plannerInitButton"));
        plannerInitButton->setGeometry(QRect(10, 50, 40, 30));
        plannerStartButton = new QPushButton(groupBox3);
        plannerStartButton->setObjectName(QString::fromUtf8("plannerStartButton"));
        plannerStartButton->setGeometry(QRect(110, 50, 30, 30));
        plannerTypeBox = new QComboBox(groupBox3);
        plannerTypeBox->setObjectName(QString::fromUtf8("plannerTypeBox"));
        plannerTypeBox->setGeometry(QRect(50, 20, 140, 20));
        instantEnergyButton = new QPushButton(groupBox3);
        instantEnergyButton->setObjectName(QString::fromUtf8("instantEnergyButton"));
        instantEnergyButton->setGeometry(QRect(200, 50, 20, 31));
        groupBox8 = new Q3GroupBox(EigenGraspPlannerDlgUI);
        groupBox8->setObjectName(QString::fromUtf8("groupBox8"));
        groupBox8->setGeometry(QRect(270, 10, 280, 200));
        groupBox8->setOrientation(Qt::Vertical);
        nextGraspButton = new QPushButton(groupBox8);
        nextGraspButton->setObjectName(QString::fromUtf8("nextGraspButton"));
        nextGraspButton->setGeometry(QRect(80, 151, 30, 30));
        bestGraspButton = new QPushButton(groupBox8);
        bestGraspButton->setObjectName(QString::fromUtf8("bestGraspButton"));
        bestGraspButton->setGeometry(QRect(40, 151, 41, 30));
        prevGraspButton = new QPushButton(groupBox8);
        prevGraspButton->setObjectName(QString::fromUtf8("prevGraspButton"));
        prevGraspButton->setGeometry(QRect(10, 151, 30, 30));
        textLabel2_3 = new QLabel(groupBox8);
        textLabel2_3->setObjectName(QString::fromUtf8("textLabel2_3"));
        textLabel2_3->setGeometry(QRect(27, 129, 63, 20));
        textLabel2_3->setWordWrap(false);
        energyBox = new QComboBox(groupBox8);
        energyBox->setObjectName(QString::fromUtf8("energyBox"));
        energyBox->setGeometry(QRect(140, 20, 111, 21));
        textLabel1_2 = new QLabel(groupBox8);
        textLabel1_2->setObjectName(QString::fromUtf8("textLabel1_2"));
        textLabel1_2->setGeometry(QRect(10, 20, 122, 20));
        textLabel1_2->setWordWrap(false);
        setContactsBox = new QCheckBox(groupBox8);
        setContactsBox->setObjectName(QString::fromUtf8("setContactsBox"));
        setContactsBox->setGeometry(QRect(140, 48, 16, 20));
        textLabel1_5 = new QLabel(groupBox8);
        textLabel1_5->setObjectName(QString::fromUtf8("textLabel1_5"));
        textLabel1_5->setGeometry(QRect(22, 48, 99, 20));
        textLabel1_5->setWordWrap(false);
        textLabel1_4 = new QLabel(groupBox8);
        textLabel1_4->setObjectName(QString::fromUtf8("textLabel1_4"));
        textLabel1_4->setGeometry(QRect(147, 77, 30, 20));
        textLabel1_4->setWordWrap(false);
        currentStepLabel = new QLabel(groupBox8);
        currentStepLabel->setObjectName(QString::fromUtf8("currentStepLabel"));
        currentStepLabel->setGeometry(QRect(13, 78, 120, 20));
        currentStepLabel->setWordWrap(false);
        timeLabel = new QLabel(groupBox8);
        timeLabel->setObjectName(QString::fromUtf8("timeLabel"));
        timeLabel->setGeometry(QRect(13, 99, 150, 20));
        timeLabel->setWordWrap(false);
        line1_2 = new QFrame(groupBox8);
        line1_2->setObjectName(QString::fromUtf8("line1_2"));
        line1_2->setGeometry(QRect(10, 113, 260, 20));
        line1_2->setFrameShape(QFrame::HLine);
        line1_2->setFrameShadow(QFrame::Sunken);
        line1_2->setFrameShape(QFrame::HLine);
        line1_2_2_2 = new QFrame(groupBox8);
        line1_2_2_2->setObjectName(QString::fromUtf8("line1_2_2_2"));
        line1_2_2_2->setGeometry(QRect(10, 62, 260, 20));
        line1_2_2_2->setFrameShape(QFrame::HLine);
        line1_2_2_2->setFrameShadow(QFrame::Sunken);
        line1_2_2_2->setFrameShape(QFrame::HLine);
        rankLabel = new QLabel(groupBox8);
        rankLabel->setObjectName(QString::fromUtf8("rankLabel"));
        rankLabel->setGeometry(QRect(142, 135, 120, 20));
        rankLabel->setWordWrap(false);
        energyLabel = new QLabel(groupBox8);
        energyLabel->setObjectName(QString::fromUtf8("energyLabel"));
        energyLabel->setGeometry(QRect(135, 152, 130, 20));
        energyLabel->setWordWrap(false);
        iterationLabel = new QLabel(groupBox8);
        iterationLabel->setObjectName(QString::fromUtf8("iterationLabel"));
        iterationLabel->setGeometry(QRect(131, 169, 140, 20));
        iterationLabel->setWordWrap(false);
        annStepsEdit = new QLineEdit(groupBox8);
        annStepsEdit->setObjectName(QString::fromUtf8("annStepsEdit"));
        annStepsEdit->setGeometry(QRect(180, 76, 60, 21));
        onlineDetailsGroup = new Q3GroupBox(EigenGraspPlannerDlgUI);
        onlineDetailsGroup->setObjectName(QString::fromUtf8("onlineDetailsGroup"));
        onlineDetailsGroup->setGeometry(QRect(270, 320, 280, 220));
        onlineDetailsGroup->setOrientation(Qt::Vertical);
        objDistLabel = new QLabel(onlineDetailsGroup);
        objDistLabel->setObjectName(QString::fromUtf8("objDistLabel"));
        objDistLabel->setGeometry(QRect(10, 20, 120, 21));
        objDistLabel->setWordWrap(false);
        solDistLabel = new QLabel(onlineDetailsGroup);
        solDistLabel->setObjectName(QString::fromUtf8("solDistLabel"));
        solDistLabel->setGeometry(QRect(10, 40, 120, 21));
        solDistLabel->setWordWrap(false);
        onlineStatusLabel = new QLabel(onlineDetailsGroup);
        onlineStatusLabel->setObjectName(QString::fromUtf8("onlineStatusLabel"));
        onlineStatusLabel->setGeometry(QRect(10, 60, 100, 20));
        onlineStatusLabel->setWordWrap(false);
        fcBufferLabel = new QLabel(onlineDetailsGroup);
        fcBufferLabel->setObjectName(QString::fromUtf8("fcBufferLabel"));
        fcBufferLabel->setGeometry(QRect(10, 98, 120, 20));
        fcBufferLabel->setWordWrap(false);
        saBufferLabel = new QLabel(onlineDetailsGroup);
        saBufferLabel->setObjectName(QString::fromUtf8("saBufferLabel"));
        saBufferLabel->setGeometry(QRect(10, 79, 100, 20));
        saBufferLabel->setWordWrap(false);
        onlineGraspButton = new QPushButton(onlineDetailsGroup);
        onlineGraspButton->setObjectName(QString::fromUtf8("onlineGraspButton"));
        onlineGraspButton->setGeometry(QRect(211, 10, 40, 30));
        autoGraspBox = new QCheckBox(onlineDetailsGroup);
        autoGraspBox->setObjectName(QString::fromUtf8("autoGraspBox"));
        autoGraspBox->setGeometry(QRect(208, 119, 50, 21));
        onlineReleaseButton = new QPushButton(onlineDetailsGroup);
        onlineReleaseButton->setObjectName(QString::fromUtf8("onlineReleaseButton"));
        onlineReleaseButton->setGeometry(QRect(210, 43, 40, 30));
        onlinePlanButton = new QPushButton(onlineDetailsGroup);
        onlinePlanButton->setObjectName(QString::fromUtf8("onlinePlanButton"));
        onlinePlanButton->setGeometry(QRect(210, 76, 40, 30));
        line3 = new QFrame(onlineDetailsGroup);
        line3->setObjectName(QString::fromUtf8("line3"));
        line3->setGeometry(QRect(180, 14, 20, 190));
        line3->setFrameShape(QFrame::VLine);
        line3->setFrameShadow(QFrame::Sunken);
        line3->setFrameShape(QFrame::VLine);
        showCloneBox = new QCheckBox(onlineDetailsGroup);
        showCloneBox->setObjectName(QString::fromUtf8("showCloneBox"));
        showCloneBox->setGeometry(QRect(10, 120, 170, 21));
        showSolutionBox = new QCheckBox(onlineDetailsGroup);
        showSolutionBox->setObjectName(QString::fromUtf8("showSolutionBox"));
        showSolutionBox->setGeometry(QRect(10, 140, 160, 21));
        line4 = new QFrame(onlineDetailsGroup);
        line4->setObjectName(QString::fromUtf8("line4"));
        line4->setGeometry(QRect(10, 155, 160, 20));
        line4->setFrameShape(QFrame::HLine);
        line4->setFrameShadow(QFrame::Sunken);
        line4->setFrameShape(QFrame::HLine);
        useVirtualHandBox = new QCheckBox(onlineDetailsGroup);
        useVirtualHandBox->setObjectName(QString::fromUtf8("useVirtualHandBox"));
        useVirtualHandBox->setGeometry(QRect(10, 170, 140, 21));
        useRealBarrettBox = new QCheckBox(onlineDetailsGroup);
        useRealBarrettBox->setObjectName(QString::fromUtf8("useRealBarrettBox"));
        useRealBarrettBox->setGeometry(QRect(10, 190, 170, 21));
        inputBox = new Q3GroupBox(EigenGraspPlannerDlgUI);
        inputBox->setObjectName(QString::fromUtf8("inputBox"));
        inputBox->setGeometry(QRect(10, 420, 250, 80));
        inputBox->setOrientation(Qt::Vertical);
        inputLoadButton = new QPushButton(inputBox);
        inputLoadButton->setObjectName(QString::fromUtf8("inputLoadButton"));
        inputLoadButton->setGeometry(QRect(10, 50, 100, 20));
        inputGloveBox = new QCheckBox(inputBox);
        inputGloveBox->setObjectName(QString::fromUtf8("inputGloveBox"));
        inputGloveBox->setGeometry(QRect(10, 20, 121, 21));
        variableBox = new Q3GroupBox(EigenGraspPlannerDlgUI);
        variableBox->setObjectName(QString::fromUtf8("variableBox"));
        variableBox->setGeometry(QRect(10, 10, 250, 400));
        variableBox->setOrientation(Qt::Vertical);
        spaceSearchBox = new QComboBox(variableBox);
        spaceSearchBox->setObjectName(QString::fromUtf8("spaceSearchBox"));
        spaceSearchBox->setGeometry(QRect(60, 20, 180, 21));
        spaceSearchLabel = new QLabel(variableBox);
        spaceSearchLabel->setObjectName(QString::fromUtf8("spaceSearchLabel"));
        spaceSearchLabel->setGeometry(QRect(10, 20, 35, 20));
        spaceSearchLabel->setWordWrap(false);

        retranslateUi(EigenGraspPlannerDlgUI);
        QObject::connect(exitButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(exitButton_clicked()));
        QObject::connect(prevGraspButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(prevGraspButton_clicked()));
        QObject::connect(bestGraspButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(bestGraspButton_clicked()));
        QObject::connect(nextGraspButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(nextGraspButton_clicked()));
        QObject::connect(setContactsBox, SIGNAL(toggled(bool)), EigenGraspPlannerDlgUI, SLOT(setContactsBox_toggled(bool)));
        QObject::connect(spaceSearchBox, SIGNAL(activated(QString)), EigenGraspPlannerDlgUI, SLOT(spaceSearchBox_activated(QString)));
        QObject::connect(energyBox, SIGNAL(activated(QString)), EigenGraspPlannerDlgUI, SLOT(energyBox_activated(QString)));
        QObject::connect(plannerTypeBox, SIGNAL(activated(QString)), EigenGraspPlannerDlgUI, SLOT(plannerTypeBox_activated(QString)));
        QObject::connect(plannerResetButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(plannerReset_clicked()));
        QObject::connect(plannerInitButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(plannerInit_clicked()));
        QObject::connect(plannerStartButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(plannerStart_clicked()));
        QObject::connect(onlineGraspButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(onlineGraspButton_clicked()));
        QObject::connect(autoGraspBox, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(autoGraspBox_clicked()));
        QObject::connect(onlineReleaseButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(onlineReleaseButton_clicked()));
        QObject::connect(onlinePlanButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(onlinePlanButton_clicked()));
        QObject::connect(instantEnergyButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(instantEnergyButton_clicked()));
        QObject::connect(showCloneBox, SIGNAL(toggled(bool)), EigenGraspPlannerDlgUI, SLOT(showCloneBox_toggled(bool)));
        QObject::connect(showSolutionBox, SIGNAL(toggled(bool)), EigenGraspPlannerDlgUI, SLOT(showSolutionBox_toggled(bool)));
        QObject::connect(useRealBarrettBox, SIGNAL(toggled(bool)), EigenGraspPlannerDlgUI, SLOT(useRealBarrettBox_toggled(bool)));
        QObject::connect(inputGloveBox, SIGNAL(toggled(bool)), EigenGraspPlannerDlgUI, SLOT(inputGloveBox_toggled(bool)));
        QObject::connect(inputLoadButton, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(inputLoadButton_clicked()));
        QObject::connect(useVirtualHandBox, SIGNAL(clicked()), EigenGraspPlannerDlgUI, SLOT(useVirtualHandBox_clicked()));

        QMetaObject::connectSlotsByName(EigenGraspPlannerDlgUI);
    } // setupUi

    void retranslateUi(QDialog *EigenGraspPlannerDlgUI)
    {
        EigenGraspPlannerDlgUI->setWindowTitle(QApplication::translate("EigenGraspPlannerDlgUI", "EigenGrasp Planners", 0, QApplication::UnicodeUTF8));
        exitButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Exit", 0, QApplication::UnicodeUTF8));
        groupBox3->setTitle(QApplication::translate("EigenGraspPlannerDlgUI", "Planner", 0, QApplication::UnicodeUTF8));
        textLabel1->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Type:", 0, QApplication::UnicodeUTF8));
        plannerResetButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Reset", 0, QApplication::UnicodeUTF8));
        plannerInitButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Init", 0, QApplication::UnicodeUTF8));
        plannerStartButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", ">", 0, QApplication::UnicodeUTF8));
        instantEnergyButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "e", 0, QApplication::UnicodeUTF8));
        groupBox8->setTitle(QApplication::translate("EigenGraspPlannerDlgUI", "Settings and results", 0, QApplication::UnicodeUTF8));
        nextGraspButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", ">", 0, QApplication::UnicodeUTF8));
        bestGraspButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Best", 0, QApplication::UnicodeUTF8));
        prevGraspButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "<", 0, QApplication::UnicodeUTF8));
        textLabel2_3->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Show results:", 0, QApplication::UnicodeUTF8));
        textLabel1_2->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Energy formulation:", 0, QApplication::UnicodeUTF8));
        setContactsBox->setText(QString());
        textLabel1_5->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Preset contacts:", 0, QApplication::UnicodeUTF8));
        textLabel1_4->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Max:", 0, QApplication::UnicodeUTF8));
        currentStepLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Current step: 0", 0, QApplication::UnicodeUTF8));
        timeLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Time used: 0 sec.", 0, QApplication::UnicodeUTF8));
        rankLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Rank: 0/0", 0, QApplication::UnicodeUTF8));
        energyLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Energy: 0", 0, QApplication::UnicodeUTF8));
        iterationLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Iteration: 0", 0, QApplication::UnicodeUTF8));
        onlineDetailsGroup->setTitle(QApplication::translate("EigenGraspPlannerDlgUI", "OnLine Planning Details", 0, QApplication::UnicodeUTF8));
        objDistLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Object distance:", 0, QApplication::UnicodeUTF8));
        solDistLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Solution distance:", 0, QApplication::UnicodeUTF8));
        onlineStatusLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Status:", 0, QApplication::UnicodeUTF8));
        fcBufferLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "FC Thread buffer:", 0, QApplication::UnicodeUTF8));
        saBufferLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "SimAnn buffer:", 0, QApplication::UnicodeUTF8));
        onlineGraspButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Grasp", 0, QApplication::UnicodeUTF8));
        autoGraspBox->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Auto", 0, QApplication::UnicodeUTF8));
        onlineReleaseButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Open", 0, QApplication::UnicodeUTF8));
        onlinePlanButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Plan", 0, QApplication::UnicodeUTF8));
        showCloneBox->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Show pre-grasp search", 0, QApplication::UnicodeUTF8));
        showSolutionBox->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Show current solution", 0, QApplication::UnicodeUTF8));
        useVirtualHandBox->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Shape virtual hand model", 0, QApplication::UnicodeUTF8));
        useRealBarrettBox->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Shape real Barrett Hand", 0, QApplication::UnicodeUTF8));
        inputBox->setTitle(QApplication::translate("EigenGraspPlannerDlgUI", "Input", 0, QApplication::UnicodeUTF8));
        inputLoadButton->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Load from File...", 0, QApplication::UnicodeUTF8));
        inputGloveBox->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Use CyberGlove", 0, QApplication::UnicodeUTF8));
        variableBox->setTitle(QApplication::translate("EigenGraspPlannerDlgUI", "Space Search", 0, QApplication::UnicodeUTF8));
        spaceSearchLabel->setText(QApplication::translate("EigenGraspPlannerDlgUI", "Type:", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class EigenGraspPlannerDlgUI: public Ui_EigenGraspPlannerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EGPLANNERDLG_H
