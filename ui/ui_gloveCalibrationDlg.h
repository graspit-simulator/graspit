/********************************************************************************
** Form generated from reading UI file 'gloveCalibrationDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GLOVECALIBRATIONDLG_H
#define UI_GLOVECALIBRATIONDLG_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_GloveCalibrationDlgUI
{
public:
    Q3GroupBox *groupBox5;
    QLabel *textLabel2;
    QPushButton *nextPoseButton;
    QPushButton *recordButton;
    QPushButton *prevPoseButton;
    QLabel *textLabel1_2;
    QLabel *textLabel2_2;
    QComboBox *poseDistanceBox;
    QFrame *line1_2;
    QPushButton *savePosesButton;
    QPushButton *loadPosesButton;
    QLabel *textLabel1_3;
    QLabel *numberPosesLabel;
    QPushButton *clearPosesButton;
    Q3GroupBox *groupBox3;
    QPushButton *saveButton;
    QPushButton *loadButton;
    QPushButton *doneButton;
    QComboBox *calibrationTypeBox;
    QPushButton *calibrateButton;
    QPushButton *initButton;
    QLabel *textLabel1;
    QFrame *line1;
    QPushButton *startGloveButton;

    void setupUi(QDialog *GloveCalibrationDlgUI)
    {
        if (GloveCalibrationDlgUI->objectName().isEmpty())
            GloveCalibrationDlgUI->setObjectName(QString::fromUtf8("GloveCalibrationDlgUI"));
        GloveCalibrationDlgUI->resize(246, 399);
        groupBox5 = new Q3GroupBox(GloveCalibrationDlgUI);
        groupBox5->setObjectName(QString::fromUtf8("groupBox5"));
        groupBox5->setGeometry(QRect(10, 10, 230, 200));
        textLabel2 = new QLabel(groupBox5);
        textLabel2->setObjectName(QString::fromUtf8("textLabel2"));
        textLabel2->setGeometry(QRect(60, 50, 110, 20));
        textLabel2->setWordWrap(false);
        nextPoseButton = new QPushButton(groupBox5);
        nextPoseButton->setObjectName(QString::fromUtf8("nextPoseButton"));
        nextPoseButton->setGeometry(QRect(150, 20, 50, 30));
        recordButton = new QPushButton(groupBox5);
        recordButton->setObjectName(QString::fromUtf8("recordButton"));
        recordButton->setGeometry(QRect(90, 20, 50, 30));
        prevPoseButton = new QPushButton(groupBox5);
        prevPoseButton->setObjectName(QString::fromUtf8("prevPoseButton"));
        prevPoseButton->setGeometry(QRect(30, 20, 50, 30));
        textLabel1_2 = new QLabel(groupBox5);
        textLabel1_2->setObjectName(QString::fromUtf8("textLabel1_2"));
        textLabel1_2->setGeometry(QRect(20, 80, 80, 20));
        textLabel1_2->setWordWrap(false);
        textLabel2_2 = new QLabel(groupBox5);
        textLabel2_2->setObjectName(QString::fromUtf8("textLabel2_2"));
        textLabel2_2->setGeometry(QRect(150, 80, 49, 20));
        textLabel2_2->setWordWrap(false);
        poseDistanceBox = new QComboBox(groupBox5);
        poseDistanceBox->setObjectName(QString::fromUtf8("poseDistanceBox"));
        poseDistanceBox->setGeometry(QRect(100, 80, 50, 21));
        line1_2 = new QFrame(groupBox5);
        line1_2->setObjectName(QString::fromUtf8("line1_2"));
        line1_2->setGeometry(QRect(10, 100, 211, 20));
        line1_2->setFrameShape(QFrame::HLine);
        line1_2->setFrameShadow(QFrame::Sunken);
        savePosesButton = new QPushButton(groupBox5);
        savePosesButton->setObjectName(QString::fromUtf8("savePosesButton"));
        savePosesButton->setGeometry(QRect(121, 120, 90, 30));
        loadPosesButton = new QPushButton(groupBox5);
        loadPosesButton->setObjectName(QString::fromUtf8("loadPosesButton"));
        loadPosesButton->setGeometry(QRect(20, 120, 90, 30));
        textLabel1_3 = new QLabel(groupBox5);
        textLabel1_3->setObjectName(QString::fromUtf8("textLabel1_3"));
        textLabel1_3->setGeometry(QRect(130, 170, 60, 20));
        textLabel1_3->setWordWrap(false);
        numberPosesLabel = new QLabel(groupBox5);
        numberPosesLabel->setObjectName(QString::fromUtf8("numberPosesLabel"));
        numberPosesLabel->setGeometry(QRect(190, 170, 30, 20));
        numberPosesLabel->setWordWrap(false);
        clearPosesButton = new QPushButton(groupBox5);
        clearPosesButton->setObjectName(QString::fromUtf8("clearPosesButton"));
        clearPosesButton->setGeometry(QRect(20, 160, 91, 30));
        groupBox3 = new Q3GroupBox(GloveCalibrationDlgUI);
        groupBox3->setObjectName(QString::fromUtf8("groupBox3"));
        groupBox3->setGeometry(QRect(10, 210, 230, 180));
        saveButton = new QPushButton(groupBox3);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(10, 140, 100, 30));
        loadButton = new QPushButton(groupBox3);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));
        loadButton->setGeometry(QRect(10, 100, 100, 30));
        doneButton = new QPushButton(groupBox3);
        doneButton->setObjectName(QString::fromUtf8("doneButton"));
        doneButton->setGeometry(QRect(150, 140, 71, 31));
        calibrationTypeBox = new QComboBox(groupBox3);
        calibrationTypeBox->setObjectName(QString::fromUtf8("calibrationTypeBox"));
        calibrationTypeBox->setGeometry(QRect(40, 20, 120, 20));
        calibrateButton = new QPushButton(groupBox3);
        calibrateButton->setObjectName(QString::fromUtf8("calibrateButton"));
        calibrateButton->setGeometry(QRect(100, 50, 80, 30));
        initButton = new QPushButton(groupBox3);
        initButton->setObjectName(QString::fromUtf8("initButton"));
        initButton->setGeometry(QRect(10, 50, 80, 30));
        textLabel1 = new QLabel(groupBox3);
        textLabel1->setObjectName(QString::fromUtf8("textLabel1"));
        textLabel1->setGeometry(QRect(10, 20, 30, 20));
        textLabel1->setWordWrap(false);
        line1 = new QFrame(groupBox3);
        line1->setObjectName(QString::fromUtf8("line1"));
        line1->setGeometry(QRect(10, 80, 211, 20));
        line1->setFrameShape(QFrame::HLine);
        line1->setFrameShadow(QFrame::Sunken);
        startGloveButton = new QPushButton(groupBox3);
        startGloveButton->setObjectName(QString::fromUtf8("startGloveButton"));
        startGloveButton->setGeometry(QRect(141, 100, 80, 31));

        retranslateUi(GloveCalibrationDlgUI);
        QObject::connect(prevPoseButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(prevPose()));
        QObject::connect(nextPoseButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(nextPose()));
        QObject::connect(recordButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(record()));
        QObject::connect(calibrateButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(calibrate()));
        QObject::connect(saveButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(save()));
        QObject::connect(doneButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(done()));
        QObject::connect(savePosesButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(savePoses()));
        QObject::connect(loadPosesButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(loadPoses()));
        QObject::connect(loadButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(loadCalibration()));
        QObject::connect(initButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(initCalibration()));
        QObject::connect(clearPosesButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(clearPoses()));
        QObject::connect(startGloveButton, SIGNAL(clicked()), GloveCalibrationDlgUI, SLOT(startGlove()));

        QMetaObject::connectSlotsByName(GloveCalibrationDlgUI);
    } // setupUi

    void retranslateUi(QDialog *GloveCalibrationDlgUI)
    {
        GloveCalibrationDlgUI->setWindowTitle(QApplication::translate("GloveCalibrationDlgUI", "Glove Calibration", 0, QApplication::UnicodeUTF8));
        groupBox5->setTitle(QApplication::translate("GloveCalibrationDlgUI", "Poses", 0, QApplication::UnicodeUTF8));
        textLabel2->setText(QApplication::translate("GloveCalibrationDlgUI", "No values recorded", 0, QApplication::UnicodeUTF8));
        nextPoseButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Next ->", 0, QApplication::UnicodeUTF8));
        recordButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Record", 0, QApplication::UnicodeUTF8));
        prevPoseButton->setText(QApplication::translate("GloveCalibrationDlgUI", "<- Prev.", 0, QApplication::UnicodeUTF8));
        textLabel1_2->setText(QApplication::translate("GloveCalibrationDlgUI", "Pose distance:", 0, QApplication::UnicodeUTF8));
        textLabel2_2->setText(QApplication::translate("GloveCalibrationDlgUI", "mm", 0, QApplication::UnicodeUTF8));
        savePosesButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Save Poses...", 0, QApplication::UnicodeUTF8));
        loadPosesButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Load Poses...", 0, QApplication::UnicodeUTF8));
        textLabel1_3->setText(QApplication::translate("GloveCalibrationDlgUI", "Total poses:", 0, QApplication::UnicodeUTF8));
        numberPosesLabel->setText(QApplication::translate("GloveCalibrationDlgUI", "0", 0, QApplication::UnicodeUTF8));
        clearPosesButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Clear Poses", 0, QApplication::UnicodeUTF8));
        groupBox3->setTitle(QApplication::translate("GloveCalibrationDlgUI", "Calibration", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Save Calibration...", 0, QApplication::UnicodeUTF8));
        loadButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Load Calibration...", 0, QApplication::UnicodeUTF8));
        doneButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Done", 0, QApplication::UnicodeUTF8));
        calibrateButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Calibrate", 0, QApplication::UnicodeUTF8));
        initButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Initialize", 0, QApplication::UnicodeUTF8));
        textLabel1->setText(QApplication::translate("GloveCalibrationDlgUI", "Type:", 0, QApplication::UnicodeUTF8));
        startGloveButton->setText(QApplication::translate("GloveCalibrationDlgUI", "Start Glove", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GloveCalibrationDlgUI: public Ui_GloveCalibrationDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GLOVECALIBRATIONDLG_H
