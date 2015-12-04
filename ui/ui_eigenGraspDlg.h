/********************************************************************************
** Form generated from reading UI file 'eigenGraspDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EIGENGRASPDLG_H
#define UI_EIGENGRASPDLG_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <vector>

QT_BEGIN_NAMESPACE

class Ui_EigenGraspDlgUI
{
public:
    QPushButton *exitButton;
    Q3GroupBox *groupBox3;
    QPushButton *closeHandButton;
    QPushButton *setOriginButton;
    QPushButton *goToOriginButton;
    QFrame *line1;
    QCheckBox *rigidCheckBox;
    QPushButton *loadButton;
    QLabel *fileNameLabel;
    QPushButton *saveButton;
    QPushButton *identityButton;

    void setupUi(QDialog *EigenGraspDlgUI)
    {
        if (EigenGraspDlgUI->objectName().isEmpty())
            EigenGraspDlgUI->setObjectName(QString::fromUtf8("EigenGraspDlgUI"));
        EigenGraspDlgUI->resize(278, 290);
        exitButton = new QPushButton(EigenGraspDlgUI);
        exitButton->setObjectName(QString::fromUtf8("exitButton"));
        exitButton->setGeometry(QRect(210, 250, 60, 31));
        groupBox3 = new Q3GroupBox(EigenGraspDlgUI);
        groupBox3->setObjectName(QString::fromUtf8("groupBox3"));
        groupBox3->setGeometry(QRect(10, 10, 261, 230));
        groupBox3->setOrientation(Qt::Vertical);
        closeHandButton = new QPushButton(groupBox3);
        closeHandButton->setObjectName(QString::fromUtf8("closeHandButton"));
        closeHandButton->setGeometry(QRect(20, 90, 100, 31));
        setOriginButton = new QPushButton(groupBox3);
        setOriginButton->setObjectName(QString::fromUtf8("setOriginButton"));
        setOriginButton->setGeometry(QRect(20, 50, 100, 31));
        goToOriginButton = new QPushButton(groupBox3);
        goToOriginButton->setObjectName(QString::fromUtf8("goToOriginButton"));
        goToOriginButton->setGeometry(QRect(130, 50, 101, 31));
        line1 = new QFrame(groupBox3);
        line1->setObjectName(QString::fromUtf8("line1"));
        line1->setGeometry(QRect(10, 130, 241, 20));
        line1->setFrameShape(QFrame::HLine);
        line1->setFrameShadow(QFrame::Sunken);
        line1->setFrameShape(QFrame::HLine);
        rigidCheckBox = new QCheckBox(groupBox3);
        rigidCheckBox->setObjectName(QString::fromUtf8("rigidCheckBox"));
        rigidCheckBox->setGeometry(QRect(20, 20, 50, 31));
        loadButton = new QPushButton(groupBox3);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));
        loadButton->setGeometry(QRect(20, 150, 50, 31));
        fileNameLabel = new QLabel(groupBox3);
        fileNameLabel->setObjectName(QString::fromUtf8("fileNameLabel"));
        fileNameLabel->setGeometry(QRect(20, 190, 230, 20));
        fileNameLabel->setWordWrap(false);
        saveButton = new QPushButton(groupBox3);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(80, 150, 50, 31));
        identityButton = new QPushButton(groupBox3);
        identityButton->setObjectName(QString::fromUtf8("identityButton"));
        identityButton->setGeometry(QRect(140, 150, 91, 31));

        retranslateUi(EigenGraspDlgUI);
        QObject::connect(saveButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(saveButton_clicked()));
        QObject::connect(exitButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(exitButton_clicked()));
        QObject::connect(setOriginButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(setOriginButton_clicked()));
        QObject::connect(rigidCheckBox, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(rigidCheckBox_clicked()));
        QObject::connect(closeHandButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(closeHandButton_clicked()));
        QObject::connect(goToOriginButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(goToOriginButton_clicked()));
        QObject::connect(loadButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(loadButton_clicked()));
        QObject::connect(identityButton, SIGNAL(clicked()), EigenGraspDlgUI, SLOT(identityButton_clicked()));

        QMetaObject::connectSlotsByName(EigenGraspDlgUI);
    } // setupUi

    void retranslateUi(QDialog *EigenGraspDlgUI)
    {
        EigenGraspDlgUI->setWindowTitle(QApplication::translate("EigenGraspDlgUI", "EigenGrasps", 0, QApplication::UnicodeUTF8));
        exitButton->setText(QApplication::translate("EigenGraspDlgUI", "Exit", 0, QApplication::UnicodeUTF8));
        groupBox3->setTitle(QApplication::translate("EigenGraspDlgUI", "Properties and control", 0, QApplication::UnicodeUTF8));
        closeHandButton->setText(QApplication::translate("EigenGraspDlgUI", "Close to Contacts", 0, QApplication::UnicodeUTF8));
        setOriginButton->setText(QApplication::translate("EigenGraspDlgUI", "Set Origin", 0, QApplication::UnicodeUTF8));
        goToOriginButton->setText(QApplication::translate("EigenGraspDlgUI", "Go To Origin", 0, QApplication::UnicodeUTF8));
        rigidCheckBox->setText(QApplication::translate("EigenGraspDlgUI", "Rigid", 0, QApplication::UnicodeUTF8));
        loadButton->setText(QApplication::translate("EigenGraspDlgUI", "Load...", 0, QApplication::UnicodeUTF8));
        fileNameLabel->setText(QApplication::translate("EigenGraspDlgUI", "Filename:", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("EigenGraspDlgUI", "Save...", 0, QApplication::UnicodeUTF8));
        identityButton->setText(QApplication::translate("EigenGraspDlgUI", "Use Identity", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class EigenGraspDlgUI: public Ui_EigenGraspDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EIGENGRASPDLG_H
