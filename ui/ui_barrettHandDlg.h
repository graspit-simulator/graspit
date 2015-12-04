/********************************************************************************
** Form generated from reading UI file 'barrettHandDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BARRETTHANDDLG_H
#define UI_BARRETTHANDDLG_H

#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_BarrettHandDlgUI
{
public:
    QPushButton *pushButton17;
    QLabel *textLabel1;
    QLineEdit *stepSize;
    QPushButton *continuousOperationButton;
    QPushButton *pushButton18;
    QPushButton *pushButton19;
    QPushButton *smoothButton;
    QPushButton *initSpreadButton;

    void setupUi(QDialog *BarrettHandDlgUI)
    {
        if (BarrettHandDlgUI->objectName().isEmpty())
            BarrettHandDlgUI->setObjectName(QString::fromUtf8("BarrettHandDlgUI"));
        BarrettHandDlgUI->resize(330, 157);
        pushButton17 = new QPushButton(BarrettHandDlgUI);
        pushButton17->setObjectName(QString::fromUtf8("pushButton17"));
        pushButton17->setGeometry(QRect(10, 80, 100, 31));
        textLabel1 = new QLabel(BarrettHandDlgUI);
        textLabel1->setObjectName(QString::fromUtf8("textLabel1"));
        textLabel1->setGeometry(QRect(10, 50, 101, 20));
        textLabel1->setWordWrap(false);
        stepSize = new QLineEdit(BarrettHandDlgUI);
        stepSize->setObjectName(QString::fromUtf8("stepSize"));
        stepSize->setGeometry(QRect(120, 50, 26, 20));
        continuousOperationButton = new QPushButton(BarrettHandDlgUI);
        continuousOperationButton->setObjectName(QString::fromUtf8("continuousOperationButton"));
        continuousOperationButton->setGeometry(QRect(10, 120, 151, 31));
        pushButton18 = new QPushButton(BarrettHandDlgUI);
        pushButton18->setObjectName(QString::fromUtf8("pushButton18"));
        pushButton18->setGeometry(QRect(120, 80, 100, 31));
        pushButton19 = new QPushButton(BarrettHandDlgUI);
        pushButton19->setObjectName(QString::fromUtf8("pushButton19"));
        pushButton19->setGeometry(QRect(10, 10, 82, 26));
        smoothButton = new QPushButton(BarrettHandDlgUI);
        smoothButton->setObjectName(QString::fromUtf8("smoothButton"));
        smoothButton->setGeometry(QRect(100, 10, 101, 30));
        initSpreadButton = new QPushButton(BarrettHandDlgUI);
        initSpreadButton->setObjectName(QString::fromUtf8("initSpreadButton"));
        initSpreadButton->setGeometry(QRect(210, 10, 111, 31));

        retranslateUi(BarrettHandDlgUI);
        QObject::connect(pushButton18, SIGNAL(clicked()), BarrettHandDlgUI, SLOT(realHandFromSimulation()));
        QObject::connect(continuousOperationButton, SIGNAL(clicked()), BarrettHandDlgUI, SLOT(toggleContinuousOperation()));
        QObject::connect(pushButton17, SIGNAL(clicked()), BarrettHandDlgUI, SLOT(simulationFromRealHand()));
        QObject::connect(pushButton19, SIGNAL(clicked()), BarrettHandDlgUI, SLOT(initializeHand()));
        QObject::connect(smoothButton, SIGNAL(clicked()), BarrettHandDlgUI, SLOT(smoothButton_clicked()));
        QObject::connect(initSpreadButton, SIGNAL(clicked()), BarrettHandDlgUI, SLOT(initSpreadButton_clicked()));

        QMetaObject::connectSlotsByName(BarrettHandDlgUI);
    } // setupUi

    void retranslateUi(QDialog *BarrettHandDlgUI)
    {
        BarrettHandDlgUI->setWindowTitle(QApplication::translate("BarrettHandDlgUI", "Barrett Hand", 0, QApplication::UnicodeUTF8));
        pushButton17->setText(QApplication::translate("BarrettHandDlgUI", "Update simulation", 0, QApplication::UnicodeUTF8));
        textLabel1->setText(QApplication::translate("BarrettHandDlgUI", "Step size (in degrees)", 0, QApplication::UnicodeUTF8));
        stepSize->setText(QApplication::translate("BarrettHandDlgUI", "15", 0, QApplication::UnicodeUTF8));
        continuousOperationButton->setText(QApplication::translate("BarrettHandDlgUI", "Begin continuous operation", 0, QApplication::UnicodeUTF8));
        pushButton18->setText(QApplication::translate("BarrettHandDlgUI", "Update real hand", 0, QApplication::UnicodeUTF8));
        pushButton19->setText(QApplication::translate("BarrettHandDlgUI", "Initialize hand", 0, QApplication::UnicodeUTF8));
        smoothButton->setText(QApplication::translate("BarrettHandDlgUI", "Smooth Spread", 0, QApplication::UnicodeUTF8));
        initSpreadButton->setText(QApplication::translate("BarrettHandDlgUI", "pushButton6", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class BarrettHandDlgUI: public Ui_BarrettHandDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BARRETTHANDDLG_H
