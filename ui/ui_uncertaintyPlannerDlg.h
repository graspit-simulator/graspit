/********************************************************************************
** Form generated from reading ui file 'uncertaintyPlannerDlg.ui'
**
** Created: Thu Jul 28 04:10:03 2011
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_UNCERTAINTYPLANNERDLG_H
#define UI_UNCERTAINTYPLANNERDLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_UncertaintyPlannerDlgUI
{
public:
    QPushButton *sampleButton;
    QPushButton *loadButton;
    QPushButton *visualizeButton;
    QPushButton *loadUncertaintyButton;
    QSlider *percentageSlider;
    QLabel *percentageLabel;
    QSlider *intervalSlider;
    QLabel *intervalLabel;
    QPushButton *marchingCubeButton;

    void setupUi(QWidget *UncertaintyPlannerDlgUI)
    {
        if (UncertaintyPlannerDlgUI->objectName().isEmpty())
            UncertaintyPlannerDlgUI->setObjectName(QString::fromUtf8("UncertaintyPlannerDlgUI"));
        UncertaintyPlannerDlgUI->resize(400, 300);
        sampleButton = new QPushButton(UncertaintyPlannerDlgUI);
        sampleButton->setObjectName(QString::fromUtf8("sampleButton"));
        sampleButton->setGeometry(QRect(20, 200, 93, 27));
        loadButton = new QPushButton(UncertaintyPlannerDlgUI);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));
        loadButton->setGeometry(QRect(20, 170, 93, 27));
        visualizeButton = new QPushButton(UncertaintyPlannerDlgUI);
        visualizeButton->setObjectName(QString::fromUtf8("visualizeButton"));
        visualizeButton->setGeometry(QRect(20, 230, 93, 27));
        loadUncertaintyButton = new QPushButton(UncertaintyPlannerDlgUI);
        loadUncertaintyButton->setObjectName(QString::fromUtf8("loadUncertaintyButton"));
        loadUncertaintyButton->setGeometry(QRect(20, 140, 93, 27));
        percentageSlider = new QSlider(UncertaintyPlannerDlgUI);
        percentageSlider->setObjectName(QString::fromUtf8("percentageSlider"));
        percentageSlider->setGeometry(QRect(140, 240, 160, 18));
        percentageSlider->setOrientation(Qt::Horizontal);
        percentageLabel = new QLabel(UncertaintyPlannerDlgUI);
        percentageLabel->setObjectName(QString::fromUtf8("percentageLabel"));
        percentageLabel->setGeometry(QRect(320, 240, 62, 17));
        intervalSlider = new QSlider(UncertaintyPlannerDlgUI);
        intervalSlider->setObjectName(QString::fromUtf8("intervalSlider"));
        intervalSlider->setGeometry(QRect(140, 210, 160, 18));
        intervalSlider->setMinimum(1);
        intervalSlider->setMaximum(20);
        intervalSlider->setOrientation(Qt::Horizontal);
        intervalLabel = new QLabel(UncertaintyPlannerDlgUI);
        intervalLabel->setObjectName(QString::fromUtf8("intervalLabel"));
        intervalLabel->setGeometry(QRect(320, 200, 62, 17));
        marchingCubeButton = new QPushButton(UncertaintyPlannerDlgUI);
        marchingCubeButton->setObjectName(QString::fromUtf8("marchingCubeButton"));
        marchingCubeButton->setGeometry(QRect(20, 261, 93, 27));

        retranslateUi(UncertaintyPlannerDlgUI);

        QMetaObject::connectSlotsByName(UncertaintyPlannerDlgUI);
    } // setupUi

    void retranslateUi(QWidget *UncertaintyPlannerDlgUI)
    {
        UncertaintyPlannerDlgUI->setWindowTitle(QApplication::translate("UncertaintyPlannerDlgUI", "Form", 0, QApplication::UnicodeUTF8));
        sampleButton->setText(QApplication::translate("UncertaintyPlannerDlgUI", "sample", 0, QApplication::UnicodeUTF8));
        loadButton->setText(QApplication::translate("UncertaintyPlannerDlgUI", "object", 0, QApplication::UnicodeUTF8));
        visualizeButton->setText(QApplication::translate("UncertaintyPlannerDlgUI", "visualize", 0, QApplication::UnicodeUTF8));
        loadUncertaintyButton->setText(QApplication::translate("UncertaintyPlannerDlgUI", "uncertainty", 0, QApplication::UnicodeUTF8));
        percentageLabel->setText(QApplication::translate("UncertaintyPlannerDlgUI", "n/a", 0, QApplication::UnicodeUTF8));
        intervalLabel->setText(QApplication::translate("UncertaintyPlannerDlgUI", "n/a", 0, QApplication::UnicodeUTF8));
        marchingCubeButton->setText(QApplication::translate("UncertaintyPlannerDlgUI", "Mesh it", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(UncertaintyPlannerDlgUI);
    } // retranslateUi

};

namespace Ui {
    class UncertaintyPlannerDlgUI: public Ui_UncertaintyPlannerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_UNCERTAINTYPLANNERDLG_H
