/********************************************************************************
** Form generated from reading UI file 'gfoDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GFODLG_H
#define UI_GFODLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_GFODlgUI
{
public:
    QPushButton *exitButton;
    QGroupBox *groupBox;
    QLabel *statusLabel;
    QLabel *label_2;
    QLabel *label;
    QComboBox *optimizationTypeBox;
    QCheckBox *optimizationOnBox;

    void setupUi(QDialog *GFODlgUI)
    {
        if (GFODlgUI->objectName().isEmpty())
            GFODlgUI->setObjectName(QString::fromUtf8("GFODlgUI"));
        GFODlgUI->resize(241, 139);
        exitButton = new QPushButton(GFODlgUI);
        exitButton->setObjectName(QString::fromUtf8("exitButton"));
        exitButton->setGeometry(QRect(180, 110, 51, 23));
        groupBox = new QGroupBox(GFODlgUI);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 10, 221, 91));
        statusLabel = new QLabel(groupBox);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));
        statusLabel->setGeometry(QRect(10, 60, 191, 16));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(190, 10, 21, 16));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 101, 16));
        optimizationTypeBox = new QComboBox(groupBox);
        optimizationTypeBox->setObjectName(QString::fromUtf8("optimizationTypeBox"));
        optimizationTypeBox->setGeometry(QRect(10, 30, 171, 22));
        optimizationOnBox = new QCheckBox(groupBox);
        optimizationOnBox->setObjectName(QString::fromUtf8("optimizationOnBox"));
        optimizationOnBox->setGeometry(QRect(190, 30, 21, 18));

        retranslateUi(GFODlgUI);

        QMetaObject::connectSlotsByName(GFODlgUI);
    } // setupUi

    void retranslateUi(QDialog *GFODlgUI)
    {
        GFODlgUI->setWindowTitle(QApplication::translate("GFODlgUI", "Grasp Force Optimization", 0, QApplication::UnicodeUTF8));
        exitButton->setText(QApplication::translate("GFODlgUI", "Exit", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QString());
        statusLabel->setText(QApplication::translate("GFODlgUI", "Status:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("GFODlgUI", "On:", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("GFODlgUI", "Optimization type:", 0, QApplication::UnicodeUTF8));
        optimizationOnBox->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class GFODlgUI: public Ui_GFODlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GFODLG_H
