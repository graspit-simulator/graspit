/********************************************************************************
** Form generated from reading UI file 'optimizerDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPTIMIZERDLG_H
#define UI_OPTIMIZERDLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_OptimizerDlgUI
{
public:
    QPushButton *exitButton;
    QPushButton *torqueButton;

    void setupUi(QDialog *OptimizerDlgUI)
    {
        if (OptimizerDlgUI->objectName().isEmpty())
            OptimizerDlgUI->setObjectName(QString::fromUtf8("OptimizerDlgUI"));
        OptimizerDlgUI->resize(183, 92);
        exitButton = new QPushButton(OptimizerDlgUI);
        exitButton->setObjectName(QString::fromUtf8("exitButton"));
        exitButton->setGeometry(QRect(130, 60, 41, 23));
        torqueButton = new QPushButton(OptimizerDlgUI);
        torqueButton->setObjectName(QString::fromUtf8("torqueButton"));
        torqueButton->setGeometry(QRect(10, 10, 81, 23));

        retranslateUi(OptimizerDlgUI);

        QMetaObject::connectSlotsByName(OptimizerDlgUI);
    } // setupUi

    void retranslateUi(QDialog *OptimizerDlgUI)
    {
        OptimizerDlgUI->setWindowTitle(QApplication::translate("OptimizerDlgUI", "Optimizer", 0, QApplication::UnicodeUTF8));
        exitButton->setText(QApplication::translate("OptimizerDlgUI", "Exit", 0, QApplication::UnicodeUTF8));
        torqueButton->setText(QApplication::translate("OptimizerDlgUI", "CGDB Torques", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class OptimizerDlgUI: public Ui_OptimizerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPTIMIZERDLG_H
