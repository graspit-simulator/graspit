/********************************************************************************
** Form generated from reading ui file 'StrategyPlannerDlg.ui'
**
** Created: Mon Jun 25 14:40:15 2012
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_STRATEGYPLANNERDLG_H
#define UI_STRATEGYPLANNERDLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_StrategyPlannerDlgUI
{
public:
    QDialogButtonBox *buttonBox;
    QPushButton *planButton;

    void setupUi(QDialog *StrategyPlannerDlgUI)
    {
        if (StrategyPlannerDlgUI->objectName().isEmpty())
            StrategyPlannerDlgUI->setObjectName(QString::fromUtf8("StrategyPlannerDlgUI"));
        StrategyPlannerDlgUI->resize(400, 300);
        buttonBox = new QDialogButtonBox(StrategyPlannerDlgUI);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(30, 240, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        planButton = new QPushButton(StrategyPlannerDlgUI);
        planButton->setObjectName(QString::fromUtf8("planButton"));
        planButton->setGeometry(QRect(70, 90, 75, 23));

        retranslateUi(StrategyPlannerDlgUI);
        QObject::connect(buttonBox, SIGNAL(accepted()), StrategyPlannerDlgUI, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), StrategyPlannerDlgUI, SLOT(reject()));

        QMetaObject::connectSlotsByName(StrategyPlannerDlgUI);
    } // setupUi

    void retranslateUi(QDialog *StrategyPlannerDlgUI)
    {
        StrategyPlannerDlgUI->setWindowTitle(QApplication::translate("StrategyPlannerDlgUI", "Dialog", 0, QApplication::UnicodeUTF8));
        planButton->setText(QApplication::translate("StrategyPlannerDlgUI", "Plan", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(StrategyPlannerDlgUI);
    } // retranslateUi

};

namespace Ui {
    class StrategyPlannerDlgUI: public Ui_StrategyPlannerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STRATEGYPLANNERDLG_H
