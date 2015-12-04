/********************************************************************************
** Form generated from reading UI file 'qualityIndicator.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QUALITYINDICATOR_H
#define UI_QUALITYINDICATOR_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QProgressBar>

QT_BEGIN_NAMESPACE

class Ui_QualityIndicatorUI
{
public:
    QProgressBar *qualityBar;
    QLabel *label;
    QLabel *label_2;
    QLabel *fcLabel;

    void setupUi(QDialog *QualityIndicatorUI)
    {
        if (QualityIndicatorUI->objectName().isEmpty())
            QualityIndicatorUI->setObjectName(QString::fromUtf8("QualityIndicatorUI"));
        QualityIndicatorUI->resize(247, 39);
        qualityBar = new QProgressBar(QualityIndicatorUI);
        qualityBar->setObjectName(QString::fromUtf8("qualityBar"));
        qualityBar->setGeometry(QRect(69, 10, 171, 21));
        qualityBar->setValue(0);
        qualityBar->setOrientation(Qt::Horizontal);
        label = new QLabel(QualityIndicatorUI);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(41, 12, 21, 16));
        label_2 = new QLabel(QualityIndicatorUI);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(210, 12, 21, 16));
        fcLabel = new QLabel(QualityIndicatorUI);
        fcLabel->setObjectName(QString::fromUtf8("fcLabel"));
        fcLabel->setGeometry(QRect(10, 13, 21, 16));

        retranslateUi(QualityIndicatorUI);

        QMetaObject::connectSlotsByName(QualityIndicatorUI);
    } // setupUi

    void retranslateUi(QDialog *QualityIndicatorUI)
    {
        QualityIndicatorUI->setWindowTitle(QApplication::translate("QualityIndicatorUI", "Quality indicator", 0, QApplication::UnicodeUTF8));
        qualityBar->setFormat(QString());
        label->setText(QApplication::translate("QualityIndicatorUI", "0.00", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("QualityIndicatorUI", "0.25", 0, QApplication::UnicodeUTF8));
        fcLabel->setText(QApplication::translate("QualityIndicatorUI", "F/C", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class QualityIndicatorUI: public Ui_QualityIndicatorUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QUALITYINDICATOR_H
