/********************************************************************************
** Form generated from reading UI file 'archBuilderDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ARCHBUILDERDLG_H
#define UI_ARCHBUILDERDLG_H

#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>

QT_BEGIN_NAMESPACE

class Ui_ArchBuilderDlgUI
{
public:
    QLabel *textLabel1;
    QLabel *textLabel4;
    QLabel *textLabel2;
    QLabel *textLabel3;
    QLineEdit *innerRadiusEdit;
    QLineEdit *outerRadiusEdit;
    QLineEdit *thicknessEdit;
    QSpinBox *numberBlocksBox;
    QCheckBox *supportsCheckBox;
    QPushButton *OKButton;
    QPushButton *cancelButton;

    void setupUi(QDialog *ArchBuilderDlgUI)
    {
        if (ArchBuilderDlgUI->objectName().isEmpty())
            ArchBuilderDlgUI->setObjectName(QString::fromUtf8("ArchBuilderDlgUI"));
        ArchBuilderDlgUI->resize(174, 210);
        textLabel1 = new QLabel(ArchBuilderDlgUI);
        textLabel1->setObjectName(QString::fromUtf8("textLabel1"));
        textLabel1->setGeometry(QRect(10, 10, 70, 20));
        textLabel1->setWordWrap(false);
        textLabel4 = new QLabel(ArchBuilderDlgUI);
        textLabel4->setObjectName(QString::fromUtf8("textLabel4"));
        textLabel4->setGeometry(QRect(10, 70, 49, 20));
        textLabel4->setWordWrap(false);
        textLabel2 = new QLabel(ArchBuilderDlgUI);
        textLabel2->setObjectName(QString::fromUtf8("textLabel2"));
        textLabel2->setGeometry(QRect(10, 40, 70, 20));
        textLabel2->setWordWrap(false);
        textLabel3 = new QLabel(ArchBuilderDlgUI);
        textLabel3->setObjectName(QString::fromUtf8("textLabel3"));
        textLabel3->setGeometry(QRect(10, 100, 83, 20));
        textLabel3->setWordWrap(false);
        innerRadiusEdit = new QLineEdit(ArchBuilderDlgUI);
        innerRadiusEdit->setObjectName(QString::fromUtf8("innerRadiusEdit"));
        innerRadiusEdit->setGeometry(QRect(90, 10, 71, 21));
        outerRadiusEdit = new QLineEdit(ArchBuilderDlgUI);
        outerRadiusEdit->setObjectName(QString::fromUtf8("outerRadiusEdit"));
        outerRadiusEdit->setGeometry(QRect(90, 40, 71, 21));
        thicknessEdit = new QLineEdit(ArchBuilderDlgUI);
        thicknessEdit->setObjectName(QString::fromUtf8("thicknessEdit"));
        thicknessEdit->setGeometry(QRect(90, 70, 71, 21));
        numberBlocksBox = new QSpinBox(ArchBuilderDlgUI);
        numberBlocksBox->setObjectName(QString::fromUtf8("numberBlocksBox"));
        numberBlocksBox->setGeometry(QRect(120, 100, 40, 20));
        supportsCheckBox = new QCheckBox(ArchBuilderDlgUI);
        supportsCheckBox->setObjectName(QString::fromUtf8("supportsCheckBox"));
        supportsCheckBox->setGeometry(QRect(10, 140, 90, 21));
        OKButton = new QPushButton(ArchBuilderDlgUI);
        OKButton->setObjectName(QString::fromUtf8("OKButton"));
        OKButton->setGeometry(QRect(10, 170, 70, 30));
        cancelButton = new QPushButton(ArchBuilderDlgUI);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));
        cancelButton->setGeometry(QRect(90, 170, 70, 31));

        retranslateUi(ArchBuilderDlgUI);
        QObject::connect(OKButton, SIGNAL(clicked()), ArchBuilderDlgUI, SLOT(accept()));
        QObject::connect(cancelButton, SIGNAL(clicked()), ArchBuilderDlgUI, SLOT(reject()));

        QMetaObject::connectSlotsByName(ArchBuilderDlgUI);
    } // setupUi

    void retranslateUi(QDialog *ArchBuilderDlgUI)
    {
        ArchBuilderDlgUI->setWindowTitle(QApplication::translate("ArchBuilderDlgUI", "Arch Builder", 0, QApplication::UnicodeUTF8));
        textLabel1->setText(QApplication::translate("ArchBuilderDlgUI", "Inner radius", 0, QApplication::UnicodeUTF8));
        textLabel4->setText(QApplication::translate("ArchBuilderDlgUI", "Thickness", 0, QApplication::UnicodeUTF8));
        textLabel2->setText(QApplication::translate("ArchBuilderDlgUI", "Outer radius", 0, QApplication::UnicodeUTF8));
        textLabel3->setText(QApplication::translate("ArchBuilderDlgUI", "Number of blocks", 0, QApplication::UnicodeUTF8));
        supportsCheckBox->setText(QApplication::translate("ArchBuilderDlgUI", "Add supports", 0, QApplication::UnicodeUTF8));
        OKButton->setText(QApplication::translate("ArchBuilderDlgUI", "OK", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("ArchBuilderDlgUI", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ArchBuilderDlgUI: public Ui_ArchBuilderDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ARCHBUILDERDLG_H
