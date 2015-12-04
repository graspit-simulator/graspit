/********************************************************************************
** Form generated from reading UI file 'contactExaminerDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTACTEXAMINERDLG_H
#define UI_CONTACTEXAMINERDLG_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <vector>

QT_BEGIN_NAMESPACE

class Ui_ContactExaminerDlgUI
{
public:
    Q3GroupBox *graspQualityAnalysisGroupBox;
    QPushButton *updateQualityButton;
    QLabel *textLabel3;
    QLabel *qualityLabel;
    QPushButton *showGWSButton;
    QPushButton *exitButton;
    QGroupBox *contactLoadingGroupBox;
    QPushButton *loadButton;
    QPushButton *clearButton;
    QPushButton *saveButton;
    QGroupBox *contactCollectionGroupBox;
    QPushButton *markButton;
    QLabel *textLabel1;
    QLabel *markedLabel;
    QGroupBox *groupBox;
    QRadioButton *objectRadioButton;
    QRadioButton *handRadioButton;

    void setupUi(QDialog *ContactExaminerDlgUI)
    {
        if (ContactExaminerDlgUI->objectName().isEmpty())
            ContactExaminerDlgUI->setObjectName(QString::fromUtf8("ContactExaminerDlgUI"));
        ContactExaminerDlgUI->resize(292, 321);
        graspQualityAnalysisGroupBox = new Q3GroupBox(ContactExaminerDlgUI);
        graspQualityAnalysisGroupBox->setObjectName(QString::fromUtf8("graspQualityAnalysisGroupBox"));
        graspQualityAnalysisGroupBox->setGeometry(QRect(10, 200, 270, 71));
        graspQualityAnalysisGroupBox->setOrientation(Qt::Vertical);
        updateQualityButton = new QPushButton(graspQualityAnalysisGroupBox);
        updateQualityButton->setObjectName(QString::fromUtf8("updateQualityButton"));
        updateQualityButton->setGeometry(QRect(10, 24, 71, 30));
        textLabel3 = new QLabel(graspQualityAnalysisGroupBox);
        textLabel3->setObjectName(QString::fromUtf8("textLabel3"));
        textLabel3->setGeometry(QRect(87, 27, 40, 21));
        textLabel3->setWordWrap(false);
        qualityLabel = new QLabel(graspQualityAnalysisGroupBox);
        qualityLabel->setObjectName(QString::fromUtf8("qualityLabel"));
        qualityLabel->setGeometry(QRect(131, 28, 50, 20));
        qualityLabel->setWordWrap(false);
        showGWSButton = new QPushButton(graspQualityAnalysisGroupBox);
        showGWSButton->setObjectName(QString::fromUtf8("showGWSButton"));
        showGWSButton->setGeometry(QRect(200, 24, 61, 30));
        exitButton = new QPushButton(ContactExaminerDlgUI);
        exitButton->setObjectName(QString::fromUtf8("exitButton"));
        exitButton->setGeometry(QRect(220, 280, 61, 31));
        contactLoadingGroupBox = new QGroupBox(ContactExaminerDlgUI);
        contactLoadingGroupBox->setObjectName(QString::fromUtf8("contactLoadingGroupBox"));
        contactLoadingGroupBox->setGeometry(QRect(10, 132, 270, 61));
        loadButton = new QPushButton(contactLoadingGroupBox);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));
        loadButton->setGeometry(QRect(10, 20, 71, 30));
        clearButton = new QPushButton(contactLoadingGroupBox);
        clearButton->setObjectName(QString::fromUtf8("clearButton"));
        clearButton->setGeometry(QRect(173, 20, 71, 30));
        saveButton = new QPushButton(contactLoadingGroupBox);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(90, 20, 71, 30));
        contactCollectionGroupBox = new QGroupBox(ContactExaminerDlgUI);
        contactCollectionGroupBox->setObjectName(QString::fromUtf8("contactCollectionGroupBox"));
        contactCollectionGroupBox->setGeometry(QRect(10, 62, 270, 61));
        markButton = new QPushButton(contactCollectionGroupBox);
        markButton->setObjectName(QString::fromUtf8("markButton"));
        markButton->setGeometry(QRect(10, 19, 81, 31));
        textLabel1 = new QLabel(contactCollectionGroupBox);
        textLabel1->setObjectName(QString::fromUtf8("textLabel1"));
        textLabel1->setGeometry(QRect(98, 26, 49, 20));
        textLabel1->setWordWrap(false);
        markedLabel = new QLabel(contactCollectionGroupBox);
        markedLabel->setObjectName(QString::fromUtf8("markedLabel"));
        markedLabel->setGeometry(QRect(142, 26, 20, 20));
        markedLabel->setWordWrap(false);
        groupBox = new QGroupBox(ContactExaminerDlgUI);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 10, 271, 41));
        objectRadioButton = new QRadioButton(groupBox);
        objectRadioButton->setObjectName(QString::fromUtf8("objectRadioButton"));
        objectRadioButton->setEnabled(false);
        objectRadioButton->setGeometry(QRect(69, 16, 51, 19));
        handRadioButton = new QRadioButton(groupBox);
        handRadioButton->setObjectName(QString::fromUtf8("handRadioButton"));
        handRadioButton->setEnabled(false);
        handRadioButton->setGeometry(QRect(11, 16, 51, 19));

        retranslateUi(ContactExaminerDlgUI);
        QObject::connect(exitButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(exitButton_clicked()));
        QObject::connect(updateQualityButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(updateQualityButton_clicked()));
        QObject::connect(saveButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(saveButton_clicked()));
        QObject::connect(markButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(markButton_clicked()));
        QObject::connect(loadButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(loadButton_clicked()));
        QObject::connect(clearButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(clearButton_clicked()));
        QObject::connect(showGWSButton, SIGNAL(clicked()), ContactExaminerDlgUI, SLOT(showGWSButton_clicked()));

        QMetaObject::connectSlotsByName(ContactExaminerDlgUI);
    } // setupUi

    void retranslateUi(QDialog *ContactExaminerDlgUI)
    {
        ContactExaminerDlgUI->setWindowTitle(QApplication::translate("ContactExaminerDlgUI", "Virtual Grasp", 0, QApplication::UnicodeUTF8));
        graspQualityAnalysisGroupBox->setTitle(QApplication::translate("ContactExaminerDlgUI", "Virtual Grasp Quality", 0, QApplication::UnicodeUTF8));
        updateQualityButton->setText(QApplication::translate("ContactExaminerDlgUI", "Compute", 0, QApplication::UnicodeUTF8));
        textLabel3->setText(QApplication::translate("ContactExaminerDlgUI", "Quality:", 0, QApplication::UnicodeUTF8));
        qualityLabel->setText(QApplication::translate("ContactExaminerDlgUI", "0", 0, QApplication::UnicodeUTF8));
        showGWSButton->setText(QApplication::translate("ContactExaminerDlgUI", "Show GWS", 0, QApplication::UnicodeUTF8));
        exitButton->setText(QApplication::translate("ContactExaminerDlgUI", "Exit", 0, QApplication::UnicodeUTF8));
        contactLoadingGroupBox->setTitle(QApplication::translate("ContactExaminerDlgUI", "Virtual Contacts", 0, QApplication::UnicodeUTF8));
        loadButton->setText(QApplication::translate("ContactExaminerDlgUI", "Load ...", 0, QApplication::UnicodeUTF8));
        clearButton->setText(QApplication::translate("ContactExaminerDlgUI", "Clear", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("ContactExaminerDlgUI", "Save ...", 0, QApplication::UnicodeUTF8));
        contactCollectionGroupBox->setTitle(QApplication::translate("ContactExaminerDlgUI", "Contact Collection", 0, QApplication::UnicodeUTF8));
        markButton->setText(QApplication::translate("ContactExaminerDlgUI", "Mark Contact", 0, QApplication::UnicodeUTF8));
        textLabel1->setText(QApplication::translate("ContactExaminerDlgUI", "Marked:", 0, QApplication::UnicodeUTF8));
        markedLabel->setText(QApplication::translate("ContactExaminerDlgUI", "0", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("ContactExaminerDlgUI", "Source", 0, QApplication::UnicodeUTF8));
        objectRadioButton->setText(QApplication::translate("ContactExaminerDlgUI", "object", 0, QApplication::UnicodeUTF8));
        handRadioButton->setText(QApplication::translate("ContactExaminerDlgUI", "hand", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ContactExaminerDlgUI: public Ui_ContactExaminerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTACTEXAMINERDLG_H
