/********************************************************************************
** Form generated from reading UI file 'compliantPlannerDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COMPLIANTPLANNERDLG_H
#define UI_COMPLIANTPLANNERDLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CompliantPlannerDlgUI
{
public:
    QPushButton *generateButton;
    QPushButton *nextButton;
    QPushButton *prevButton;
    QPushButton *bestButton;
    QLabel *label;
    QPushButton *testButton;
    QLineEdit *resolutionEdit;
    QLabel *energyLabel;
    QLabel *rankLabel;
    QPushButton *testOneButton;
    QPushButton *showOneButton;
    QPushButton *prepareOneButton;
    QLineEdit *testOneEdit;
    QLabel *currentLabel;
    QCheckBox *visualMarkersBox;
    QComboBox *energyTypeBox;
    QLabel *iterationLabel;
    QWidget *layoutWidget;
    QHBoxLayout *hboxLayout;
    QSpacerItem *spacerItem;
    QPushButton *okButton;
    QLabel *label_2;
    QPushButton *resetObjectButton;
    QLineEdit *resultsFileEdit;
    QComboBox *resultsBox;
    QFrame *line;
    QPushButton *designTestButton;
    QLineEdit *tFromEdit;
    QLineEdit *tToEdit;
    QLineEdit *tStepEdit;
    QLineEdit *sFromEdit;
    QLineEdit *sToEdit;
    QLineEdit *sStepEdit;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QFrame *line_2;

    void setupUi(QDialog *CompliantPlannerDlgUI)
    {
        if (CompliantPlannerDlgUI->objectName().isEmpty())
            CompliantPlannerDlgUI->setObjectName(QString::fromUtf8("CompliantPlannerDlgUI"));
        CompliantPlannerDlgUI->resize(246, 444);
        generateButton = new QPushButton(CompliantPlannerDlgUI);
        generateButton->setObjectName(QString::fromUtf8("generateButton"));
        generateButton->setGeometry(QRect(40, 10, 61, 25));
        nextButton = new QPushButton(CompliantPlannerDlgUI);
        nextButton->setObjectName(QString::fromUtf8("nextButton"));
        nextButton->setGeometry(QRect(90, 140, 21, 25));
        prevButton = new QPushButton(CompliantPlannerDlgUI);
        prevButton->setObjectName(QString::fromUtf8("prevButton"));
        prevButton->setGeometry(QRect(40, 140, 21, 25));
        bestButton = new QPushButton(CompliantPlannerDlgUI);
        bestButton->setObjectName(QString::fromUtf8("bestButton"));
        bestButton->setGeometry(QRect(60, 140, 31, 25));
        label = new QLabel(CompliantPlannerDlgUI);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(130, 110, 111, 16));
        testButton = new QPushButton(CompliantPlannerDlgUI);
        testButton->setObjectName(QString::fromUtf8("testButton"));
        testButton->setGeometry(QRect(10, 40, 61, 25));
        resolutionEdit = new QLineEdit(CompliantPlannerDlgUI);
        resolutionEdit->setObjectName(QString::fromUtf8("resolutionEdit"));
        resolutionEdit->setGeometry(QRect(10, 12, 23, 20));
        energyLabel = new QLabel(CompliantPlannerDlgUI);
        energyLabel->setObjectName(QString::fromUtf8("energyLabel"));
        energyLabel->setGeometry(QRect(120, 166, 101, 16));
        rankLabel = new QLabel(CompliantPlannerDlgUI);
        rankLabel->setObjectName(QString::fromUtf8("rankLabel"));
        rankLabel->setGeometry(QRect(120, 145, 91, 16));
        testOneButton = new QPushButton(CompliantPlannerDlgUI);
        testOneButton->setObjectName(QString::fromUtf8("testOneButton"));
        testOneButton->setGeometry(QRect(60, 70, 31, 25));
        showOneButton = new QPushButton(CompliantPlannerDlgUI);
        showOneButton->setObjectName(QString::fromUtf8("showOneButton"));
        showOneButton->setGeometry(QRect(100, 70, 41, 25));
        prepareOneButton = new QPushButton(CompliantPlannerDlgUI);
        prepareOneButton->setObjectName(QString::fromUtf8("prepareOneButton"));
        prepareOneButton->setGeometry(QRect(150, 70, 41, 25));
        testOneEdit = new QLineEdit(CompliantPlannerDlgUI);
        testOneEdit->setObjectName(QString::fromUtf8("testOneEdit"));
        testOneEdit->setGeometry(QRect(10, 72, 41, 20));
        currentLabel = new QLabel(CompliantPlannerDlgUI);
        currentLabel->setObjectName(QString::fromUtf8("currentLabel"));
        currentLabel->setGeometry(QRect(77, 46, 101, 16));
        visualMarkersBox = new QCheckBox(CompliantPlannerDlgUI);
        visualMarkersBox->setObjectName(QString::fromUtf8("visualMarkersBox"));
        visualMarkersBox->setGeometry(QRect(108, 14, 101, 18));
        energyTypeBox = new QComboBox(CompliantPlannerDlgUI);
        energyTypeBox->setObjectName(QString::fromUtf8("energyTypeBox"));
        energyTypeBox->setGeometry(QRect(10, 110, 111, 22));
        iterationLabel = new QLabel(CompliantPlannerDlgUI);
        iterationLabel->setObjectName(QString::fromUtf8("iterationLabel"));
        iterationLabel->setGeometry(QRect(120, 190, 101, 16));
        layoutWidget = new QWidget(CompliantPlannerDlgUI);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(120, 400, 111, 33));
        hboxLayout = new QHBoxLayout(layoutWidget);
#ifndef Q_OS_MAC
        hboxLayout->setSpacing(6);
#endif
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        spacerItem = new QSpacerItem(131, 31, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacerItem);

        okButton = new QPushButton(layoutWidget);
        okButton->setObjectName(QString::fromUtf8("okButton"));

        hboxLayout->addWidget(okButton);

        label_2 = new QLabel(CompliantPlannerDlgUI);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 374, 61, 16));
        resetObjectButton = new QPushButton(CompliantPlannerDlgUI);
        resetObjectButton->setObjectName(QString::fromUtf8("resetObjectButton"));
        resetObjectButton->setGeometry(QRect(10, 400, 75, 25));
        resultsFileEdit = new QLineEdit(CompliantPlannerDlgUI);
        resultsFileEdit->setObjectName(QString::fromUtf8("resultsFileEdit"));
        resultsFileEdit->setGeometry(QRect(152, 370, 81, 20));
        resultsBox = new QComboBox(CompliantPlannerDlgUI);
        resultsBox->setObjectName(QString::fromUtf8("resultsBox"));
        resultsBox->setGeometry(QRect(70, 370, 71, 22));
        line = new QFrame(CompliantPlannerDlgUI);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(10, 210, 221, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        designTestButton = new QPushButton(CompliantPlannerDlgUI);
        designTestButton->setObjectName(QString::fromUtf8("designTestButton"));
        designTestButton->setGeometry(QRect(10, 230, 75, 23));
        tFromEdit = new QLineEdit(CompliantPlannerDlgUI);
        tFromEdit->setObjectName(QString::fromUtf8("tFromEdit"));
        tFromEdit->setGeometry(QRect(110, 260, 31, 20));
        tToEdit = new QLineEdit(CompliantPlannerDlgUI);
        tToEdit->setObjectName(QString::fromUtf8("tToEdit"));
        tToEdit->setGeometry(QRect(110, 290, 31, 20));
        tStepEdit = new QLineEdit(CompliantPlannerDlgUI);
        tStepEdit->setObjectName(QString::fromUtf8("tStepEdit"));
        tStepEdit->setGeometry(QRect(110, 320, 31, 20));
        sFromEdit = new QLineEdit(CompliantPlannerDlgUI);
        sFromEdit->setObjectName(QString::fromUtf8("sFromEdit"));
        sFromEdit->setGeometry(QRect(150, 260, 31, 20));
        sToEdit = new QLineEdit(CompliantPlannerDlgUI);
        sToEdit->setObjectName(QString::fromUtf8("sToEdit"));
        sToEdit->setGeometry(QRect(150, 290, 31, 20));
        sStepEdit = new QLineEdit(CompliantPlannerDlgUI);
        sStepEdit->setObjectName(QString::fromUtf8("sStepEdit"));
        sStepEdit->setGeometry(QRect(150, 320, 31, 20));
        label_3 = new QLabel(CompliantPlannerDlgUI);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(80, 260, 31, 16));
        label_4 = new QLabel(CompliantPlannerDlgUI);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(90, 290, 16, 16));
        label_5 = new QLabel(CompliantPlannerDlgUI);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(80, 320, 31, 16));
        label_6 = new QLabel(CompliantPlannerDlgUI);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(110, 230, 41, 16));
        label_7 = new QLabel(CompliantPlannerDlgUI);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(150, 230, 21, 16));
        line_2 = new QFrame(CompliantPlannerDlgUI);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 340, 221, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        retranslateUi(CompliantPlannerDlgUI);
        QObject::connect(okButton, SIGNAL(clicked()), CompliantPlannerDlgUI, SLOT(accept()));

        QMetaObject::connectSlotsByName(CompliantPlannerDlgUI);
    } // setupUi

    void retranslateUi(QDialog *CompliantPlannerDlgUI)
    {
        CompliantPlannerDlgUI->setWindowTitle(QApplication::translate("CompliantPlannerDlgUI", "Compliant Planner", 0, QApplication::UnicodeUTF8));
        generateButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Generate", 0, QApplication::UnicodeUTF8));
        nextButton->setText(QApplication::translate("CompliantPlannerDlgUI", ">", 0, QApplication::UnicodeUTF8));
        prevButton->setText(QApplication::translate("CompliantPlannerDlgUI", "<", 0, QApplication::UnicodeUTF8));
        bestButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Best", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("CompliantPlannerDlgUI", "computation type", 0, QApplication::UnicodeUTF8));
        testButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Test All", 0, QApplication::UnicodeUTF8));
        resolutionEdit->setText(QString());
        energyLabel->setText(QApplication::translate("CompliantPlannerDlgUI", "Energy: 0.00", 0, QApplication::UnicodeUTF8));
        rankLabel->setText(QApplication::translate("CompliantPlannerDlgUI", "Rank: 0/0", 0, QApplication::UnicodeUTF8));
        testOneButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Test", 0, QApplication::UnicodeUTF8));
        showOneButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Show", 0, QApplication::UnicodeUTF8));
        prepareOneButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Prep", 0, QApplication::UnicodeUTF8));
        currentLabel->setText(QApplication::translate("CompliantPlannerDlgUI", "TextLabel", 0, QApplication::UnicodeUTF8));
        visualMarkersBox->setText(QApplication::translate("CompliantPlannerDlgUI", "Visual markers", 0, QApplication::UnicodeUTF8));
        iterationLabel->setText(QApplication::translate("CompliantPlannerDlgUI", "Iteration: 0", 0, QApplication::UnicodeUTF8));
        okButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Close", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("CompliantPlannerDlgUI", "Output to:", 0, QApplication::UnicodeUTF8));
        resetObjectButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Reset Object", 0, QApplication::UnicodeUTF8));
        designTestButton->setText(QApplication::translate("CompliantPlannerDlgUI", "Design Test", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("CompliantPlannerDlgUI", "From", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("CompliantPlannerDlgUI", "To", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("CompliantPlannerDlgUI", "Step", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("CompliantPlannerDlgUI", "Torque", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("CompliantPlannerDlgUI", "Stiff", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CompliantPlannerDlgUI: public Ui_CompliantPlannerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COMPLIANTPLANNERDLG_H
