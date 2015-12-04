/********************************************************************************
** Form generated from reading UI file 'plannerdlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLANNERDLG_H
#define UI_PLANNERDLG_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <Qt3Support/Q3TSFUNC>
#include <QtCore/QFile>

QT_BEGIN_NAMESPACE

class Ui_PlannerDlgUI
{
public:
    QVBoxLayout *vboxLayout;
    QHBoxLayout *hboxLayout;
    Q3GroupBox *plannerBox;
    QVBoxLayout *vboxLayout1;
    QCheckBox *automaticCheckBox;
    Q3GroupBox *automaticBox;
    QGridLayout *gridLayout;
    QLabel *TextLabel1;
    QLineEdit *densityFactorLine;
    Q3GroupBox *manualBox;
    QGridLayout *gridLayout1;
    QLineEdit *num360stepsLine;
    QHBoxLayout *hboxLayout1;
    QLineEdit *filenameLineEdit;
    QPushButton *BrowseButton;
    QLineEdit *numGraspRotsLine;
    QLineEdit *num180graspsLine;
    QLineEdit *numParPlanesLine;
    QLabel *TextLabel2;
    QLabel *TextLabel3;
    QLabel *TextLabel4;
    QLabel *TextLabel5;
    QLabel *TextLabel1_2;
    Q3GroupBox *testerBox;
    QGridLayout *gridLayout2;
    QLabel *TextLabel6;
    QLineEdit *backstepSizeLine;
    QCheckBox *visualizeBox;
    QLabel *TextLabel9;
    QLineEdit *maxNumStepsLine;
    QComboBox *qmComboBox;
    QLabel *TextLabel7;
    QHBoxLayout *hboxLayout2;
    QLineEdit *savefileLineEdit;
    QPushButton *BrowseSaveButton6;
    QSpacerItem *spacerItem;
    QLabel *textLabel1;
    QPushButton *newQMButton;
    QHBoxLayout *hboxLayout3;
    QSpacerItem *spacerItem1;
    QPushButton *GenerateButton;
    QPushButton *TestButton;
    QPushButton *ShowButton;
    QPushButton *CloseButton;

    void setupUi(QDialog *PlannerDlgUI)
    {
        if (PlannerDlgUI->objectName().isEmpty())
            PlannerDlgUI->setObjectName(QString::fromUtf8("PlannerDlgUI"));
        PlannerDlgUI->resize(535, 460);
        vboxLayout = new QVBoxLayout(PlannerDlgUI);
        vboxLayout->setSpacing(6);
        vboxLayout->setContentsMargins(11, 11, 11, 11);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        hboxLayout = new QHBoxLayout();
        hboxLayout->setSpacing(6);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        plannerBox = new Q3GroupBox(PlannerDlgUI);
        plannerBox->setObjectName(QString::fromUtf8("plannerBox"));
        plannerBox->setOrientation(Qt::Vertical);
        plannerBox->setColumnLayout(0, Qt::Vertical);
        plannerBox->layout()->setSpacing(6);
        plannerBox->layout()->setContentsMargins(11, 11, 11, 11);
        vboxLayout1 = new QVBoxLayout();
        QBoxLayout *boxlayout = qobject_cast<QBoxLayout *>(plannerBox->layout());
        if (boxlayout)
            boxlayout->addLayout(vboxLayout1);
        vboxLayout1->setAlignment(Qt::AlignTop);
        vboxLayout1->setObjectName(QString::fromUtf8("vboxLayout1"));
        automaticCheckBox = new QCheckBox(plannerBox);
        automaticCheckBox->setObjectName(QString::fromUtf8("automaticCheckBox"));
        automaticCheckBox->setChecked(true);

        vboxLayout1->addWidget(automaticCheckBox);

        automaticBox = new Q3GroupBox(plannerBox);
        automaticBox->setObjectName(QString::fromUtf8("automaticBox"));
        automaticBox->setOrientation(Qt::Vertical);
        automaticBox->setColumnLayout(0, Qt::Vertical);
        automaticBox->layout()->setSpacing(6);
        automaticBox->layout()->setContentsMargins(11, 11, 11, 11);
        gridLayout = new QGridLayout();
        QBoxLayout *boxlayout1 = qobject_cast<QBoxLayout *>(automaticBox->layout());
        if (boxlayout1)
            boxlayout1->addLayout(gridLayout);
        gridLayout->setAlignment(Qt::AlignTop);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        TextLabel1 = new QLabel(automaticBox);
        TextLabel1->setObjectName(QString::fromUtf8("TextLabel1"));
        TextLabel1->setWordWrap(false);

        gridLayout->addWidget(TextLabel1, 0, 0, 1, 1);

        densityFactorLine = new QLineEdit(automaticBox);
        densityFactorLine->setObjectName(QString::fromUtf8("densityFactorLine"));
        densityFactorLine->setMaximumSize(QSize(30, 27));

        gridLayout->addWidget(densityFactorLine, 0, 1, 1, 1);


        vboxLayout1->addWidget(automaticBox);

        manualBox = new Q3GroupBox(plannerBox);
        manualBox->setObjectName(QString::fromUtf8("manualBox"));
        manualBox->setEnabled(false);
        manualBox->setOrientation(Qt::Vertical);
        manualBox->setColumnLayout(0, Qt::Vertical);
        manualBox->layout()->setSpacing(6);
        manualBox->layout()->setContentsMargins(11, 11, 11, 11);
        gridLayout1 = new QGridLayout();
        QBoxLayout *boxlayout2 = qobject_cast<QBoxLayout *>(manualBox->layout());
        if (boxlayout2)
            boxlayout2->addLayout(gridLayout1);
        gridLayout1->setAlignment(Qt::AlignTop);
        gridLayout1->setObjectName(QString::fromUtf8("gridLayout1"));
        num360stepsLine = new QLineEdit(manualBox);
        num360stepsLine->setObjectName(QString::fromUtf8("num360stepsLine"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(0), static_cast<QSizePolicy::Policy>(0));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(num360stepsLine->sizePolicy().hasHeightForWidth());
        num360stepsLine->setSizePolicy(sizePolicy);
        num360stepsLine->setMaximumSize(QSize(30, 27));

        gridLayout1->addWidget(num360stepsLine, 0, 1, 1, 1);

        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setSpacing(6);
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        filenameLineEdit = new QLineEdit(manualBox);
        filenameLineEdit->setObjectName(QString::fromUtf8("filenameLineEdit"));

        hboxLayout1->addWidget(filenameLineEdit);

        BrowseButton = new QPushButton(manualBox);
        BrowseButton->setObjectName(QString::fromUtf8("BrowseButton"));

        hboxLayout1->addWidget(BrowseButton);


        gridLayout1->addLayout(hboxLayout1, 5, 0, 1, 2);

        numGraspRotsLine = new QLineEdit(manualBox);
        numGraspRotsLine->setObjectName(QString::fromUtf8("numGraspRotsLine"));
        sizePolicy.setHeightForWidth(numGraspRotsLine->sizePolicy().hasHeightForWidth());
        numGraspRotsLine->setSizePolicy(sizePolicy);
        numGraspRotsLine->setMaximumSize(QSize(30, 27));

        gridLayout1->addWidget(numGraspRotsLine, 3, 1, 1, 1);

        num180graspsLine = new QLineEdit(manualBox);
        num180graspsLine->setObjectName(QString::fromUtf8("num180graspsLine"));
        sizePolicy.setHeightForWidth(num180graspsLine->sizePolicy().hasHeightForWidth());
        num180graspsLine->setSizePolicy(sizePolicy);
        num180graspsLine->setMaximumSize(QSize(30, 27));

        gridLayout1->addWidget(num180graspsLine, 2, 1, 1, 1);

        numParPlanesLine = new QLineEdit(manualBox);
        numParPlanesLine->setObjectName(QString::fromUtf8("numParPlanesLine"));
        sizePolicy.setHeightForWidth(numParPlanesLine->sizePolicy().hasHeightForWidth());
        numParPlanesLine->setSizePolicy(sizePolicy);
        numParPlanesLine->setMaximumSize(QSize(30, 27));

        gridLayout1->addWidget(numParPlanesLine, 1, 1, 1, 1);

        TextLabel2 = new QLabel(manualBox);
        TextLabel2->setObjectName(QString::fromUtf8("TextLabel2"));
        TextLabel2->setWordWrap(false);

        gridLayout1->addWidget(TextLabel2, 0, 0, 1, 1);

        TextLabel3 = new QLabel(manualBox);
        TextLabel3->setObjectName(QString::fromUtf8("TextLabel3"));
        TextLabel3->setWordWrap(false);

        gridLayout1->addWidget(TextLabel3, 1, 0, 1, 1);

        TextLabel4 = new QLabel(manualBox);
        TextLabel4->setObjectName(QString::fromUtf8("TextLabel4"));
        TextLabel4->setWordWrap(false);

        gridLayout1->addWidget(TextLabel4, 2, 0, 1, 1);

        TextLabel5 = new QLabel(manualBox);
        TextLabel5->setObjectName(QString::fromUtf8("TextLabel5"));
        TextLabel5->setWordWrap(false);

        gridLayout1->addWidget(TextLabel5, 3, 0, 1, 1);

        TextLabel1_2 = new QLabel(manualBox);
        TextLabel1_2->setObjectName(QString::fromUtf8("TextLabel1_2"));
        TextLabel1_2->setAlignment(Qt::AlignCenter);
        TextLabel1_2->setWordWrap(false);

        gridLayout1->addWidget(TextLabel1_2, 4, 0, 1, 2);


        vboxLayout1->addWidget(manualBox);


        hboxLayout->addWidget(plannerBox);

        testerBox = new Q3GroupBox(PlannerDlgUI);
        testerBox->setObjectName(QString::fromUtf8("testerBox"));
        testerBox->setOrientation(Qt::Vertical);
        testerBox->setColumnLayout(0, Qt::Vertical);
        testerBox->layout()->setSpacing(6);
        testerBox->layout()->setContentsMargins(11, 11, 11, 11);
        gridLayout2 = new QGridLayout();
        QBoxLayout *boxlayout3 = qobject_cast<QBoxLayout *>(testerBox->layout());
        if (boxlayout3)
            boxlayout3->addLayout(gridLayout2);
        gridLayout2->setAlignment(Qt::AlignTop);
        gridLayout2->setObjectName(QString::fromUtf8("gridLayout2"));
        TextLabel6 = new QLabel(testerBox);
        TextLabel6->setObjectName(QString::fromUtf8("TextLabel6"));
        TextLabel6->setWordWrap(false);

        gridLayout2->addWidget(TextLabel6, 0, 0, 1, 2);

        backstepSizeLine = new QLineEdit(testerBox);
        backstepSizeLine->setObjectName(QString::fromUtf8("backstepSizeLine"));
        backstepSizeLine->setMaximumSize(QSize(30, 27));

        gridLayout2->addWidget(backstepSizeLine, 1, 2, 1, 1);

        visualizeBox = new QCheckBox(testerBox);
        visualizeBox->setObjectName(QString::fromUtf8("visualizeBox"));

        gridLayout2->addWidget(visualizeBox, 4, 0, 1, 2);

        TextLabel9 = new QLabel(testerBox);
        TextLabel9->setObjectName(QString::fromUtf8("TextLabel9"));
        TextLabel9->setWordWrap(false);

        gridLayout2->addWidget(TextLabel9, 2, 0, 1, 1);

        maxNumStepsLine = new QLineEdit(testerBox);
        maxNumStepsLine->setObjectName(QString::fromUtf8("maxNumStepsLine"));
        maxNumStepsLine->setMaximumSize(QSize(30, 27));

        gridLayout2->addWidget(maxNumStepsLine, 0, 2, 1, 1);

        qmComboBox = new QComboBox(testerBox);
        qmComboBox->setObjectName(QString::fromUtf8("qmComboBox"));

        gridLayout2->addWidget(qmComboBox, 3, 0, 1, 1);

        TextLabel7 = new QLabel(testerBox);
        TextLabel7->setObjectName(QString::fromUtf8("TextLabel7"));
        TextLabel7->setWordWrap(false);

        gridLayout2->addWidget(TextLabel7, 1, 0, 1, 2);

        hboxLayout2 = new QHBoxLayout();
        hboxLayout2->setSpacing(6);
        hboxLayout2->setContentsMargins(11, 11, 11, 11);
        hboxLayout2->setObjectName(QString::fromUtf8("hboxLayout2"));
        savefileLineEdit = new QLineEdit(testerBox);
        savefileLineEdit->setObjectName(QString::fromUtf8("savefileLineEdit"));

        hboxLayout2->addWidget(savefileLineEdit);

        BrowseSaveButton6 = new QPushButton(testerBox);
        BrowseSaveButton6->setObjectName(QString::fromUtf8("BrowseSaveButton6"));
        BrowseSaveButton6->setAutoDefault(false);

        hboxLayout2->addWidget(BrowseSaveButton6);


        gridLayout2->addLayout(hboxLayout2, 7, 0, 1, 3);

        spacerItem = new QSpacerItem(20, 120, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout2->addItem(spacerItem, 5, 0, 1, 1);

        textLabel1 = new QLabel(testerBox);
        textLabel1->setObjectName(QString::fromUtf8("textLabel1"));
        textLabel1->setWordWrap(false);

        gridLayout2->addWidget(textLabel1, 6, 0, 1, 3);

        newQMButton = new QPushButton(testerBox);
        newQMButton->setObjectName(QString::fromUtf8("newQMButton"));
        newQMButton->setAutoDefault(false);

        gridLayout2->addWidget(newQMButton, 3, 1, 1, 2);


        hboxLayout->addWidget(testerBox);


        vboxLayout->addLayout(hboxLayout);

        hboxLayout3 = new QHBoxLayout();
        hboxLayout3->setSpacing(6);
        hboxLayout3->setContentsMargins(11, 11, 11, 11);
        hboxLayout3->setObjectName(QString::fromUtf8("hboxLayout3"));
        spacerItem1 = new QSpacerItem(170, 16, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout3->addItem(spacerItem1);

        GenerateButton = new QPushButton(PlannerDlgUI);
        GenerateButton->setObjectName(QString::fromUtf8("GenerateButton"));

        hboxLayout3->addWidget(GenerateButton);

        TestButton = new QPushButton(PlannerDlgUI);
        TestButton->setObjectName(QString::fromUtf8("TestButton"));
        TestButton->setEnabled(false);
        TestButton->setAutoDefault(false);

        hboxLayout3->addWidget(TestButton);

        ShowButton = new QPushButton(PlannerDlgUI);
        ShowButton->setObjectName(QString::fromUtf8("ShowButton"));
        ShowButton->setEnabled(false);

        hboxLayout3->addWidget(ShowButton);

        CloseButton = new QPushButton(PlannerDlgUI);
        CloseButton->setObjectName(QString::fromUtf8("CloseButton"));

        hboxLayout3->addWidget(CloseButton);


        vboxLayout->addLayout(hboxLayout3);

        QWidget::setTabOrder(automaticCheckBox, densityFactorLine);
        QWidget::setTabOrder(densityFactorLine, num360stepsLine);
        QWidget::setTabOrder(num360stepsLine, numParPlanesLine);
        QWidget::setTabOrder(numParPlanesLine, num180graspsLine);
        QWidget::setTabOrder(num180graspsLine, numGraspRotsLine);
        QWidget::setTabOrder(numGraspRotsLine, maxNumStepsLine);
        QWidget::setTabOrder(maxNumStepsLine, backstepSizeLine);
        QWidget::setTabOrder(backstepSizeLine, qmComboBox);
        QWidget::setTabOrder(qmComboBox, newQMButton);
        QWidget::setTabOrder(newQMButton, visualizeBox);
        QWidget::setTabOrder(visualizeBox, GenerateButton);
        QWidget::setTabOrder(GenerateButton, ShowButton);
        QWidget::setTabOrder(ShowButton, CloseButton);

        retranslateUi(PlannerDlgUI);
        QObject::connect(CloseButton, SIGNAL(clicked()), PlannerDlgUI, SLOT(close()));
        QObject::connect(GenerateButton, SIGNAL(clicked()), PlannerDlgUI, SLOT(generateGrasps()));
        QObject::connect(ShowButton, SIGNAL(clicked()), PlannerDlgUI, SLOT(showGrasp()));
        QObject::connect(newQMButton, SIGNAL(clicked()), PlannerDlgUI, SLOT(newQM()));
        QObject::connect(automaticCheckBox, SIGNAL(toggled(bool)), manualBox, SLOT(setDisabled(bool)));
        QObject::connect(automaticCheckBox, SIGNAL(toggled(bool)), automaticBox, SLOT(setEnabled(bool)));
        QObject::connect(BrowseButton, SIGNAL(clicked()), PlannerDlgUI, SLOT(chooseFile()));
        QObject::connect(BrowseSaveButton6, SIGNAL(clicked()), PlannerDlgUI, SLOT(chooseSaveFile()));
        QObject::connect(TestButton, SIGNAL(clicked()), PlannerDlgUI, SLOT(testGrasps()));

        QMetaObject::connectSlotsByName(PlannerDlgUI);
    } // setupUi

    void retranslateUi(QDialog *PlannerDlgUI)
    {
        PlannerDlgUI->setWindowTitle(QApplication::translate("PlannerDlgUI", "Grasp Planner", 0, QApplication::UnicodeUTF8));
        plannerBox->setTitle(QApplication::translate("PlannerDlgUI", "Generator", 0, QApplication::UnicodeUTF8));
        automaticCheckBox->setText(QApplication::translate("PlannerDlgUI", "Automatic Sampling", 0, QApplication::UnicodeUTF8));
        automaticBox->setTitle(QApplication::translate("PlannerDlgUI", "Automatic", 0, QApplication::UnicodeUTF8));
        TextLabel1->setText(QApplication::translate("PlannerDlgUI", "Sampling density factor", 0, QApplication::UnicodeUTF8));
        manualBox->setTitle(QApplication::translate("PlannerDlgUI", "Manual", 0, QApplication::UnicodeUTF8));
        BrowseButton->setText(QApplication::translate("PlannerDlgUI", "Browse...", 0, QApplication::UnicodeUTF8));
        TextLabel2->setText(QApplication::translate("PlannerDlgUI", "360 Deg steps (Even)", 0, QApplication::UnicodeUTF8));
        TextLabel3->setText(QApplication::translate("PlannerDlgUI", "Parallel planes", 0, QApplication::UnicodeUTF8));
        TextLabel4->setText(QApplication::translate("PlannerDlgUI", "180 Deg grasps", 0, QApplication::UnicodeUTF8));
        TextLabel5->setText(QApplication::translate("PlannerDlgUI", "Grasp rotations", 0, QApplication::UnicodeUTF8));
        TextLabel1_2->setText(QApplication::translate("PlannerDlgUI", "or use a file", 0, QApplication::UnicodeUTF8));
        testerBox->setTitle(QApplication::translate("PlannerDlgUI", "Tester", 0, QApplication::UnicodeUTF8));
        TextLabel6->setText(QApplication::translate("PlannerDlgUI", "Max # of backsteps", 0, QApplication::UnicodeUTF8));
        visualizeBox->setText(QApplication::translate("PlannerDlgUI", "Visualize process", 0, QApplication::UnicodeUTF8));
        TextLabel9->setText(QApplication::translate("PlannerDlgUI", "Quality Measure", 0, QApplication::UnicodeUTF8));
        qmComboBox->clear();
        qmComboBox->insertItems(0, QStringList()
         << QApplication::translate("PlannerDlgUI", "None", 0, QApplication::UnicodeUTF8)
        );
        TextLabel7->setText(QApplication::translate("PlannerDlgUI", "Backstep size (mm)", 0, QApplication::UnicodeUTF8));
        BrowseSaveButton6->setText(QApplication::translate("PlannerDlgUI", "Browse...", 0, QApplication::UnicodeUTF8));
        textLabel1->setText(QApplication::translate("PlannerDlgUI", "Optional: save results to file:", 0, QApplication::UnicodeUTF8));
        newQMButton->setText(QApplication::translate("PlannerDlgUI", "New...", 0, QApplication::UnicodeUTF8));
        GenerateButton->setText(QApplication::translate("PlannerDlgUI", "Generate", 0, QApplication::UnicodeUTF8));
        TestButton->setText(QApplication::translate("PlannerDlgUI", "Test", 0, QApplication::UnicodeUTF8));
        ShowButton->setText(QApplication::translate("PlannerDlgUI", "Show", 0, QApplication::UnicodeUTF8));
        CloseButton->setText(QApplication::translate("PlannerDlgUI", "Close", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlannerDlgUI: public Ui_PlannerDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANNERDLG_H
