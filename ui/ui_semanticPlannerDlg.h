/********************************************************************************
** Form generated from reading ui file 'semanticPlannerDlg.ui'
**
** Created: Thu Apr 5 20:08:41 2012
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_SEMANTICPLANNERDLG_H
#define UI_SEMANTICPLANNERDLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_SemanticPlannerDlg
{
public:
    QDialogButtonBox *buttonBox;
    QLineEdit *codebookFilePathInlineEdit;
    QPushButton *loadScanbookButton;
    QLabel *codebookStatusLabel;
    QLineEdit *semanticGraspPathInlineEdit;
    QPushButton *loadSemanticGraspsButton;
    QGroupBox *groupBox_2;
    QPushButton *nextNeighborButton;
    QPushButton *setHandButton;
    QPushButton *adjustHandButton;
    QPushButton *previousNeighborButton;
    QGroupBox *groupBox;
    QLabel *label_2;
    QLineEdit *numberOfVotingNeighbors;
    QPushButton *voteButton;
    QLineEdit *phiLineEdit;
    QLineEdit *thetaLineEdit;
    QPushButton *bestButton;
    QPushButton *encodeButton;
    QPushButton *refineButton;
    QPushButton *scanButton;
    QPushButton *getNNButton;
    QPushButton *pauseButton;
    QPushButton *previousButton;
    QPushButton *nextButton;
    QLabel *solutionLabel;
    QPushButton *rankButton;
    QCheckBox *showArrowCheckBox;
    QLabel *SGStatusLabel;
    QGroupBox *groupBox_3;
    QPushButton *tactileButton;
    QPushButton *storeButton;
    QPushButton *showExamplePreGraspButton;
    QLineEdit *SGFileNameInlineEdit;
    QLineEdit *rotLineEdit;
    QLineEdit *bestIdxLineEdit;
    QLabel *label_4;
    QLabel *label_5;
    QPushButton *similarityMeasureButton;
    QLabel *label_3;
    QPushButton *planButton;
    QLineEdit *bestIdxLineEdit_2;
    QGroupBox *groupBox_4;
    QPushButton *executeButton;
    QFrame *line;
    QGroupBox *groupBox_5;
    QLineEdit *correspondenceFilePathInlineEdit;
    QPushButton *loadCorrespondenceButton;
    QPushButton *loadSGCorrButton;
    QLineEdit *SGCorrFilePathInlineEdit;

    void setupUi(QDialog *SemanticPlannerDlg)
    {
        if (SemanticPlannerDlg->objectName().isEmpty())
            SemanticPlannerDlg->setObjectName(QString::fromUtf8("SemanticPlannerDlg"));
        SemanticPlannerDlg->resize(957, 466);
        buttonBox = new QDialogButtonBox(SemanticPlannerDlg);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(240, 430, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        codebookFilePathInlineEdit = new QLineEdit(SemanticPlannerDlg);
        codebookFilePathInlineEdit->setObjectName(QString::fromUtf8("codebookFilePathInlineEdit"));
        codebookFilePathInlineEdit->setGeometry(QRect(20, 30, 231, 20));
        loadScanbookButton = new QPushButton(SemanticPlannerDlg);
        loadScanbookButton->setObjectName(QString::fromUtf8("loadScanbookButton"));
        loadScanbookButton->setGeometry(QRect(260, 30, 75, 23));
        codebookStatusLabel = new QLabel(SemanticPlannerDlg);
        codebookStatusLabel->setObjectName(QString::fromUtf8("codebookStatusLabel"));
        codebookStatusLabel->setGeometry(QRect(350, 30, 141, 20));
        semanticGraspPathInlineEdit = new QLineEdit(SemanticPlannerDlg);
        semanticGraspPathInlineEdit->setObjectName(QString::fromUtf8("semanticGraspPathInlineEdit"));
        semanticGraspPathInlineEdit->setGeometry(QRect(20, 60, 231, 20));
        loadSemanticGraspsButton = new QPushButton(SemanticPlannerDlg);
        loadSemanticGraspsButton->setObjectName(QString::fromUtf8("loadSemanticGraspsButton"));
        loadSemanticGraspsButton->setGeometry(QRect(260, 60, 75, 23));
        groupBox_2 = new QGroupBox(SemanticPlannerDlg);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 100, 351, 221));
        nextNeighborButton = new QPushButton(groupBox_2);
        nextNeighborButton->setObjectName(QString::fromUtf8("nextNeighborButton"));
        nextNeighborButton->setGeometry(QRect(260, 60, 75, 23));
        setHandButton = new QPushButton(groupBox_2);
        setHandButton->setObjectName(QString::fromUtf8("setHandButton"));
        setHandButton->setGeometry(QRect(100, 20, 75, 23));
        adjustHandButton = new QPushButton(groupBox_2);
        adjustHandButton->setObjectName(QString::fromUtf8("adjustHandButton"));
        adjustHandButton->setGeometry(QRect(20, 150, 75, 23));
        previousNeighborButton = new QPushButton(groupBox_2);
        previousNeighborButton->setObjectName(QString::fromUtf8("previousNeighborButton"));
        previousNeighborButton->setGeometry(QRect(100, 60, 75, 23));
        groupBox = new QGroupBox(groupBox_2);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(20, 90, 311, 51));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 20, 211, 16));
        numberOfVotingNeighbors = new QLineEdit(groupBox);
        numberOfVotingNeighbors->setObjectName(QString::fromUtf8("numberOfVotingNeighbors"));
        numberOfVotingNeighbors->setGeometry(QRect(170, 20, 31, 20));
        voteButton = new QPushButton(groupBox);
        voteButton->setObjectName(QString::fromUtf8("voteButton"));
        voteButton->setGeometry(QRect(230, 20, 75, 23));
        phiLineEdit = new QLineEdit(groupBox_2);
        phiLineEdit->setObjectName(QString::fromUtf8("phiLineEdit"));
        phiLineEdit->setGeometry(QRect(60, 20, 31, 20));
        thetaLineEdit = new QLineEdit(groupBox_2);
        thetaLineEdit->setObjectName(QString::fromUtf8("thetaLineEdit"));
        thetaLineEdit->setGeometry(QRect(20, 20, 31, 20));
        bestButton = new QPushButton(groupBox_2);
        bestButton->setObjectName(QString::fromUtf8("bestButton"));
        bestButton->setGeometry(QRect(180, 60, 75, 23));
        encodeButton = new QPushButton(groupBox_2);
        encodeButton->setObjectName(QString::fromUtf8("encodeButton"));
        encodeButton->setGeometry(QRect(260, 20, 75, 23));
        refineButton = new QPushButton(groupBox_2);
        refineButton->setObjectName(QString::fromUtf8("refineButton"));
        refineButton->setGeometry(QRect(110, 150, 75, 23));
        scanButton = new QPushButton(groupBox_2);
        scanButton->setObjectName(QString::fromUtf8("scanButton"));
        scanButton->setGeometry(QRect(180, 20, 75, 23));
        getNNButton = new QPushButton(groupBox_2);
        getNNButton->setObjectName(QString::fromUtf8("getNNButton"));
        getNNButton->setGeometry(QRect(20, 60, 75, 23));
        pauseButton = new QPushButton(groupBox_2);
        pauseButton->setObjectName(QString::fromUtf8("pauseButton"));
        pauseButton->setGeometry(QRect(290, 190, 21, 23));
        previousButton = new QPushButton(groupBox_2);
        previousButton->setObjectName(QString::fromUtf8("previousButton"));
        previousButton->setGeometry(QRect(260, 190, 31, 23));
        nextButton = new QPushButton(groupBox_2);
        nextButton->setObjectName(QString::fromUtf8("nextButton"));
        nextButton->setGeometry(QRect(310, 190, 31, 23));
        solutionLabel = new QLabel(groupBox_2);
        solutionLabel->setObjectName(QString::fromUtf8("solutionLabel"));
        solutionLabel->setGeometry(QRect(270, 160, 71, 20));
        rankButton = new QPushButton(groupBox_2);
        rankButton->setObjectName(QString::fromUtf8("rankButton"));
        rankButton->setGeometry(QRect(20, 190, 75, 23));
        showArrowCheckBox = new QCheckBox(groupBox_2);
        showArrowCheckBox->setObjectName(QString::fromUtf8("showArrowCheckBox"));
        showArrowCheckBox->setGeometry(QRect(180, 190, 70, 17));
        SGStatusLabel = new QLabel(SemanticPlannerDlg);
        SGStatusLabel->setObjectName(QString::fromUtf8("SGStatusLabel"));
        SGStatusLabel->setGeometry(QRect(350, 60, 141, 20));
        groupBox_3 = new QGroupBox(SemanticPlannerDlg);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(380, 100, 201, 231));
        tactileButton = new QPushButton(groupBox_3);
        tactileButton->setObjectName(QString::fromUtf8("tactileButton"));
        tactileButton->setGeometry(QRect(90, 180, 71, 41));
        storeButton = new QPushButton(groupBox_3);
        storeButton->setObjectName(QString::fromUtf8("storeButton"));
        storeButton->setGeometry(QRect(130, 30, 60, 91));
        showExamplePreGraspButton = new QPushButton(groupBox_3);
        showExamplePreGraspButton->setObjectName(QString::fromUtf8("showExamplePreGraspButton"));
        showExamplePreGraspButton->setGeometry(QRect(20, 180, 60, 41));
        SGFileNameInlineEdit = new QLineEdit(groupBox_3);
        SGFileNameInlineEdit->setObjectName(QString::fromUtf8("SGFileNameInlineEdit"));
        SGFileNameInlineEdit->setGeometry(QRect(20, 80, 101, 41));
        rotLineEdit = new QLineEdit(groupBox_3);
        rotLineEdit->setObjectName(QString::fromUtf8("rotLineEdit"));
        rotLineEdit->setGeometry(QRect(80, 60, 41, 20));
        bestIdxLineEdit = new QLineEdit(groupBox_3);
        bestIdxLineEdit->setObjectName(QString::fromUtf8("bestIdxLineEdit"));
        bestIdxLineEdit->setGeometry(QRect(80, 30, 41, 20));
        label_4 = new QLabel(groupBox_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 60, 51, 16));
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 32, 51, 16));
        similarityMeasureButton = new QPushButton(groupBox_3);
        similarityMeasureButton->setObjectName(QString::fromUtf8("similarityMeasureButton"));
        similarityMeasureButton->setGeometry(QRect(20, 130, 71, 41));
        label_3 = new QLabel(SemanticPlannerDlg);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(28, 331, 80, 16));
        planButton = new QPushButton(SemanticPlannerDlg);
        planButton->setObjectName(QString::fromUtf8("planButton"));
        planButton->setGeometry(QRect(188, 329, 75, 23));
        bestIdxLineEdit_2 = new QLineEdit(SemanticPlannerDlg);
        bestIdxLineEdit_2->setObjectName(QString::fromUtf8("bestIdxLineEdit_2"));
        bestIdxLineEdit_2->setGeometry(QRect(110, 330, 70, 20));
        groupBox_4 = new QGroupBox(SemanticPlannerDlg);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(380, 340, 201, 80));
        executeButton = new QPushButton(groupBox_4);
        executeButton->setObjectName(QString::fromUtf8("executeButton"));
        executeButton->setGeometry(QRect(10, 50, 75, 23));
        line = new QFrame(SemanticPlannerDlg);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(603, 30, 20, 401));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        groupBox_5 = new QGroupBox(SemanticPlannerDlg);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(620, 30, 331, 401));
        correspondenceFilePathInlineEdit = new QLineEdit(groupBox_5);
        correspondenceFilePathInlineEdit->setObjectName(QString::fromUtf8("correspondenceFilePathInlineEdit"));
        correspondenceFilePathInlineEdit->setGeometry(QRect(10, 20, 231, 20));
        loadCorrespondenceButton = new QPushButton(groupBox_5);
        loadCorrespondenceButton->setObjectName(QString::fromUtf8("loadCorrespondenceButton"));
        loadCorrespondenceButton->setGeometry(QRect(248, 20, 75, 23));
        loadSGCorrButton = new QPushButton(groupBox_5);
        loadSGCorrButton->setObjectName(QString::fromUtf8("loadSGCorrButton"));
        loadSGCorrButton->setGeometry(QRect(248, 50, 75, 23));
        SGCorrFilePathInlineEdit = new QLineEdit(groupBox_5);
        SGCorrFilePathInlineEdit->setObjectName(QString::fromUtf8("SGCorrFilePathInlineEdit"));
        SGCorrFilePathInlineEdit->setGeometry(QRect(10, 50, 231, 20));

        retranslateUi(SemanticPlannerDlg);
        QObject::connect(buttonBox, SIGNAL(accepted()), SemanticPlannerDlg, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SemanticPlannerDlg, SLOT(reject()));

        QMetaObject::connectSlotsByName(SemanticPlannerDlg);
    } // setupUi

    void retranslateUi(QDialog *SemanticPlannerDlg)
    {
        SemanticPlannerDlg->setWindowTitle(QApplication::translate("SemanticPlannerDlg", "Dialog", 0, QApplication::UnicodeUTF8));
        codebookFilePathInlineEdit->setText(QApplication::translate("SemanticPlannerDlg", "C:\\output_data\\graspit_test_scans\\guidebook504s125.txt", 0, QApplication::UnicodeUTF8));
        loadScanbookButton->setText(QApplication::translate("SemanticPlannerDlg", "Load", 0, QApplication::UnicodeUTF8));
        codebookStatusLabel->setText(QApplication::translate("SemanticPlannerDlg", "Loaded: ", 0, QApplication::UnicodeUTF8));
        semanticGraspPathInlineEdit->setText(QApplication::translate("SemanticPlannerDlg", "c:\\output_data\\graspit_test_scans\\sg_template.txt", 0, QApplication::UnicodeUTF8));
        loadSemanticGraspsButton->setText(QApplication::translate("SemanticPlannerDlg", "Load SG's", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("SemanticPlannerDlg", "Steps", 0, QApplication::UnicodeUTF8));
        nextNeighborButton->setText(QApplication::translate("SemanticPlannerDlg", "->", 0, QApplication::UnicodeUTF8));
        setHandButton->setText(QApplication::translate("SemanticPlannerDlg", "Set Hand", 0, QApplication::UnicodeUTF8));
        adjustHandButton->setText(QApplication::translate("SemanticPlannerDlg", "Adjust", 0, QApplication::UnicodeUTF8));
        previousNeighborButton->setText(QApplication::translate("SemanticPlannerDlg", "<-", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SemanticPlannerDlg", "Neighbor", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SemanticPlannerDlg", "N_id: N_LL", 0, QApplication::UnicodeUTF8));
        numberOfVotingNeighbors->setText(QApplication::translate("SemanticPlannerDlg", "5", 0, QApplication::UnicodeUTF8));
        voteButton->setText(QApplication::translate("SemanticPlannerDlg", "Vote", 0, QApplication::UnicodeUTF8));
        phiLineEdit->setText(QApplication::translate("SemanticPlannerDlg", "30", 0, QApplication::UnicodeUTF8));
        thetaLineEdit->setText(QApplication::translate("SemanticPlannerDlg", "30", 0, QApplication::UnicodeUTF8));
        bestButton->setText(QApplication::translate("SemanticPlannerDlg", "Best", 0, QApplication::UnicodeUTF8));
        encodeButton->setText(QApplication::translate("SemanticPlannerDlg", "Encode", 0, QApplication::UnicodeUTF8));
        refineButton->setText(QApplication::translate("SemanticPlannerDlg", "Refine", 0, QApplication::UnicodeUTF8));
        scanButton->setText(QApplication::translate("SemanticPlannerDlg", "Scan", 0, QApplication::UnicodeUTF8));
        getNNButton->setText(QApplication::translate("SemanticPlannerDlg", "Get N.N.", 0, QApplication::UnicodeUTF8));
        pauseButton->setText(QApplication::translate("SemanticPlannerDlg", "||", 0, QApplication::UnicodeUTF8));
        previousButton->setText(QApplication::translate("SemanticPlannerDlg", "<", 0, QApplication::UnicodeUTF8));
        nextButton->setText(QApplication::translate("SemanticPlannerDlg", ">", 0, QApplication::UnicodeUTF8));
        solutionLabel->setText(QApplication::translate("SemanticPlannerDlg", "0/0", 0, QApplication::UnicodeUTF8));
        rankButton->setText(QApplication::translate("SemanticPlannerDlg", "Rank", 0, QApplication::UnicodeUTF8));
        showArrowCheckBox->setText(QApplication::translate("SemanticPlannerDlg", "arrow", 0, QApplication::UnicodeUTF8));
        SGStatusLabel->setText(QApplication::translate("SemanticPlannerDlg", "Loaded: ", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SemanticPlannerDlg", "Utils", 0, QApplication::UnicodeUTF8));
        tactileButton->setText(QApplication::translate("SemanticPlannerDlg", "Print Tactile", 0, QApplication::UnicodeUTF8));
        storeButton->setText(QApplication::translate("SemanticPlannerDlg", "Store SG", 0, QApplication::UnicodeUTF8));
        showExamplePreGraspButton->setText(QApplication::translate("SemanticPlannerDlg", "Show", 0, QApplication::UnicodeUTF8));
        SGFileNameInlineEdit->setText(QApplication::translate("SemanticPlannerDlg", "sg.txt", 0, QApplication::UnicodeUTF8));
        rotLineEdit->setText(QApplication::translate("SemanticPlannerDlg", "90", 0, QApplication::UnicodeUTF8));
        bestIdxLineEdit->setText(QApplication::translate("SemanticPlannerDlg", "45", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SemanticPlannerDlg", "roll", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("SemanticPlannerDlg", "index", 0, QApplication::UnicodeUTF8));
        similarityMeasureButton->setText(QApplication::translate("SemanticPlannerDlg", "Cal Similarity", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SemanticPlannerDlg", "Example SG ID:", 0, QApplication::UnicodeUTF8));
        planButton->setText(QApplication::translate("SemanticPlannerDlg", "Plan", 0, QApplication::UnicodeUTF8));
        bestIdxLineEdit_2->setText(QApplication::translate("SemanticPlannerDlg", "0", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("SemanticPlannerDlg", "Execution", 0, QApplication::UnicodeUTF8));
        executeButton->setText(QApplication::translate("SemanticPlannerDlg", "Execute", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("SemanticPlannerDlg", "Shape correspondence", 0, QApplication::UnicodeUTF8));
        correspondenceFilePathInlineEdit->setText(QString());
        loadCorrespondenceButton->setText(QApplication::translate("SemanticPlannerDlg", "Load Cor.", 0, QApplication::UnicodeUTF8));
        loadSGCorrButton->setText(QApplication::translate("SemanticPlannerDlg", "Load SG's", 0, QApplication::UnicodeUTF8));
        SGCorrFilePathInlineEdit->setText(QString());
        Q_UNUSED(SemanticPlannerDlg);
    } // retranslateUi

};

namespace Ui {
    class SemanticPlannerDlg: public Ui_SemanticPlannerDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SEMANTICPLANNERDLG_H
