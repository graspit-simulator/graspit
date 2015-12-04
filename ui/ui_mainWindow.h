/********************************************************************************
** Form generated from reading UI file 'mainWindow.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3ListBox>
#include <Qt3Support/Q3MainWindow>
#include <Qt3Support/Q3MimeSourceFactory>
#include <Qt3Support/Q3ToolBar>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowUI
{
public:
    QAction *fileNewAction;
    QAction *fileOpenAction;
    QAction *fileSaveAction;
    QAction *fileSaveAsAction;
    QAction *fileExitAction;
    QAction *helpManualAction;
    QAction *helpIndexAction;
    QAction *helpAboutAction;
    QAction *fileImportRobotAction;
    QAction *fileImportObjectAction;
    QAction *fileImportObstacleAction;
    QAction *fileSaveImageAction;
    QAction *graspCreateProjectionAction;
    QAction *graspQualityMeasuresAction;
    QAction *fileEditSettingsAction;
    QAction *graspPlannerAction;
    QAction *miscOptimizerAction;
    QAction *actionArizona_Project;
    QAction *staubliControl;
    QAction *graspGFOAction;
    QAction *graspCompliantPlannerAction;
    QAction *dynamicsPlayAction;
    QAction *dynamicsPopAction;
    QAction *dynamicsPushAction;
    QAction *elementCollisionToggleAction;
    QAction *elementBodyPropertiesAction;
    QAction *graspAutoGraspAction;
    QAction *helpAboutQTAction;
    QAction *fileImport_Human_HandAction;
    QAction *dynamicsArch_BuilderAction;
    QAction *sensorsSensor_InputAction;
    QAction *stereoOnAction;
    QAction *stereoOffAction;
    QAction *stereoFlip_leftrightAction;
    QAction *dbaseGraspCaptureAction;
    QAction *graspEigenGrasp_InterfaceAction;
    QAction *graspContact_ExaminerAction;
    QAction *graspEigenGrasp_PlannerAction;
    QAction *sensorsBarrett_HandAction;
    QAction *graspAuto_OpenAction;
    QAction *elementPrimitivesAction;
    QAction *miscEigengridsAction;
    QAction *dbaseGUIAction;
    QAction *dbasePlannerAction;
    QActionGroup *elementGroup;
    QAction *translateToolAction;
    QAction *rotateToolAction;
    QAction *selectToolAction;
    QWidget *widget;
    QVBoxLayout *vboxLayout;
    Q3GroupBox *worldBox;
    QHBoxLayout *hboxLayout;
    QWidget *viewerHolder;
    QHBoxLayout *hboxLayout1;
    Q3GroupBox *qualityGroupBox;
    QHBoxLayout *hboxLayout2;
    Q3ListBox *qualityListBox;
    Q3GroupBox *contactsGroupBox;
    QHBoxLayout *hboxLayout3;
    Q3ListBox *contactsListBox;
    Q3ToolBar *toolBar;
    Q3ToolBar *Toolbar_2;
    QComboBox *materialComboBox;
    Q3ToolBar *ToolbarDynamics;
    QLCDNumber *timeReadout;
    Q3ToolBar *graspToolbar;
    QComboBox *handSelectionBox;
    QComboBox *graspedBodyBox;
    Q3ToolBar *tendonToolbar;
    QLabel *Tendon_force_label;
    QComboBox *tendonNamesBox;
    QLabel *tendonActiveForceLabel;
    QSpinBox *TendonForceInput;
    QLabel *tendonPassiveForceLabel;
    QLineEdit *tendonPassiveForceEdit;
    QLabel *tendonExcursionLabel;
    QLineEdit *tendonExcursionEdit;
    QLabel *tendonVisibleLabel;
    QCheckBox *tendonVisibleCheckBox;
    QLabel *forcesVisibleLabel;
    QCheckBox *forcesVisibleCheckBox;
    QMenuBar *menubar;
    QMenu *fileMenu;
    QMenu *elementMenu;
    QMenu *graspMenu;
    QMenu *dbaseMenu;
    QMenu *Sensors;
    QMenu *Stereo;
    QMenu *Misc;
    QMenu *helpMenu;

    void setupUi(Q3MainWindow *MainWindowUI)
    {
        if (MainWindowUI->objectName().isEmpty())
            MainWindowUI->setObjectName(QString::fromUtf8("MainWindowUI"));
        MainWindowUI->resize(879, 710);
        fileNewAction = new QAction(MainWindowUI);
        fileNewAction->setObjectName(QString::fromUtf8("fileNewAction"));
        fileNewAction->setName("fileNewAction");
        const QIcon icon = QIcon(qPixmapFromMimeSource("filenew.xpm"));
        fileNewAction->setIcon(icon);
        fileOpenAction = new QAction(MainWindowUI);
        fileOpenAction->setObjectName(QString::fromUtf8("fileOpenAction"));
        fileOpenAction->setName("fileOpenAction");
        const QIcon icon1 = QIcon(qPixmapFromMimeSource("fileopen.xpm"));
        fileOpenAction->setIcon(icon1);
        fileSaveAction = new QAction(MainWindowUI);
        fileSaveAction->setObjectName(QString::fromUtf8("fileSaveAction"));
        fileSaveAction->setName("fileSaveAction");
        const QIcon icon2 = QIcon(qPixmapFromMimeSource("filesave.xpm"));
        fileSaveAction->setIcon(icon2);
        fileSaveAsAction = new QAction(MainWindowUI);
        fileSaveAsAction->setObjectName(QString::fromUtf8("fileSaveAsAction"));
        fileSaveAsAction->setName("fileSaveAsAction");
        fileExitAction = new QAction(MainWindowUI);
        fileExitAction->setObjectName(QString::fromUtf8("fileExitAction"));
        fileExitAction->setName("fileExitAction");
        helpManualAction = new QAction(MainWindowUI);
        helpManualAction->setObjectName(QString::fromUtf8("helpManualAction"));
        helpManualAction->setName("helpManualAction");
        helpIndexAction = new QAction(MainWindowUI);
        helpIndexAction->setObjectName(QString::fromUtf8("helpIndexAction"));
        helpIndexAction->setName("helpIndexAction");
        helpAboutAction = new QAction(MainWindowUI);
        helpAboutAction->setObjectName(QString::fromUtf8("helpAboutAction"));
        helpAboutAction->setName("helpAboutAction");
        fileImportRobotAction = new QAction(MainWindowUI);
        fileImportRobotAction->setObjectName(QString::fromUtf8("fileImportRobotAction"));
        fileImportRobotAction->setName("fileImportRobotAction");
        fileImportObjectAction = new QAction(MainWindowUI);
        fileImportObjectAction->setObjectName(QString::fromUtf8("fileImportObjectAction"));
        fileImportObjectAction->setName("fileImportObjectAction");
        fileImportObstacleAction = new QAction(MainWindowUI);
        fileImportObstacleAction->setObjectName(QString::fromUtf8("fileImportObstacleAction"));
        fileImportObstacleAction->setName("fileImportObstacleAction");
        fileSaveImageAction = new QAction(MainWindowUI);
        fileSaveImageAction->setObjectName(QString::fromUtf8("fileSaveImageAction"));
        fileSaveImageAction->setName("fileSaveImageAction");
        graspCreateProjectionAction = new QAction(MainWindowUI);
        graspCreateProjectionAction->setObjectName(QString::fromUtf8("graspCreateProjectionAction"));
        graspCreateProjectionAction->setName("graspCreateProjectionAction");
        graspQualityMeasuresAction = new QAction(MainWindowUI);
        graspQualityMeasuresAction->setObjectName(QString::fromUtf8("graspQualityMeasuresAction"));
        graspQualityMeasuresAction->setName("graspQualityMeasuresAction");
        fileEditSettingsAction = new QAction(MainWindowUI);
        fileEditSettingsAction->setObjectName(QString::fromUtf8("fileEditSettingsAction"));
        fileEditSettingsAction->setName("fileEditSettingsAction");
        graspPlannerAction = new QAction(MainWindowUI);
        graspPlannerAction->setObjectName(QString::fromUtf8("graspPlannerAction"));
        graspPlannerAction->setName("graspPlannerAction");
        miscOptimizerAction = new QAction(MainWindowUI);
        miscOptimizerAction->setObjectName(QString::fromUtf8("miscOptimizerAction"));
        miscOptimizerAction->setName("miscOptimizerAction");
        actionArizona_Project = new QAction(MainWindowUI);
        actionArizona_Project->setObjectName(QString::fromUtf8("actionArizona_Project"));
        actionArizona_Project->setName("actionArizona_Project");
        staubliControl = new QAction(MainWindowUI);
        staubliControl->setObjectName(QString::fromUtf8("staubliControl"));
        staubliControl->setName("staubliControl");
        graspGFOAction = new QAction(MainWindowUI);
        graspGFOAction->setObjectName(QString::fromUtf8("graspGFOAction"));
        graspGFOAction->setName("graspGFOAction");
        graspCompliantPlannerAction = new QAction(MainWindowUI);
        graspCompliantPlannerAction->setObjectName(QString::fromUtf8("graspCompliantPlannerAction"));
        graspCompliantPlannerAction->setName("graspCompliantPlannerAction");
        dynamicsPlayAction = new QAction(MainWindowUI);
        dynamicsPlayAction->setObjectName(QString::fromUtf8("dynamicsPlayAction"));
        dynamicsPlayAction->setName("dynamicsPlayAction");
        dynamicsPlayAction->setCheckable(true);
        const QIcon icon3 = QIcon(qPixmapFromMimeSource("play.xpm"));
        dynamicsPlayAction->setIcon(icon3);
        dynamicsPopAction = new QAction(MainWindowUI);
        dynamicsPopAction->setObjectName(QString::fromUtf8("dynamicsPopAction"));
        dynamicsPopAction->setName("dynamicsPopAction");
        const QIcon icon4 = QIcon(qPixmapFromMimeSource("prevMark.xpm"));
        dynamicsPopAction->setIcon(icon4);
        dynamicsPushAction = new QAction(MainWindowUI);
        dynamicsPushAction->setObjectName(QString::fromUtf8("dynamicsPushAction"));
        dynamicsPushAction->setName("dynamicsPushAction");
        const QIcon icon5 = QIcon(qPixmapFromMimeSource("mark.xpm"));
        dynamicsPushAction->setIcon(icon5);
        elementCollisionToggleAction = new QAction(MainWindowUI);
        elementCollisionToggleAction->setObjectName(QString::fromUtf8("elementCollisionToggleAction"));
        elementCollisionToggleAction->setName("elementCollisionToggleAction");
        elementCollisionToggleAction->setCheckable(true);
        const QIcon icon6 = QIcon(qPixmapFromMimeSource("collide.xpm"));
        elementCollisionToggleAction->setIcon(icon6);
        elementBodyPropertiesAction = new QAction(MainWindowUI);
        elementBodyPropertiesAction->setObjectName(QString::fromUtf8("elementBodyPropertiesAction"));
        elementBodyPropertiesAction->setName("elementBodyPropertiesAction");
        elementBodyPropertiesAction->setEnabled(false);
        graspAutoGraspAction = new QAction(MainWindowUI);
        graspAutoGraspAction->setObjectName(QString::fromUtf8("graspAutoGraspAction"));
        graspAutoGraspAction->setName("graspAutoGraspAction");
        helpAboutQTAction = new QAction(MainWindowUI);
        helpAboutQTAction->setObjectName(QString::fromUtf8("helpAboutQTAction"));
        helpAboutQTAction->setName("helpAboutQTAction");
        fileImport_Human_HandAction = new QAction(MainWindowUI);
        fileImport_Human_HandAction->setObjectName(QString::fromUtf8("fileImport_Human_HandAction"));
        fileImport_Human_HandAction->setName("fileImport_Human_HandAction");
        dynamicsArch_BuilderAction = new QAction(MainWindowUI);
        dynamicsArch_BuilderAction->setObjectName(QString::fromUtf8("dynamicsArch_BuilderAction"));
        dynamicsArch_BuilderAction->setName("dynamicsArch_BuilderAction");
        sensorsSensor_InputAction = new QAction(MainWindowUI);
        sensorsSensor_InputAction->setObjectName(QString::fromUtf8("sensorsSensor_InputAction"));
        sensorsSensor_InputAction->setName("sensorsSensor_InputAction");
        stereoOnAction = new QAction(MainWindowUI);
        stereoOnAction->setObjectName(QString::fromUtf8("stereoOnAction"));
        stereoOnAction->setName("stereoOnAction");
        stereoOffAction = new QAction(MainWindowUI);
        stereoOffAction->setObjectName(QString::fromUtf8("stereoOffAction"));
        stereoOffAction->setName("stereoOffAction");
        stereoFlip_leftrightAction = new QAction(MainWindowUI);
        stereoFlip_leftrightAction->setObjectName(QString::fromUtf8("stereoFlip_leftrightAction"));
        stereoFlip_leftrightAction->setName("stereoFlip_leftrightAction");
        dbaseGraspCaptureAction = new QAction(MainWindowUI);
        dbaseGraspCaptureAction->setObjectName(QString::fromUtf8("dbaseGraspCaptureAction"));
        dbaseGraspCaptureAction->setName("dbaseGraspCaptureAction");
        graspEigenGrasp_InterfaceAction = new QAction(MainWindowUI);
        graspEigenGrasp_InterfaceAction->setObjectName(QString::fromUtf8("graspEigenGrasp_InterfaceAction"));
        graspEigenGrasp_InterfaceAction->setName("graspEigenGrasp_InterfaceAction");
        graspContact_ExaminerAction = new QAction(MainWindowUI);
        graspContact_ExaminerAction->setObjectName(QString::fromUtf8("graspContact_ExaminerAction"));
        graspContact_ExaminerAction->setName("graspContact_ExaminerAction");
        graspEigenGrasp_PlannerAction = new QAction(MainWindowUI);
        graspEigenGrasp_PlannerAction->setObjectName(QString::fromUtf8("graspEigenGrasp_PlannerAction"));
        graspEigenGrasp_PlannerAction->setName("graspEigenGrasp_PlannerAction");
        sensorsBarrett_HandAction = new QAction(MainWindowUI);
        sensorsBarrett_HandAction->setObjectName(QString::fromUtf8("sensorsBarrett_HandAction"));
        sensorsBarrett_HandAction->setName("sensorsBarrett_HandAction");
        graspAuto_OpenAction = new QAction(MainWindowUI);
        graspAuto_OpenAction->setObjectName(QString::fromUtf8("graspAuto_OpenAction"));
        graspAuto_OpenAction->setName("graspAuto_OpenAction");
        elementPrimitivesAction = new QAction(MainWindowUI);
        elementPrimitivesAction->setObjectName(QString::fromUtf8("elementPrimitivesAction"));
        elementPrimitivesAction->setName("elementPrimitivesAction");
        elementPrimitivesAction->setEnabled(false);
        miscEigengridsAction = new QAction(MainWindowUI);
        miscEigengridsAction->setObjectName(QString::fromUtf8("miscEigengridsAction"));
        miscEigengridsAction->setName("miscEigengridsAction");
        dbaseGUIAction = new QAction(MainWindowUI);
        dbaseGUIAction->setObjectName(QString::fromUtf8("dbaseGUIAction"));
        dbaseGUIAction->setName("dbaseGUIAction");
        dbasePlannerAction = new QAction(MainWindowUI);
        dbasePlannerAction->setObjectName(QString::fromUtf8("dbasePlannerAction"));
        dbasePlannerAction->setName("dbasePlannerAction");
        elementGroup = new QActionGroup(MainWindowUI);
        elementGroup->setObjectName(QString::fromUtf8("elementGroup"));
        elementGroup->setName("elementGroup");
        translateToolAction = new QAction(elementGroup);
        translateToolAction->setObjectName(QString::fromUtf8("translateToolAction"));
        translateToolAction->setName("translateToolAction");
        translateToolAction->setCheckable(true);
        translateToolAction->setChecked(true);
        const QIcon icon7 = QIcon(qPixmapFromMimeSource("translateTool.xpm"));
        translateToolAction->setIcon(icon7);
        rotateToolAction = new QAction(elementGroup);
        rotateToolAction->setObjectName(QString::fromUtf8("rotateToolAction"));
        rotateToolAction->setName("rotateToolAction");
        rotateToolAction->setCheckable(true);
        const QIcon icon8 = QIcon(qPixmapFromMimeSource("rotateTool.xpm"));
        rotateToolAction->setIcon(icon8);
        selectToolAction = new QAction(elementGroup);
        selectToolAction->setObjectName(QString::fromUtf8("selectToolAction"));
        selectToolAction->setName("selectToolAction");
        selectToolAction->setCheckable(true);
        const QIcon icon9 = QIcon(qPixmapFromMimeSource("selectTool.xpm"));
        selectToolAction->setIcon(icon9);
        widget = new QWidget(MainWindowUI);
        widget->setObjectName(QString::fromUtf8("widget"));
        vboxLayout = new QVBoxLayout(widget);
        vboxLayout->setSpacing(0);
        vboxLayout->setContentsMargins(1, 1, 1, 1);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        vboxLayout->setContentsMargins(0, 0, 0, 0);
        worldBox = new Q3GroupBox(widget);
        worldBox->setObjectName(QString::fromUtf8("worldBox"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(7), static_cast<QSizePolicy::Policy>(7));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(worldBox->sizePolicy().hasHeightForWidth());
        worldBox->setSizePolicy(sizePolicy);
        worldBox->setColumnLayout(0, Qt::Vertical);
        worldBox->layout()->setSpacing(0);
        worldBox->layout()->setContentsMargins(1, 1, 1, 1);
        hboxLayout = new QHBoxLayout();
        QBoxLayout *boxlayout = qobject_cast<QBoxLayout *>(worldBox->layout());
        if (boxlayout)
            boxlayout->addLayout(hboxLayout);
        hboxLayout->setAlignment(Qt::AlignTop);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        viewerHolder = new QWidget(worldBox);
        viewerHolder->setObjectName(QString::fromUtf8("viewerHolder"));
        sizePolicy.setHeightForWidth(viewerHolder->sizePolicy().hasHeightForWidth());
        viewerHolder->setSizePolicy(sizePolicy);
        viewerHolder->setMinimumSize(QSize(300, 200));

        hboxLayout->addWidget(viewerHolder);


        vboxLayout->addWidget(worldBox);

        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setSpacing(6);
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        qualityGroupBox = new Q3GroupBox(widget);
        qualityGroupBox->setObjectName(QString::fromUtf8("qualityGroupBox"));
        QSizePolicy sizePolicy1(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(0));
        sizePolicy1.setHorizontalStretch(2);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(qualityGroupBox->sizePolicy().hasHeightForWidth());
        qualityGroupBox->setSizePolicy(sizePolicy1);
        qualityGroupBox->setMinimumSize(QSize(100, 120));
        qualityGroupBox->setColumnLayout(0, Qt::Vertical);
        qualityGroupBox->layout()->setSpacing(6);
        qualityGroupBox->layout()->setContentsMargins(11, 11, 11, 11);
        hboxLayout2 = new QHBoxLayout();
        QBoxLayout *boxlayout1 = qobject_cast<QBoxLayout *>(qualityGroupBox->layout());
        if (boxlayout1)
            boxlayout1->addLayout(hboxLayout2);
        hboxLayout2->setAlignment(Qt::AlignTop);
        hboxLayout2->setObjectName(QString::fromUtf8("hboxLayout2"));
        qualityListBox = new Q3ListBox(qualityGroupBox);
        qualityListBox->setObjectName(QString::fromUtf8("qualityListBox"));
        qualityListBox->setFocusPolicy(Qt::NoFocus);
        qualityListBox->setSelectionMode(Q3ListBox::NoSelection);

        hboxLayout2->addWidget(qualityListBox);


        hboxLayout1->addWidget(qualityGroupBox);

        contactsGroupBox = new Q3GroupBox(widget);
        contactsGroupBox->setObjectName(QString::fromUtf8("contactsGroupBox"));
        sizePolicy1.setHeightForWidth(contactsGroupBox->sizePolicy().hasHeightForWidth());
        contactsGroupBox->setSizePolicy(sizePolicy1);
        contactsGroupBox->setMinimumSize(QSize(100, 120));
        contactsGroupBox->setColumnLayout(0, Qt::Vertical);
        contactsGroupBox->layout()->setSpacing(6);
        contactsGroupBox->layout()->setContentsMargins(11, 11, 11, 11);
        hboxLayout3 = new QHBoxLayout();
        QBoxLayout *boxlayout2 = qobject_cast<QBoxLayout *>(contactsGroupBox->layout());
        if (boxlayout2)
            boxlayout2->addLayout(hboxLayout3);
        hboxLayout3->setAlignment(Qt::AlignTop);
        hboxLayout3->setObjectName(QString::fromUtf8("hboxLayout3"));
        contactsListBox = new Q3ListBox(contactsGroupBox);
        contactsListBox->setObjectName(QString::fromUtf8("contactsListBox"));
        contactsListBox->setFocusPolicy(Qt::NoFocus);

        hboxLayout3->addWidget(contactsListBox);


        hboxLayout1->addWidget(contactsGroupBox);


        vboxLayout->addLayout(hboxLayout1);

        MainWindowUI->setCentralWidget(widget);
        toolBar = new Q3ToolBar(MainWindowUI);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setResizeEnabled(false);
        Toolbar_2 = new Q3ToolBar(MainWindowUI);
        Toolbar_2->setObjectName(QString::fromUtf8("Toolbar_2"));
        Toolbar_2->setResizeEnabled(false);
        materialComboBox = new QComboBox();
        materialComboBox->setObjectName(QString::fromUtf8("materialComboBox"));
        materialComboBox->setFocusPolicy(Qt::NoFocus);
        ToolbarDynamics = new Q3ToolBar(MainWindowUI);
        ToolbarDynamics->setObjectName(QString::fromUtf8("ToolbarDynamics"));
        ToolbarDynamics->setResizeEnabled(false);
        timeReadout = new QLCDNumber();
        timeReadout->setObjectName(QString::fromUtf8("timeReadout"));
        timeReadout->setNumDigits(9);
        graspToolbar = new Q3ToolBar(MainWindowUI);
        graspToolbar->setObjectName(QString::fromUtf8("graspToolbar"));
        graspToolbar->setResizeEnabled(false);
        handSelectionBox = new QComboBox();
        handSelectionBox->setObjectName(QString::fromUtf8("handSelectionBox"));
        handSelectionBox->setFocusPolicy(Qt::NoFocus);
        graspedBodyBox = new QComboBox();
        graspedBodyBox->setObjectName(QString::fromUtf8("graspedBodyBox"));
        graspedBodyBox->setFocusPolicy(Qt::NoFocus);
        tendonToolbar = new Q3ToolBar(MainWindowUI);
        tendonToolbar->setObjectName(QString::fromUtf8("tendonToolbar"));
        tendonToolbar->setResizeEnabled(false);
        Tendon_force_label = new QLabel();
        Tendon_force_label->setObjectName(QString::fromUtf8("Tendon_force_label"));
        Tendon_force_label->setMargin(4);
        Tendon_force_label->setWordWrap(false);
        tendonNamesBox = new QComboBox();
        tendonNamesBox->setObjectName(QString::fromUtf8("tendonNamesBox"));
        QSizePolicy sizePolicy2(static_cast<QSizePolicy::Policy>(7), static_cast<QSizePolicy::Policy>(0));
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(tendonNamesBox->sizePolicy().hasHeightForWidth());
        tendonNamesBox->setSizePolicy(sizePolicy2);
        tendonNamesBox->setMinimumSize(QSize(150, 0));
        tendonActiveForceLabel = new QLabel();
        tendonActiveForceLabel->setObjectName(QString::fromUtf8("tendonActiveForceLabel"));
        tendonActiveForceLabel->setMargin(4);
        tendonActiveForceLabel->setAlignment(Qt::AlignVCenter);
        tendonActiveForceLabel->setWordWrap(false);
        TendonForceInput = new QSpinBox();
        TendonForceInput->setObjectName(QString::fromUtf8("TendonForceInput"));
        tendonPassiveForceLabel = new QLabel();
        tendonPassiveForceLabel->setObjectName(QString::fromUtf8("tendonPassiveForceLabel"));
        tendonPassiveForceLabel->setMargin(4);
        tendonPassiveForceLabel->setWordWrap(false);
        tendonPassiveForceEdit = new QLineEdit();
        tendonPassiveForceEdit->setObjectName(QString::fromUtf8("tendonPassiveForceEdit"));
        QSizePolicy sizePolicy3(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(0));
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(tendonPassiveForceEdit->sizePolicy().hasHeightForWidth());
        tendonPassiveForceEdit->setSizePolicy(sizePolicy3);
        tendonPassiveForceEdit->setMaximumSize(QSize(40, 32767));
        tendonPassiveForceEdit->setMaxLength(4);
        tendonPassiveForceEdit->setReadOnly(true);
        tendonExcursionLabel = new QLabel();
        tendonExcursionLabel->setObjectName(QString::fromUtf8("tendonExcursionLabel"));
        tendonExcursionLabel->setMargin(4);
        tendonExcursionLabel->setWordWrap(false);
        tendonExcursionEdit = new QLineEdit();
        tendonExcursionEdit->setObjectName(QString::fromUtf8("tendonExcursionEdit"));
        tendonExcursionEdit->setMaximumSize(QSize(40, 32767));
        tendonExcursionEdit->setReadOnly(true);
        tendonVisibleLabel = new QLabel();
        tendonVisibleLabel->setObjectName(QString::fromUtf8("tendonVisibleLabel"));
        tendonVisibleLabel->setMargin(4);
        tendonVisibleLabel->setWordWrap(false);
        tendonVisibleCheckBox = new QCheckBox();
        tendonVisibleCheckBox->setObjectName(QString::fromUtf8("tendonVisibleCheckBox"));
        forcesVisibleLabel = new QLabel();
        forcesVisibleLabel->setObjectName(QString::fromUtf8("forcesVisibleLabel"));
        forcesVisibleLabel->setMargin(4);
        forcesVisibleLabel->setWordWrap(false);
        forcesVisibleCheckBox = new QCheckBox();
        forcesVisibleCheckBox->setObjectName(QString::fromUtf8("forcesVisibleCheckBox"));
        menubar = new QMenuBar(MainWindowUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        fileMenu = new QMenu(menubar);
        fileMenu->setObjectName(QString::fromUtf8("fileMenu"));
        elementMenu = new QMenu(menubar);
        elementMenu->setObjectName(QString::fromUtf8("elementMenu"));
        graspMenu = new QMenu(menubar);
        graspMenu->setObjectName(QString::fromUtf8("graspMenu"));
        dbaseMenu = new QMenu(menubar);
        dbaseMenu->setObjectName(QString::fromUtf8("dbaseMenu"));
        Sensors = new QMenu(menubar);
        Sensors->setObjectName(QString::fromUtf8("Sensors"));
        Stereo = new QMenu(menubar);
        Stereo->setObjectName(QString::fromUtf8("Stereo"));
        Misc = new QMenu(menubar);
        Misc->setObjectName(QString::fromUtf8("Misc"));
        helpMenu = new QMenu(menubar);
        helpMenu->setObjectName(QString::fromUtf8("helpMenu"));

        toolBar->addAction(fileNewAction);
        toolBar->addAction(fileOpenAction);
        toolBar->addAction(fileSaveAction);
        toolBar->addSeparator();
        Toolbar_2->addAction(translateToolAction);
        Toolbar_2->addAction(rotateToolAction);
        Toolbar_2->addAction(selectToolAction);
        Toolbar_2->addSeparator();
        Toolbar_2->addAction(elementCollisionToggleAction);
        materialComboBox->setParent(Toolbar_2);
        ToolbarDynamics->addAction(dynamicsPopAction);
        ToolbarDynamics->addAction(dynamicsPushAction);
        ToolbarDynamics->addAction(dynamicsPlayAction);
        timeReadout->setParent(ToolbarDynamics);
        handSelectionBox->setParent(graspToolbar);
        graspedBodyBox->setParent(graspToolbar);
        Tendon_force_label->setParent(tendonToolbar);
        tendonNamesBox->setParent(tendonToolbar);
        tendonActiveForceLabel->setParent(tendonToolbar);
        TendonForceInput->setParent(tendonToolbar);
        tendonPassiveForceLabel->setParent(tendonToolbar);
        tendonPassiveForceEdit->setParent(tendonToolbar);
        tendonExcursionLabel->setParent(tendonToolbar);
        tendonExcursionEdit->setParent(tendonToolbar);
        tendonVisibleLabel->setParent(tendonToolbar);
        tendonVisibleCheckBox->setParent(tendonToolbar);
        forcesVisibleLabel->setParent(tendonToolbar);
        forcesVisibleCheckBox->setParent(tendonToolbar);
        menubar->addAction(fileMenu->menuAction());
        menubar->addAction(elementMenu->menuAction());
        menubar->addAction(graspMenu->menuAction());
        menubar->addAction(dbaseMenu->menuAction());
        menubar->addAction(Sensors->menuAction());
        menubar->addAction(Stereo->menuAction());
        menubar->addAction(Misc->menuAction());
        menubar->addAction(helpMenu->menuAction());
        fileMenu->addAction(fileNewAction);
        fileMenu->addAction(fileOpenAction);
        fileMenu->addAction(fileSaveAction);
        fileMenu->addAction(fileSaveAsAction);
        fileMenu->addAction(fileSaveImageAction);
        fileMenu->addSeparator();
        fileMenu->addAction(fileImportRobotAction);
        fileMenu->addAction(fileImportObjectAction);
        fileMenu->addAction(fileImportObstacleAction);
        fileMenu->addSeparator();
        fileMenu->addAction(fileEditSettingsAction);
        fileMenu->addSeparator();
        fileMenu->addSeparator();
        fileMenu->addAction(fileExitAction);
        elementMenu->addAction(translateToolAction);
        elementMenu->addAction(rotateToolAction);
        elementMenu->addAction(selectToolAction);
        elementMenu->addSeparator();
        elementMenu->addAction(elementCollisionToggleAction);
        elementMenu->addAction(elementBodyPropertiesAction);
        elementMenu->addAction(elementPrimitivesAction);
        graspMenu->addAction(graspAutoGraspAction);
        graspMenu->addAction(graspAuto_OpenAction);
        graspMenu->addAction(graspCreateProjectionAction);
        graspMenu->addAction(graspQualityMeasuresAction);
        graspMenu->addAction(graspPlannerAction);
        graspMenu->addAction(graspGFOAction);
        graspMenu->addSeparator();
        graspMenu->addAction(graspEigenGrasp_InterfaceAction);
        graspMenu->addAction(graspEigenGrasp_PlannerAction);
        graspMenu->addAction(graspContact_ExaminerAction);
        dbaseMenu->addAction(dbaseGUIAction);
        dbaseMenu->addAction(dbasePlannerAction);
        dbaseMenu->addAction(dbaseGraspCaptureAction);
        Sensors->addAction(sensorsSensor_InputAction);
        Sensors->addAction(sensorsBarrett_HandAction);
        Stereo->addAction(stereoOnAction);
        Stereo->addAction(stereoOffAction);
        Stereo->addAction(stereoFlip_leftrightAction);
        Misc->addAction(dynamicsArch_BuilderAction);
        Misc->addAction(miscOptimizerAction);
        Misc->addAction(miscEigengridsAction);
        Misc->addAction(actionArizona_Project);
        Misc->addAction(staubliControl);
        helpMenu->addAction(helpManualAction);
        helpMenu->addSeparator();
        helpMenu->addAction(helpAboutAction);
        helpMenu->addAction(helpAboutQTAction);

        retranslateUi(MainWindowUI);

        QMetaObject::connectSlotsByName(MainWindowUI);
    } // setupUi

    void retranslateUi(Q3MainWindow *MainWindowUI)
    {
        MainWindowUI->setWindowTitle(QApplication::translate("MainWindowUI", "GraspIt!", 0, QApplication::UnicodeUTF8));
        fileNewAction->setIconText(QApplication::translate("MainWindowUI", "New", 0, QApplication::UnicodeUTF8));
        fileNewAction->setText(QApplication::translate("MainWindowUI", "&New", 0, QApplication::UnicodeUTF8));
        fileNewAction->setShortcut(QApplication::translate("MainWindowUI", "Ctrl+N", 0, QApplication::UnicodeUTF8));
        fileOpenAction->setIconText(QApplication::translate("MainWindowUI", "Open", 0, QApplication::UnicodeUTF8));
        fileOpenAction->setText(QApplication::translate("MainWindowUI", "&Open...", 0, QApplication::UnicodeUTF8));
        fileOpenAction->setShortcut(QApplication::translate("MainWindowUI", "Ctrl+O", 0, QApplication::UnicodeUTF8));
        fileSaveAction->setIconText(QApplication::translate("MainWindowUI", "Save", 0, QApplication::UnicodeUTF8));
        fileSaveAction->setText(QApplication::translate("MainWindowUI", "&Save", 0, QApplication::UnicodeUTF8));
        fileSaveAction->setShortcut(QApplication::translate("MainWindowUI", "Ctrl+S", 0, QApplication::UnicodeUTF8));
        fileSaveAsAction->setIconText(QApplication::translate("MainWindowUI", "Save As", 0, QApplication::UnicodeUTF8));
        fileSaveAsAction->setText(QApplication::translate("MainWindowUI", "Save &As...", 0, QApplication::UnicodeUTF8));
        fileSaveAsAction->setShortcut(QString());
        fileExitAction->setIconText(QApplication::translate("MainWindowUI", "Exit", 0, QApplication::UnicodeUTF8));
        fileExitAction->setText(QApplication::translate("MainWindowUI", "E&xit", 0, QApplication::UnicodeUTF8));
        fileExitAction->setShortcut(QString());
        helpManualAction->setIconText(QApplication::translate("MainWindowUI", "Manual", 0, QApplication::UnicodeUTF8));
        helpManualAction->setText(QApplication::translate("MainWindowUI", "&Manual...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        helpManualAction->setToolTip(QApplication::translate("MainWindowUI", "Manual", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        helpManualAction->setShortcut(QString());
        helpIndexAction->setIconText(QApplication::translate("MainWindowUI", "Index", 0, QApplication::UnicodeUTF8));
        helpIndexAction->setText(QApplication::translate("MainWindowUI", "&Index...", 0, QApplication::UnicodeUTF8));
        helpIndexAction->setShortcut(QString());
        helpAboutAction->setIconText(QApplication::translate("MainWindowUI", "About", 0, QApplication::UnicodeUTF8));
        helpAboutAction->setText(QApplication::translate("MainWindowUI", "&About...", 0, QApplication::UnicodeUTF8));
        helpAboutAction->setShortcut(QString());
        fileImportRobotAction->setIconText(QApplication::translate("MainWindowUI", "Import Robot", 0, QApplication::UnicodeUTF8));
        fileImportRobotAction->setText(QApplication::translate("MainWindowUI", "Import Robot...", 0, QApplication::UnicodeUTF8));
        fileImportObjectAction->setIconText(QApplication::translate("MainWindowUI", "Import Object", 0, QApplication::UnicodeUTF8));
        fileImportObjectAction->setText(QApplication::translate("MainWindowUI", "Import Object...", 0, QApplication::UnicodeUTF8));
        fileImportObstacleAction->setIconText(QApplication::translate("MainWindowUI", "Import Obstacle", 0, QApplication::UnicodeUTF8));
        fileImportObstacleAction->setText(QApplication::translate("MainWindowUI", "Import Obstacle...", 0, QApplication::UnicodeUTF8));
        fileSaveImageAction->setIconText(QApplication::translate("MainWindowUI", "Save Image", 0, QApplication::UnicodeUTF8));
        fileSaveImageAction->setText(QApplication::translate("MainWindowUI", "Save Image...", 0, QApplication::UnicodeUTF8));
        graspCreateProjectionAction->setIconText(QApplication::translate("MainWindowUI", "Create GWS Projection", 0, QApplication::UnicodeUTF8));
        graspCreateProjectionAction->setText(QApplication::translate("MainWindowUI", "&Create GWS Projection...", 0, QApplication::UnicodeUTF8));
        graspQualityMeasuresAction->setIconText(QApplication::translate("MainWindowUI", "Quality Measures", 0, QApplication::UnicodeUTF8));
        graspQualityMeasuresAction->setText(QApplication::translate("MainWindowUI", "&Quality Measures...", 0, QApplication::UnicodeUTF8));
        fileEditSettingsAction->setIconText(QApplication::translate("MainWindowUI", "Edit Settings", 0, QApplication::UnicodeUTF8));
        fileEditSettingsAction->setText(QApplication::translate("MainWindowUI", "&Edit Settings...", 0, QApplication::UnicodeUTF8));
        graspPlannerAction->setIconText(QApplication::translate("MainWindowUI", "Planner...", 0, QApplication::UnicodeUTF8));
        graspPlannerAction->setText(QApplication::translate("MainWindowUI", "&Planner...", 0, QApplication::UnicodeUTF8));
        miscOptimizerAction->setIconText(QApplication::translate("MainWindowUI", "Optimizer...", 0, QApplication::UnicodeUTF8));
        miscOptimizerAction->setText(QApplication::translate("MainWindowUI", "Optimizer...", 0, QApplication::UnicodeUTF8));
        actionArizona_Project->setIconText(QApplication::translate("MainWindowUI", "Arizona Project...", 0, QApplication::UnicodeUTF8));
        actionArizona_Project->setText(QApplication::translate("MainWindowUI", "Arizona Project...", 0, QApplication::UnicodeUTF8));
        staubliControl->setIconText(QApplication::translate("MainWindowUI", "Staubli Control Panel...", 0, QApplication::UnicodeUTF8));
        staubliControl->setText(QApplication::translate("MainWindowUI", "Staubli Control Panel...", 0, QApplication::UnicodeUTF8));
        graspGFOAction->setIconText(QApplication::translate("MainWindowUI", "Grasp Force Optimization...", 0, QApplication::UnicodeUTF8));
        graspGFOAction->setText(QApplication::translate("MainWindowUI", "Grasp Force Optimization...", 0, QApplication::UnicodeUTF8));
        graspCompliantPlannerAction->setIconText(QApplication::translate("MainWindowUI", "Compliant Planner...", 0, QApplication::UnicodeUTF8));
        graspCompliantPlannerAction->setText(QApplication::translate("MainWindowUI", "Compliant Planner...", 0, QApplication::UnicodeUTF8));
        dynamicsPlayAction->setIconText(QApplication::translate("MainWindowUI", "Start Simulation", 0, QApplication::UnicodeUTF8));
        dynamicsPlayAction->setText(QApplication::translate("MainWindowUI", "Start Simulation", 0, QApplication::UnicodeUTF8));
        dynamicsPopAction->setIconText(QApplication::translate("MainWindowUI", "Pop State", 0, QApplication::UnicodeUTF8));
        dynamicsPopAction->setText(QApplication::translate("MainWindowUI", "Pop State", 0, QApplication::UnicodeUTF8));
        dynamicsPushAction->setIconText(QApplication::translate("MainWindowUI", "Push State", 0, QApplication::UnicodeUTF8));
        dynamicsPushAction->setText(QApplication::translate("MainWindowUI", "Push State", 0, QApplication::UnicodeUTF8));
        elementCollisionToggleAction->setIconText(QApplication::translate("MainWindowUI", "Collisions ON", 0, QApplication::UnicodeUTF8));
        elementCollisionToggleAction->setText(QApplication::translate("MainWindowUI", "Collisions ON", 0, QApplication::UnicodeUTF8));
        elementBodyPropertiesAction->setIconText(QApplication::translate("MainWindowUI", "Body Properties...", 0, QApplication::UnicodeUTF8));
        elementBodyPropertiesAction->setText(QApplication::translate("MainWindowUI", "Body Properties...", 0, QApplication::UnicodeUTF8));
        graspAutoGraspAction->setIconText(QApplication::translate("MainWindowUI", "Auto Grasp", 0, QApplication::UnicodeUTF8));
        graspAutoGraspAction->setText(QApplication::translate("MainWindowUI", "Auto &Grasp", 0, QApplication::UnicodeUTF8));
        graspAutoGraspAction->setShortcut(QApplication::translate("MainWindowUI", "Ctrl+G", 0, QApplication::UnicodeUTF8));
        helpAboutQTAction->setIconText(QApplication::translate("MainWindowUI", "About QT", 0, QApplication::UnicodeUTF8));
        helpAboutQTAction->setText(QApplication::translate("MainWindowUI", "About &QT...", 0, QApplication::UnicodeUTF8));
        fileImport_Human_HandAction->setIconText(QApplication::translate("MainWindowUI", "Import Human Hand...", 0, QApplication::UnicodeUTF8));
        fileImport_Human_HandAction->setText(QApplication::translate("MainWindowUI", "Import Human Hand...", 0, QApplication::UnicodeUTF8));
        dynamicsArch_BuilderAction->setIconText(QApplication::translate("MainWindowUI", "Arch Builder...", 0, QApplication::UnicodeUTF8));
        dynamicsArch_BuilderAction->setText(QApplication::translate("MainWindowUI", "Arch Builder...", 0, QApplication::UnicodeUTF8));
        sensorsSensor_InputAction->setIconText(QApplication::translate("MainWindowUI", "Sensor Input...", 0, QApplication::UnicodeUTF8));
        sensorsSensor_InputAction->setText(QApplication::translate("MainWindowUI", "Sensor Input...", 0, QApplication::UnicodeUTF8));
        stereoOnAction->setIconText(QApplication::translate("MainWindowUI", "On", 0, QApplication::UnicodeUTF8));
        stereoOnAction->setText(QApplication::translate("MainWindowUI", "On", 0, QApplication::UnicodeUTF8));
        stereoOffAction->setIconText(QApplication::translate("MainWindowUI", "Off", 0, QApplication::UnicodeUTF8));
        stereoOffAction->setText(QApplication::translate("MainWindowUI", "Off", 0, QApplication::UnicodeUTF8));
        stereoFlip_leftrightAction->setIconText(QApplication::translate("MainWindowUI", "Flip Left/Right", 0, QApplication::UnicodeUTF8));
        stereoFlip_leftrightAction->setText(QApplication::translate("MainWindowUI", "Flip Left/Right", 0, QApplication::UnicodeUTF8));
        dbaseGraspCaptureAction->setIconText(QApplication::translate("MainWindowUI", "Grasp Capture...", 0, QApplication::UnicodeUTF8));
        dbaseGraspCaptureAction->setText(QApplication::translate("MainWindowUI", "Grasp Capture...", 0, QApplication::UnicodeUTF8));
        graspEigenGrasp_InterfaceAction->setIconText(QApplication::translate("MainWindowUI", "EigenGrasp Interface...", 0, QApplication::UnicodeUTF8));
        graspEigenGrasp_InterfaceAction->setText(QApplication::translate("MainWindowUI", "EigenGrasp Interface...", 0, QApplication::UnicodeUTF8));
        graspContact_ExaminerAction->setIconText(QApplication::translate("MainWindowUI", "Virtual Contacts...", 0, QApplication::UnicodeUTF8));
        graspContact_ExaminerAction->setText(QApplication::translate("MainWindowUI", "Virtual Contacts...", 0, QApplication::UnicodeUTF8));
        graspEigenGrasp_PlannerAction->setIconText(QApplication::translate("MainWindowUI", "EigenGrasp Planner...", 0, QApplication::UnicodeUTF8));
        graspEigenGrasp_PlannerAction->setText(QApplication::translate("MainWindowUI", "EigenGrasp Planner...", 0, QApplication::UnicodeUTF8));
        sensorsBarrett_HandAction->setIconText(QApplication::translate("MainWindowUI", "Barrett Hand...", 0, QApplication::UnicodeUTF8));
        sensorsBarrett_HandAction->setText(QApplication::translate("MainWindowUI", "Barrett Hand...", 0, QApplication::UnicodeUTF8));
        graspAuto_OpenAction->setIconText(QApplication::translate("MainWindowUI", "Auto Open", 0, QApplication::UnicodeUTF8));
        graspAuto_OpenAction->setText(QApplication::translate("MainWindowUI", "Auto Open", 0, QApplication::UnicodeUTF8));
        elementPrimitivesAction->setIconText(QApplication::translate("MainWindowUI", "Primitives...", 0, QApplication::UnicodeUTF8));
        elementPrimitivesAction->setText(QApplication::translate("MainWindowUI", "Primitives...", 0, QApplication::UnicodeUTF8));
        miscEigengridsAction->setIconText(QApplication::translate("MainWindowUI", "Eigengrids...", 0, QApplication::UnicodeUTF8));
        miscEigengridsAction->setText(QApplication::translate("MainWindowUI", "Eigengrids...", 0, QApplication::UnicodeUTF8));
        dbaseGUIAction->setIconText(QApplication::translate("MainWindowUI", "Connect and Browse...", 0, QApplication::UnicodeUTF8));
        dbaseGUIAction->setText(QApplication::translate("MainWindowUI", "Connect and Browse...", 0, QApplication::UnicodeUTF8));
        dbasePlannerAction->setIconText(QApplication::translate("MainWindowUI", "Database Planner...", 0, QApplication::UnicodeUTF8));
        dbasePlannerAction->setText(QApplication::translate("MainWindowUI", "Database Planner...", 0, QApplication::UnicodeUTF8));
        translateToolAction->setIconText(QApplication::translate("MainWindowUI", "Translate", 0, QApplication::UnicodeUTF8));
        translateToolAction->setText(QApplication::translate("MainWindowUI", "Translate", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        translateToolAction->setToolTip(QApplication::translate("MainWindowUI", "Translate", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        translateToolAction->setShortcut(QApplication::translate("MainWindowUI", "F2", 0, QApplication::UnicodeUTF8));
        rotateToolAction->setIconText(QApplication::translate("MainWindowUI", "Rotate", 0, QApplication::UnicodeUTF8));
        rotateToolAction->setText(QApplication::translate("MainWindowUI", "Rotate", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        rotateToolAction->setToolTip(QApplication::translate("MainWindowUI", "Rotate", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        rotateToolAction->setShortcut(QApplication::translate("MainWindowUI", "F3", 0, QApplication::UnicodeUTF8));
        selectToolAction->setIconText(QApplication::translate("MainWindowUI", "Select", 0, QApplication::UnicodeUTF8));
        selectToolAction->setText(QApplication::translate("MainWindowUI", "Select", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        selectToolAction->setToolTip(QApplication::translate("MainWindowUI", "Select", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        selectToolAction->setShortcut(QApplication::translate("MainWindowUI", "F4", 0, QApplication::UnicodeUTF8));
        worldBox->setTitle(QApplication::translate("MainWindowUI", "Untitled", 0, QApplication::UnicodeUTF8));
        qualityGroupBox->setTitle(QApplication::translate("MainWindowUI", "Grasp Quality", 0, QApplication::UnicodeUTF8));
        qualityListBox->clear();
        qualityListBox->insertItem(QApplication::translate("MainWindowUI", "Use grasp menu to add quality measures", 0, QApplication::UnicodeUTF8));
        contactsGroupBox->setTitle(QApplication::translate("MainWindowUI", "Contacts", 0, QApplication::UnicodeUTF8));
        toolBar->setLabel(QApplication::translate("MainWindowUI", "File", 0, QApplication::UnicodeUTF8));
        Toolbar_2->setLabel(QApplication::translate("MainWindowUI", "Body Tools", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        materialComboBox->setProperty("toolTip", QVariant(QApplication::translate("MainWindowUI", "Material", 0, QApplication::UnicodeUTF8)));
#endif // QT_NO_TOOLTIP
        ToolbarDynamics->setLabel(QApplication::translate("MainWindowUI", "Dynamics", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        timeReadout->setProperty("toolTip", QVariant(QApplication::translate("MainWindowUI", "Simulation Time", 0, QApplication::UnicodeUTF8)));
#endif // QT_NO_TOOLTIP
        graspToolbar->setLabel(QApplication::translate("MainWindowUI", "Grasp", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        handSelectionBox->setProperty("toolTip", QVariant(QApplication::translate("MainWindowUI", "Current Hand", 0, QApplication::UnicodeUTF8)));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        graspedBodyBox->setProperty("toolTip", QVariant(QApplication::translate("MainWindowUI", "Body to grasp", 0, QApplication::UnicodeUTF8)));
#endif // QT_NO_TOOLTIP
        tendonToolbar->setLabel(QApplication::translate("MainWindowUI", "Tendon forces", 0, QApplication::UnicodeUTF8));
        Tendon_force_label->setText(QApplication::translate("MainWindowUI", "Tendon:", 0, QApplication::UnicodeUTF8));
        tendonActiveForceLabel->setText(QApplication::translate("MainWindowUI", "Active force(N):", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        TendonForceInput->setProperty("toolTip", QVariant(QApplication::translate("MainWindowUI", "Tendon force", 0, QApplication::UnicodeUTF8)));
#endif // QT_NO_TOOLTIP
        tendonPassiveForceLabel->setText(QApplication::translate("MainWindowUI", "Passive force (N):", 0, QApplication::UnicodeUTF8));
        tendonExcursionLabel->setText(QApplication::translate("MainWindowUI", "Excursion (mm):", 0, QApplication::UnicodeUTF8));
        tendonVisibleLabel->setText(QApplication::translate("MainWindowUI", "Visible:", 0, QApplication::UnicodeUTF8));
        tendonVisibleCheckBox->setText(QString());
        forcesVisibleLabel->setText(QApplication::translate("MainWindowUI", "Show forces:", 0, QApplication::UnicodeUTF8));
        forcesVisibleCheckBox->setText(QString());
        fileMenu->setTitle(QApplication::translate("MainWindowUI", "&File", 0, QApplication::UnicodeUTF8));
        elementMenu->setTitle(QApplication::translate("MainWindowUI", "&Element", 0, QApplication::UnicodeUTF8));
        graspMenu->setTitle(QApplication::translate("MainWindowUI", "&Grasp", 0, QApplication::UnicodeUTF8));
        dbaseMenu->setTitle(QApplication::translate("MainWindowUI", "Database", 0, QApplication::UnicodeUTF8));
        Sensors->setTitle(QApplication::translate("MainWindowUI", "Sensors", 0, QApplication::UnicodeUTF8));
        Stereo->setTitle(QApplication::translate("MainWindowUI", "Stereo", 0, QApplication::UnicodeUTF8));
        Misc->setTitle(QApplication::translate("MainWindowUI", "Misc.", 0, QApplication::UnicodeUTF8));
        helpMenu->setTitle(QApplication::translate("MainWindowUI", "&Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowUI: public Ui_MainWindowUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
