/********************************************************************************
** Form generated from reading UI file 'settingsDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTINGSDLG_H
#define UI_SETTINGSDLG_H

#include <Qt3Support/Q3Header>
#include <Qt3Support/Q3MimeSourceFactory>
#include <Qt3Support/Q3Table>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SettingsDlgUI
{
public:
    QWidget *Layout9;
    QHBoxLayout *hboxLayout;
    QSpacerItem *spacerItem;
    QPushButton *okButton;
    QPushButton *cancelButton;
    QTabWidget *settingsTabs;
    QWidget *cofTab;
    QVBoxLayout *vboxLayout;
    QLabel *TextLabel1;
    Q3Table *staticFrictionTable;
    QLabel *TextLabel2;
    Q3Table *kineticFrictionTable;
    QWidget *dynamicsTab;
    QVBoxLayout *vboxLayout1;
    QHBoxLayout *hboxLayout1;
    QLabel *TextLabel3;
    QLineEdit *timeStepLine;
    QSpacerItem *spacerItem1;
    QSpacerItem *spacerItem2;

    void setupUi(QDialog *SettingsDlgUI)
    {
        if (SettingsDlgUI->objectName().isEmpty())
            SettingsDlgUI->setObjectName(QString::fromUtf8("SettingsDlgUI"));
        SettingsDlgUI->resize(771, 523);
        Layout9 = new QWidget(SettingsDlgUI);
        Layout9->setObjectName(QString::fromUtf8("Layout9"));
        Layout9->setGeometry(QRect(11, 485, 749, 27));
        hboxLayout = new QHBoxLayout(Layout9);
        hboxLayout->setSpacing(6);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        spacerItem = new QSpacerItem(20, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacerItem);

        okButton = new QPushButton(Layout9);
        okButton->setObjectName(QString::fromUtf8("okButton"));
        okButton->setDefault(true);

        hboxLayout->addWidget(okButton);

        cancelButton = new QPushButton(Layout9);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        hboxLayout->addWidget(cancelButton);

        settingsTabs = new QTabWidget(SettingsDlgUI);
        settingsTabs->setObjectName(QString::fromUtf8("settingsTabs"));
        settingsTabs->setGeometry(QRect(0, 10, 771, 470));
        cofTab = new QWidget();
        cofTab->setObjectName(QString::fromUtf8("cofTab"));
        vboxLayout = new QVBoxLayout(cofTab);
        vboxLayout->setSpacing(6);
        vboxLayout->setContentsMargins(11, 11, 11, 11);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        TextLabel1 = new QLabel(cofTab);
        TextLabel1->setObjectName(QString::fromUtf8("TextLabel1"));
        TextLabel1->setWordWrap(false);

        vboxLayout->addWidget(TextLabel1);

        staticFrictionTable = new Q3Table(cofTab);
        staticFrictionTable->setObjectName(QString::fromUtf8("staticFrictionTable"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(7), static_cast<QSizePolicy::Policy>(7));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(staticFrictionTable->sizePolicy().hasHeightForWidth());
        staticFrictionTable->setSizePolicy(sizePolicy);
        staticFrictionTable->setNumRows(0);
        staticFrictionTable->setNumCols(0);

        vboxLayout->addWidget(staticFrictionTable);

        TextLabel2 = new QLabel(cofTab);
        TextLabel2->setObjectName(QString::fromUtf8("TextLabel2"));
        TextLabel2->setWordWrap(false);

        vboxLayout->addWidget(TextLabel2);

        kineticFrictionTable = new Q3Table(cofTab);
        kineticFrictionTable->setObjectName(QString::fromUtf8("kineticFrictionTable"));
        sizePolicy.setHeightForWidth(kineticFrictionTable->sizePolicy().hasHeightForWidth());
        kineticFrictionTable->setSizePolicy(sizePolicy);
        kineticFrictionTable->setNumRows(0);
        kineticFrictionTable->setNumCols(0);

        vboxLayout->addWidget(kineticFrictionTable);

        settingsTabs->addTab(cofTab, QString());
        dynamicsTab = new QWidget();
        dynamicsTab->setObjectName(QString::fromUtf8("dynamicsTab"));
        vboxLayout1 = new QVBoxLayout(dynamicsTab);
        vboxLayout1->setSpacing(6);
        vboxLayout1->setContentsMargins(11, 11, 11, 11);
        vboxLayout1->setObjectName(QString::fromUtf8("vboxLayout1"));
        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setSpacing(6);
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        TextLabel3 = new QLabel(dynamicsTab);
        TextLabel3->setObjectName(QString::fromUtf8("TextLabel3"));
        TextLabel3->setWordWrap(false);

        hboxLayout1->addWidget(TextLabel3);

        timeStepLine = new QLineEdit(dynamicsTab);
        timeStepLine->setObjectName(QString::fromUtf8("timeStepLine"));
        QSizePolicy sizePolicy1(static_cast<QSizePolicy::Policy>(0), static_cast<QSizePolicy::Policy>(0));
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(timeStepLine->sizePolicy().hasHeightForWidth());
        timeStepLine->setSizePolicy(sizePolicy1);

        hboxLayout1->addWidget(timeStepLine);

        spacerItem1 = new QSpacerItem(520, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout1->addItem(spacerItem1);


        vboxLayout1->addLayout(hboxLayout1);

        spacerItem2 = new QSpacerItem(0, 20, QSizePolicy::Minimum, QSizePolicy::Expanding);

        vboxLayout1->addItem(spacerItem2);

        settingsTabs->addTab(dynamicsTab, QString());
        QWidget::setTabOrder(settingsTabs, staticFrictionTable);
        QWidget::setTabOrder(staticFrictionTable, kineticFrictionTable);
        QWidget::setTabOrder(kineticFrictionTable, timeStepLine);
        QWidget::setTabOrder(timeStepLine, okButton);
        QWidget::setTabOrder(okButton, cancelButton);

        retranslateUi(SettingsDlgUI);
        QObject::connect(cancelButton, SIGNAL(clicked()), SettingsDlgUI, SLOT(reject()));
        QObject::connect(staticFrictionTable, SIGNAL(currentChanged(int,int)), SettingsDlgUI, SLOT(saveCurrentCOF(int,int)));
        QObject::connect(staticFrictionTable, SIGNAL(valueChanged(int,int)), SettingsDlgUI, SLOT(checkCOFEntry(int,int)));
        QObject::connect(kineticFrictionTable, SIGNAL(currentChanged(int,int)), SettingsDlgUI, SLOT(saveCurrentKCOF(int,int)));
        QObject::connect(kineticFrictionTable, SIGNAL(valueChanged(int,int)), SettingsDlgUI, SLOT(checkKCOFEntry(int,int)));
        QObject::connect(okButton, SIGNAL(clicked()), SettingsDlgUI, SLOT(validateDlg()));

        settingsTabs->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(SettingsDlgUI);
    } // setupUi

    void retranslateUi(QDialog *SettingsDlgUI)
    {
        SettingsDlgUI->setWindowTitle(QApplication::translate("SettingsDlgUI", "Settings", 0, QApplication::UnicodeUTF8));
        okButton->setText(QApplication::translate("SettingsDlgUI", "OK", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("SettingsDlgUI", "Cancel", 0, QApplication::UnicodeUTF8));
        TextLabel1->setText(QApplication::translate("SettingsDlgUI", "Coefficients of static friction", 0, QApplication::UnicodeUTF8));
        TextLabel2->setText(QApplication::translate("SettingsDlgUI", "Coefficients of kinetic friction", 0, QApplication::UnicodeUTF8));
        settingsTabs->setTabText(settingsTabs->indexOf(cofTab), QApplication::translate("SettingsDlgUI", "COF", 0, QApplication::UnicodeUTF8));
        TextLabel3->setText(QApplication::translate("SettingsDlgUI", "Default Timestep (msec):", 0, QApplication::UnicodeUTF8));
        settingsTabs->setTabText(settingsTabs->indexOf(dynamicsTab), QApplication::translate("SettingsDlgUI", "Dynamics", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SettingsDlgUI: public Ui_SettingsDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGSDLG_H
