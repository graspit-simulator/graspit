/********************************************************************************
** Form generated from reading UI file 'qmDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QMDLG_H
#define UI_QMDLG_H

#include <Qt3Support/Q3GroupBox>
#include <Qt3Support/Q3ListBox>
#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include "graspitGUI.h"
#include "ivmgr.h"
#include "quality.h"

QT_BEGIN_NAMESPACE

class Ui_QMDlgUI
{
public:
    QWidget *Layout18;
    QGridLayout *gridLayout;
    QLineEdit *qmName;
    Q3ListBox *qmListBox;
    QLabel *TextLabel2;
    Q3GroupBox *qmSettingsBox;
    QComboBox *qmTypeComboBox;
    QLabel *TextLabel1;
    Q3GroupBox *groupBox8;
    QPushButton *OKButton;
    QPushButton *AddEditButton;
    QPushButton *DeleteButton;
    QCheckBox *gravityBox;

    void setupUi(QDialog *QMDlgUI)
    {
        if (QMDlgUI->objectName().isEmpty())
            QMDlgUI->setObjectName(QString::fromUtf8("QMDlgUI"));
        QMDlgUI->resize(576, 295);
        Layout18 = new QWidget(QMDlgUI);
        Layout18->setObjectName(QString::fromUtf8("Layout18"));
        Layout18->setGeometry(QRect(11, 11, 550, 232));
        gridLayout = new QGridLayout(Layout18);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        qmName = new QLineEdit(Layout18);
        qmName->setObjectName(QString::fromUtf8("qmName"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(0), static_cast<QSizePolicy::Policy>(0));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(qmName->sizePolicy().hasHeightForWidth());
        qmName->setSizePolicy(sizePolicy);
        qmName->setMinimumSize(QSize(175, 0));

        gridLayout->addWidget(qmName, 3, 0, 1, 1);

        qmListBox = new Q3ListBox(Layout18);
        qmListBox->setObjectName(QString::fromUtf8("qmListBox"));
        QSizePolicy sizePolicy1(static_cast<QSizePolicy::Policy>(0), static_cast<QSizePolicy::Policy>(7));
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(qmListBox->sizePolicy().hasHeightForWidth());
        qmListBox->setSizePolicy(sizePolicy1);
        qmListBox->setMinimumSize(QSize(175, 0));

        gridLayout->addWidget(qmListBox, 0, 0, 2, 1);

        TextLabel2 = new QLabel(Layout18);
        TextLabel2->setObjectName(QString::fromUtf8("TextLabel2"));
        sizePolicy.setHeightForWidth(TextLabel2->sizePolicy().hasHeightForWidth());
        TextLabel2->setSizePolicy(sizePolicy);
        TextLabel2->setWordWrap(false);

        gridLayout->addWidget(TextLabel2, 0, 1, 1, 1);

        qmSettingsBox = new Q3GroupBox(Layout18);
        qmSettingsBox->setObjectName(QString::fromUtf8("qmSettingsBox"));
        QSizePolicy sizePolicy2(static_cast<QSizePolicy::Policy>(7), static_cast<QSizePolicy::Policy>(7));
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(qmSettingsBox->sizePolicy().hasHeightForWidth());
        qmSettingsBox->setSizePolicy(sizePolicy2);
        qmSettingsBox->setOrientation(Qt::Vertical);

        gridLayout->addWidget(qmSettingsBox, 1, 1, 3, 2);

        qmTypeComboBox = new QComboBox(Layout18);
        qmTypeComboBox->setObjectName(QString::fromUtf8("qmTypeComboBox"));

        gridLayout->addWidget(qmTypeComboBox, 0, 2, 1, 1);

        TextLabel1 = new QLabel(Layout18);
        TextLabel1->setObjectName(QString::fromUtf8("TextLabel1"));
        sizePolicy.setHeightForWidth(TextLabel1->sizePolicy().hasHeightForWidth());
        TextLabel1->setSizePolicy(sizePolicy);
        TextLabel1->setWordWrap(false);

        gridLayout->addWidget(TextLabel1, 2, 0, 1, 1);

        groupBox8 = new Q3GroupBox(QMDlgUI);
        groupBox8->setObjectName(QString::fromUtf8("groupBox8"));
        groupBox8->setGeometry(QRect(10, 250, 550, 40));
        groupBox8->setOrientation(Qt::Vertical);
        OKButton = new QPushButton(groupBox8);
        OKButton->setObjectName(QString::fromUtf8("OKButton"));
        OKButton->setGeometry(QRect(456, 7, 82, 26));
        OKButton->setDefault(true);
        AddEditButton = new QPushButton(groupBox8);
        AddEditButton->setObjectName(QString::fromUtf8("AddEditButton"));
        AddEditButton->setGeometry(QRect(280, 7, 82, 26));
        DeleteButton = new QPushButton(groupBox8);
        DeleteButton->setObjectName(QString::fromUtf8("DeleteButton"));
        DeleteButton->setGeometry(QRect(368, 7, 82, 26));
        gravityBox = new QCheckBox(groupBox8);
        gravityBox->setObjectName(QString::fromUtf8("gravityBox"));
        gravityBox->setGeometry(QRect(11, 10, 56, 19));

        retranslateUi(QMDlgUI);
        QObject::connect(OKButton, SIGNAL(clicked()), QMDlgUI, SLOT(accept()));
        QObject::connect(DeleteButton, SIGNAL(clicked()), QMDlgUI, SLOT(deleteQM()));
        QObject::connect(qmTypeComboBox, SIGNAL(activated(QString)), QMDlgUI, SLOT(selectQMType(QString)));
        QObject::connect(AddEditButton, SIGNAL(clicked()), QMDlgUI, SLOT(addEditQM()));
        QObject::connect(qmListBox, SIGNAL(highlighted(int)), QMDlgUI, SLOT(selectQM(int)));
        QObject::connect(gravityBox, SIGNAL(clicked()), QMDlgUI, SLOT(gravityBox_clicked()));

        QMetaObject::connectSlotsByName(QMDlgUI);
    } // setupUi

    void retranslateUi(QDialog *QMDlgUI)
    {
        QMDlgUI->setWindowTitle(QApplication::translate("QMDlgUI", "Quality Measures", 0, QApplication::UnicodeUTF8));
        qmName->setText(QApplication::translate("QMDlgUI", "New Quality Measure", 0, QApplication::UnicodeUTF8));
        TextLabel2->setText(QApplication::translate("QMDlgUI", "Quality Measure Type", 0, QApplication::UnicodeUTF8));
        qmSettingsBox->setTitle(QApplication::translate("QMDlgUI", "Settings", 0, QApplication::UnicodeUTF8));
        TextLabel1->setText(QApplication::translate("QMDlgUI", "Name", 0, QApplication::UnicodeUTF8));
        groupBox8->setTitle(QString());
        OKButton->setText(QApplication::translate("QMDlgUI", "OK", 0, QApplication::UnicodeUTF8));
        AddEditButton->setText(QApplication::translate("QMDlgUI", "Add / Edit", 0, QApplication::UnicodeUTF8));
        DeleteButton->setText(QApplication::translate("QMDlgUI", "Delete", 0, QApplication::UnicodeUTF8));
        gravityBox->setText(QApplication::translate("QMDlgUI", "Gravity", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class QMDlgUI: public Ui_QMDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QMDLG_H
