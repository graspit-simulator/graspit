/********************************************************************************
** Form generated from reading UI file 'gwsProjDlgBase.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GWSPROJDLGBASE_H
#define UI_GWSPROJDLGBASE_H

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

QT_BEGIN_NAMESPACE

class Ui_GWSProjDlgBase
{
public:
    QVBoxLayout *vboxLayout;
    QHBoxLayout *hboxLayout;
    QLabel *TextLabel1;
    QComboBox *gwsTypeComboBox;
    QGridLayout *gridLayout;
    QCheckBox *fyCheckBox;
    QHBoxLayout *hboxLayout1;
    QLineEdit *fxCoord;
    QLineEdit *fyCoord;
    QLineEdit *fzCoord;
    QLineEdit *txCoord;
    QLineEdit *tyCoord;
    QLineEdit *tzCoord;
    QCheckBox *tzCheckBox;
    QCheckBox *fxCheckBox;
    QCheckBox *txCheckBox;
    QCheckBox *fzCheckBox;
    QCheckBox *tyCheckBox;
    QLabel *TextLabel2;
    QHBoxLayout *hboxLayout2;
    QSpacerItem *Spacer1;
    QPushButton *OKButton;
    QPushButton *CancelButton;

    void setupUi(QDialog *GWSProjDlgBase)
    {
        if (GWSProjDlgBase->objectName().isEmpty())
            GWSProjDlgBase->setObjectName(QString::fromUtf8("GWSProjDlgBase"));
        GWSProjDlgBase->resize(419, 179);
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(5));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(GWSProjDlgBase->sizePolicy().hasHeightForWidth());
        GWSProjDlgBase->setSizePolicy(sizePolicy);
        vboxLayout = new QVBoxLayout(GWSProjDlgBase);
        vboxLayout->setSpacing(6);
        vboxLayout->setContentsMargins(11, 11, 11, 11);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        hboxLayout = new QHBoxLayout();
        hboxLayout->setSpacing(6);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        TextLabel1 = new QLabel(GWSProjDlgBase);
        TextLabel1->setObjectName(QString::fromUtf8("TextLabel1"));
        TextLabel1->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
        TextLabel1->setWordWrap(false);

        hboxLayout->addWidget(TextLabel1);

        gwsTypeComboBox = new QComboBox(GWSProjDlgBase);
        gwsTypeComboBox->setObjectName(QString::fromUtf8("gwsTypeComboBox"));

        hboxLayout->addWidget(gwsTypeComboBox);


        vboxLayout->addLayout(hboxLayout);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        fyCheckBox = new QCheckBox(GWSProjDlgBase);
        fyCheckBox->setObjectName(QString::fromUtf8("fyCheckBox"));

        gridLayout->addWidget(fyCheckBox, 1, 1, 1, 1);

        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setSpacing(6);
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        fxCoord = new QLineEdit(GWSProjDlgBase);
        fxCoord->setObjectName(QString::fromUtf8("fxCoord"));

        hboxLayout1->addWidget(fxCoord);

        fyCoord = new QLineEdit(GWSProjDlgBase);
        fyCoord->setObjectName(QString::fromUtf8("fyCoord"));

        hboxLayout1->addWidget(fyCoord);

        fzCoord = new QLineEdit(GWSProjDlgBase);
        fzCoord->setObjectName(QString::fromUtf8("fzCoord"));

        hboxLayout1->addWidget(fzCoord);

        txCoord = new QLineEdit(GWSProjDlgBase);
        txCoord->setObjectName(QString::fromUtf8("txCoord"));

        hboxLayout1->addWidget(txCoord);

        tyCoord = new QLineEdit(GWSProjDlgBase);
        tyCoord->setObjectName(QString::fromUtf8("tyCoord"));

        hboxLayout1->addWidget(tyCoord);

        tzCoord = new QLineEdit(GWSProjDlgBase);
        tzCoord->setObjectName(QString::fromUtf8("tzCoord"));

        hboxLayout1->addWidget(tzCoord);


        gridLayout->addLayout(hboxLayout1, 0, 0, 1, 6);

        tzCheckBox = new QCheckBox(GWSProjDlgBase);
        tzCheckBox->setObjectName(QString::fromUtf8("tzCheckBox"));

        gridLayout->addWidget(tzCheckBox, 1, 5, 1, 1);

        fxCheckBox = new QCheckBox(GWSProjDlgBase);
        fxCheckBox->setObjectName(QString::fromUtf8("fxCheckBox"));

        gridLayout->addWidget(fxCheckBox, 1, 0, 1, 1);

        txCheckBox = new QCheckBox(GWSProjDlgBase);
        txCheckBox->setObjectName(QString::fromUtf8("txCheckBox"));

        gridLayout->addWidget(txCheckBox, 1, 3, 1, 1);

        fzCheckBox = new QCheckBox(GWSProjDlgBase);
        fzCheckBox->setObjectName(QString::fromUtf8("fzCheckBox"));

        gridLayout->addWidget(fzCheckBox, 1, 2, 1, 1);

        tyCheckBox = new QCheckBox(GWSProjDlgBase);
        tyCheckBox->setObjectName(QString::fromUtf8("tyCheckBox"));

        gridLayout->addWidget(tyCheckBox, 1, 4, 1, 1);


        vboxLayout->addLayout(gridLayout);

        TextLabel2 = new QLabel(GWSProjDlgBase);
        TextLabel2->setObjectName(QString::fromUtf8("TextLabel2"));
        TextLabel2->setAlignment(Qt::AlignCenter);
        TextLabel2->setWordWrap(false);

        vboxLayout->addWidget(TextLabel2);

        hboxLayout2 = new QHBoxLayout();
        hboxLayout2->setSpacing(6);
        hboxLayout2->setContentsMargins(0, 0, 0, 0);
        hboxLayout2->setObjectName(QString::fromUtf8("hboxLayout2"));
        Spacer1 = new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout2->addItem(Spacer1);

        OKButton = new QPushButton(GWSProjDlgBase);
        OKButton->setObjectName(QString::fromUtf8("OKButton"));

        hboxLayout2->addWidget(OKButton);

        CancelButton = new QPushButton(GWSProjDlgBase);
        CancelButton->setObjectName(QString::fromUtf8("CancelButton"));
        CancelButton->setAutoDefault(false);

        hboxLayout2->addWidget(CancelButton);


        vboxLayout->addLayout(hboxLayout2);


        retranslateUi(GWSProjDlgBase);
        QObject::connect(OKButton, SIGNAL(clicked()), GWSProjDlgBase, SLOT(accept()));
        QObject::connect(CancelButton, SIGNAL(clicked()), GWSProjDlgBase, SLOT(reject()));

        QMetaObject::connectSlotsByName(GWSProjDlgBase);
    } // setupUi

    void retranslateUi(QDialog *GWSProjDlgBase)
    {
        GWSProjDlgBase->setWindowTitle(QApplication::translate("GWSProjDlgBase", "Create GWS Projection", 0, QApplication::UnicodeUTF8));
        TextLabel1->setText(QApplication::translate("GWSProjDlgBase", "Limit unit GWS using", 0, QApplication::UnicodeUTF8));
        fyCheckBox->setText(QApplication::translate("GWSProjDlgBase", "fy", 0, QApplication::UnicodeUTF8));
        fxCoord->setText(QApplication::translate("GWSProjDlgBase", "0.00", 0, QApplication::UnicodeUTF8));
        fyCoord->setText(QApplication::translate("GWSProjDlgBase", "0.00", 0, QApplication::UnicodeUTF8));
        fzCoord->setText(QApplication::translate("GWSProjDlgBase", "0.00", 0, QApplication::UnicodeUTF8));
        txCoord->setText(QApplication::translate("GWSProjDlgBase", "0.00", 0, QApplication::UnicodeUTF8));
        tyCoord->setText(QApplication::translate("GWSProjDlgBase", "0.00", 0, QApplication::UnicodeUTF8));
        tzCoord->setText(QApplication::translate("GWSProjDlgBase", "0.00", 0, QApplication::UnicodeUTF8));
        tzCheckBox->setText(QApplication::translate("GWSProjDlgBase", "tz", 0, QApplication::UnicodeUTF8));
        fxCheckBox->setText(QApplication::translate("GWSProjDlgBase", "fx", 0, QApplication::UnicodeUTF8));
        txCheckBox->setText(QApplication::translate("GWSProjDlgBase", "tx", 0, QApplication::UnicodeUTF8));
        fzCheckBox->setText(QApplication::translate("GWSProjDlgBase", "fz", 0, QApplication::UnicodeUTF8));
        tyCheckBox->setText(QApplication::translate("GWSProjDlgBase", "ty", 0, QApplication::UnicodeUTF8));
        TextLabel2->setText(QApplication::translate("GWSProjDlgBase", "Select which 3 values are fixed", 0, QApplication::UnicodeUTF8));
        OKButton->setText(QApplication::translate("GWSProjDlgBase", "OK", 0, QApplication::UnicodeUTF8));
        CancelButton->setText(QApplication::translate("GWSProjDlgBase", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GWSProjDlgBase: public Ui_GWSProjDlgBase {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GWSPROJDLGBASE_H
