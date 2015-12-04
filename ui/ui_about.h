/********************************************************************************
** Form generated from reading UI file 'about.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUT_H
#define UI_ABOUT_H

#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_AboutDlg
{
public:
    QVBoxLayout *vboxLayout;
    QHBoxLayout *hboxLayout;
    QSpacerItem *spacer5;
    QLabel *pixmapLabel1;
    QSpacerItem *spacer4;
    QLabel *versionLabel;
    QLabel *copyrightLabel;
    QSpacerItem *spacer3;
    QHBoxLayout *hboxLayout1;
    QSpacerItem *spacer2;
    QPushButton *okButton;
    QSpacerItem *spacer1;

    void setupUi(QDialog *AboutDlg)
    {
        if (AboutDlg->objectName().isEmpty())
            AboutDlg->setObjectName(QString::fromUtf8("AboutDlg"));
        AboutDlg->resize(375, 250);
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(5));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(AboutDlg->sizePolicy().hasHeightForWidth());
        AboutDlg->setSizePolicy(sizePolicy);
        vboxLayout = new QVBoxLayout(AboutDlg);
        vboxLayout->setSpacing(6);
        vboxLayout->setContentsMargins(11, 11, 11, 11);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        hboxLayout = new QHBoxLayout();
        hboxLayout->setSpacing(6);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        spacer5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacer5);

        pixmapLabel1 = new QLabel(AboutDlg);
        pixmapLabel1->setObjectName(QString::fromUtf8("pixmapLabel1"));
        pixmapLabel1->setPixmap(QPixmap(qPixmapFromMimeSource("logo.png")));
        pixmapLabel1->setScaledContents(true);
        pixmapLabel1->setWordWrap(false);

        hboxLayout->addWidget(pixmapLabel1);

        spacer4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacer4);


        vboxLayout->addLayout(hboxLayout);

        versionLabel = new QLabel(AboutDlg);
        versionLabel->setObjectName(QString::fromUtf8("versionLabel"));
        QSizePolicy sizePolicy1(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(4));
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(versionLabel->sizePolicy().hasHeightForWidth());
        versionLabel->setSizePolicy(sizePolicy1);
        versionLabel->setAlignment(Qt::AlignCenter);
        versionLabel->setWordWrap(false);

        vboxLayout->addWidget(versionLabel);

        copyrightLabel = new QLabel(AboutDlg);
        copyrightLabel->setObjectName(QString::fromUtf8("copyrightLabel"));
        sizePolicy1.setHeightForWidth(copyrightLabel->sizePolicy().hasHeightForWidth());
        copyrightLabel->setSizePolicy(sizePolicy1);
        copyrightLabel->setAlignment(Qt::AlignCenter);
        copyrightLabel->setWordWrap(false);

        vboxLayout->addWidget(copyrightLabel);

        spacer3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        vboxLayout->addItem(spacer3);

        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setSpacing(6);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        spacer2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout1->addItem(spacer2);

        okButton = new QPushButton(AboutDlg);
        okButton->setObjectName(QString::fromUtf8("okButton"));
        okButton->setDefault(true);

        hboxLayout1->addWidget(okButton);

        spacer1 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout1->addItem(spacer1);


        vboxLayout->addLayout(hboxLayout1);


        retranslateUi(AboutDlg);
        QObject::connect(okButton, SIGNAL(clicked()), AboutDlg, SLOT(accept()));

        QMetaObject::connectSlotsByName(AboutDlg);
    } // setupUi

    void retranslateUi(QDialog *AboutDlg)
    {
        AboutDlg->setWindowTitle(QApplication::translate("AboutDlg", "GraspIt!", 0, QApplication::UnicodeUTF8));
        versionLabel->setText(QApplication::translate("AboutDlg", "Version ", 0, QApplication::UnicodeUTF8));
        copyrightLabel->setText(QApplication::translate("AboutDlg", "Copyright (C) 2002 - 2009  Columbia University in the City of New York.  All rights reserved.", 0, QApplication::UnicodeUTF8));
        okButton->setText(QApplication::translate("AboutDlg", "OK", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class AboutDlg: public Ui_AboutDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUT_H
