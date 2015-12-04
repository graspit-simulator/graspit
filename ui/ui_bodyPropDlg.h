/********************************************************************************
** Form generated from reading UI file 'bodyPropDlg.ui'
**
** Created: Fri Dec 4 12:31:48 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BODYPROPDLG_H
#define UI_BODYPROPDLG_H

#include <Qt3Support/Q3MimeSourceFactory>
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
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>
#include <vector>

QT_BEGIN_NAMESPACE

class Ui_BodyPropDlgUI
{
public:
    QComboBox *materialComboBox;
    QLabel *TextLabel4;
    QLabel *TextLabel2;
    QCheckBox *axesCheckBox;
    QCheckBox *dynamicForcesCheckBox;
    QLineEdit *massLineEdit;
    QLabel *TextLabel1;
    QWidget *widget;
    QHBoxLayout *hboxLayout;
    QSpacerItem *spacerItem;
    QPushButton *OKButton;
    QPushButton *cancelButton;
    QLabel *TextLabel3_2;
    QLabel *TextLabel3;
    QWidget *widget1;
    QHBoxLayout *hboxLayout1;
    QFrame *line2;
    QCheckBox *dynamicCheckBox;
    QFrame *line1;
    QCheckBox *fcCheckBox;
    QSlider *transparencySlider;
    QCheckBox *boundingCheckBox;
    QSpinBox *boundingSpinBox;

    void setupUi(QDialog *BodyPropDlgUI)
    {
        if (BodyPropDlgUI->objectName().isEmpty())
            BodyPropDlgUI->setObjectName(QString::fromUtf8("BodyPropDlgUI"));
        BodyPropDlgUI->resize(345, 299);
        materialComboBox = new QComboBox(BodyPropDlgUI);
        materialComboBox->setObjectName(QString::fromUtf8("materialComboBox"));
        materialComboBox->setGeometry(QRect(55, 11, 156, 18));
        TextLabel4 = new QLabel(BodyPropDlgUI);
        TextLabel4->setObjectName(QString::fromUtf8("TextLabel4"));
        TextLabel4->setGeometry(QRect(11, 11, 38, 18));
        TextLabel4->setWordWrap(false);
        TextLabel2 = new QLabel(BodyPropDlgUI);
        TextLabel2->setObjectName(QString::fromUtf8("TextLabel2"));
        TextLabel2->setGeometry(QRect(11, 59, 77, 43));
        TextLabel2->setWordWrap(false);
        axesCheckBox = new QCheckBox(BodyPropDlgUI);
        axesCheckBox->setObjectName(QString::fromUtf8("axesCheckBox"));
        axesCheckBox->setGeometry(QRect(10, 180, 77, 18));
        dynamicForcesCheckBox = new QCheckBox(BodyPropDlgUI);
        dynamicForcesCheckBox->setObjectName(QString::fromUtf8("dynamicForcesCheckBox"));
        dynamicForcesCheckBox->setGeometry(QRect(10, 210, 323, 18));
        massLineEdit = new QLineEdit(BodyPropDlgUI);
        massLineEdit->setObjectName(QString::fromUtf8("massLineEdit"));
        massLineEdit->setGeometry(QRect(90, 230, 117, 18));
        TextLabel1 = new QLabel(BodyPropDlgUI);
        TextLabel1->setObjectName(QString::fromUtf8("TextLabel1"));
        TextLabel1->setGeometry(QRect(10, 230, 77, 20));
        TextLabel1->setWordWrap(false);
        widget = new QWidget(BodyPropDlgUI);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(10, 260, 323, 27));
        hboxLayout = new QHBoxLayout(widget);
        hboxLayout->setSpacing(6);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        spacerItem = new QSpacerItem(310, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacerItem);

        OKButton = new QPushButton(widget);
        OKButton->setObjectName(QString::fromUtf8("OKButton"));

        hboxLayout->addWidget(OKButton);

        cancelButton = new QPushButton(widget);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));
        cancelButton->setAutoDefault(false);

        hboxLayout->addWidget(cancelButton);

        TextLabel3_2 = new QLabel(BodyPropDlgUI);
        TextLabel3_2->setObjectName(QString::fromUtf8("TextLabel3_2"));
        TextLabel3_2->setGeometry(QRect(217, 60, 116, 16));
        QFont font;
        font.setFamily(QString::fromUtf8("MS Shell Dlg 2"));
        font.setPointSize(8);
        font.setBold(false);
        font.setItalic(false);
        font.setUnderline(false);
        font.setWeight(50);
        font.setStrikeOut(false);
        TextLabel3_2->setFont(font);
        TextLabel3_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        TextLabel3_2->setWordWrap(false);
        TextLabel3 = new QLabel(BodyPropDlgUI);
        TextLabel3->setObjectName(QString::fromUtf8("TextLabel3"));
        TextLabel3->setGeometry(QRect(95, 60, 116, 16));
        TextLabel3->setFont(font);
        TextLabel3->setWordWrap(false);
        widget1 = new QWidget(BodyPropDlgUI);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(10, 140, 323, 42));
        hboxLayout1 = new QHBoxLayout(widget1);
        hboxLayout1->setSpacing(6);
        hboxLayout1->setContentsMargins(11, 11, 11, 11);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        line2 = new QFrame(widget1);
        line2->setObjectName(QString::fromUtf8("line2"));
        line2->setFrameShape(QFrame::HLine);
        line2->setFrameShadow(QFrame::Sunken);
        line2->setFrameShape(QFrame::HLine);

        hboxLayout1->addWidget(line2);

        dynamicCheckBox = new QCheckBox(widget1);
        dynamicCheckBox->setObjectName(QString::fromUtf8("dynamicCheckBox"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(0), static_cast<QSizePolicy::Policy>(0));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dynamicCheckBox->sizePolicy().hasHeightForWidth());
        dynamicCheckBox->setSizePolicy(sizePolicy);

        hboxLayout1->addWidget(dynamicCheckBox);

        line1 = new QFrame(widget1);
        line1->setObjectName(QString::fromUtf8("line1"));
        line1->setFrameShape(QFrame::HLine);
        line1->setFrameShadow(QFrame::Sunken);
        line1->setFrameShape(QFrame::HLine);

        hboxLayout1->addWidget(line1);

        fcCheckBox = new QCheckBox(BodyPropDlgUI);
        fcCheckBox->setObjectName(QString::fromUtf8("fcCheckBox"));
        fcCheckBox->setGeometry(QRect(11, 35, 323, 18));
        transparencySlider = new QSlider(BodyPropDlgUI);
        transparencySlider->setObjectName(QString::fromUtf8("transparencySlider"));
        transparencySlider->setGeometry(QRect(95, 80, 238, 21));
        transparencySlider->setMaximum(100);
        transparencySlider->setOrientation(Qt::Horizontal);
        boundingCheckBox = new QCheckBox(BodyPropDlgUI);
        boundingCheckBox->setObjectName(QString::fromUtf8("boundingCheckBox"));
        boundingCheckBox->setGeometry(QRect(10, 110, 181, 18));
        boundingSpinBox = new QSpinBox(BodyPropDlgUI);
        boundingSpinBox->setObjectName(QString::fromUtf8("boundingSpinBox"));
        boundingSpinBox->setGeometry(QRect(196, 108, 44, 22));

        retranslateUi(BodyPropDlgUI);
        QObject::connect(OKButton, SIGNAL(clicked()), BodyPropDlgUI, SLOT(accept()));
        QObject::connect(cancelButton, SIGNAL(clicked()), BodyPropDlgUI, SLOT(revertAndClose()));
        QObject::connect(transparencySlider, SIGNAL(valueChanged(int)), BodyPropDlgUI, SLOT(setTransparency(int)));
        QObject::connect(axesCheckBox, SIGNAL(stateChanged(int)), BodyPropDlgUI, SLOT(setShowAxes(int)));
        QObject::connect(fcCheckBox, SIGNAL(stateChanged(int)), BodyPropDlgUI, SLOT(setShowFC(int)));
        QObject::connect(dynamicForcesCheckBox, SIGNAL(stateChanged(int)), BodyPropDlgUI, SLOT(setShowDynContactForces(int)));
        QObject::connect(dynamicCheckBox, SIGNAL(stateChanged(int)), BodyPropDlgUI, SLOT(setDynamic(int)));
        QObject::connect(materialComboBox, SIGNAL(activated(int)), BodyPropDlgUI, SLOT(setMaterial(int)));

        QMetaObject::connectSlotsByName(BodyPropDlgUI);
    } // setupUi

    void retranslateUi(QDialog *BodyPropDlgUI)
    {
        BodyPropDlgUI->setWindowTitle(QApplication::translate("BodyPropDlgUI", "Body Properties", 0, QApplication::UnicodeUTF8));
        TextLabel4->setText(QApplication::translate("BodyPropDlgUI", "Material", 0, QApplication::UnicodeUTF8));
        TextLabel2->setText(QApplication::translate("BodyPropDlgUI", "Transparency", 0, QApplication::UnicodeUTF8));
        axesCheckBox->setText(QApplication::translate("BodyPropDlgUI", "Show Axes", 0, QApplication::UnicodeUTF8));
        dynamicForcesCheckBox->setText(QApplication::translate("BodyPropDlgUI", "Show Dynamic Contact Forces", 0, QApplication::UnicodeUTF8));
        TextLabel1->setText(QApplication::translate("BodyPropDlgUI", "Mass (g)", 0, QApplication::UnicodeUTF8));
        OKButton->setText(QApplication::translate("BodyPropDlgUI", "OK", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("BodyPropDlgUI", "Cancel", 0, QApplication::UnicodeUTF8));
        TextLabel3_2->setText(QApplication::translate("BodyPropDlgUI", "Transparent", 0, QApplication::UnicodeUTF8));
        TextLabel3->setText(QApplication::translate("BodyPropDlgUI", "Opaque", 0, QApplication::UnicodeUTF8));
        dynamicCheckBox->setText(QApplication::translate("BodyPropDlgUI", "Dynamic", 0, QApplication::UnicodeUTF8));
        fcCheckBox->setText(QApplication::translate("BodyPropDlgUI", "Show Friction Cones", 0, QApplication::UnicodeUTF8));
        boundingCheckBox->setText(QApplication::translate("BodyPropDlgUI", "Show bounding volumes at depth", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class BodyPropDlgUI: public Ui_BodyPropDlgUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BODYPROPDLG_H
