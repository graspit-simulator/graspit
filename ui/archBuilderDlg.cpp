#include "archBuilderDlg.h"

#include <QValidator>

void ArchBuilderDlg::init()	{
	innerRadiusEdit->setValidator(new QDoubleValidator(0,1.0e+6,6,this));
	outerRadiusEdit->setValidator(new QDoubleValidator(0,1.0e+6,6,this));
	thicknessEdit->setValidator(new QDoubleValidator(0,1.0e+6,6,this));
	QString val;
	innerRadiusEdit->setText( val.setNum(900) );
	outerRadiusEdit->setText( val.setNum(1100) );
	thicknessEdit->setText( val.setNum(200) );
	numberBlocksBox->setValue( 9 );
	supportsCheckBox->setChecked( true );
}
