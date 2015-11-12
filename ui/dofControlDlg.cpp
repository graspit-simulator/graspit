#include "dofControlDlg.h"

#include "world.h"
#include "robot.h"

void DOFControlDlg::init()
{
	SLIDER_STEPS = 100000;

	mValueList.clear();
	mBarList.clear();
//	mCheckList.clear();

	mSubLayout = this;
	setLayout(getNumDOFs());
	readRobotDOF();
	mSubLayout->show();
}
void DOFControlDlg::setLayout( int num )
{
	mainLayout = new QVBoxLayout(mSubLayout, 5); 

	QLabel *valueLabel = new QLabel(QString("Value:"), mSubLayout);
	QLabel *amplLabel = new QLabel(QString("Amplitude:"), mSubLayout);
	//QLabel *fixedLabel = new QLabel(QString("Fixed"), mSubLayout);

	QHBoxLayout *fakeRow = new QHBoxLayout(mainLayout,-1);
	fakeRow->addSpacing(400);

	QHBoxLayout *labelRow = new QHBoxLayout(mainLayout,-1);
	labelRow->addSpacing(5);
	labelRow->addWidget(valueLabel,0);
	labelRow->addWidget(amplLabel,1,Qt::AlignHCenter);
	//labelRow->addWidget(fixedLabel,0);
	//labelRow->addSpacing(5);
	mainLayout->addLayout(labelRow);

	for (int i=0; i<num; i++) {
		QHBoxLayout *graspRow = new QHBoxLayout(mainLayout,10);

		QLabel *dofValue = new QLabel(QString("0.00000"), mSubLayout);
		QScrollBar *bar = new QScrollBar(Qt::Horizontal, mSubLayout);
		bar->setRange( -SLIDER_STEPS, SLIDER_STEPS );
		bar->setPageStep(5000);
		bar->setLineStep(1000);
		bar->setValue(0);
		QCheckBox *box = new QCheckBox(mSubLayout);

		graspRow->addSpacing(15);
		graspRow->addWidget(dofValue,0);
		graspRow->addWidget(bar,1);
		graspRow->addWidget(box,0);
		graspRow->addSpacing(15);

		mValueList.push_back(dofValue);
		mBarList.push_back(bar);
		//mCheckList.push_back(box);

		connect(bar,SIGNAL(sliderMoved(int)), this, SLOT(dofChanged()) );
		//comment this one out
		//connect(bar,SIGNAL(valueChanged(int)), this, SLOT(eigenGraspChanged()) );
		connect(bar,SIGNAL(nextLine()), this, SLOT(dofChanged()) );
		connect(bar,SIGNAL(prevLine()), this, SLOT(dofChanged()) );
		connect(bar,SIGNAL(nextPage()), this, SLOT(dofChanged()) );
		connect(bar,SIGNAL(prevPage()), this, SLOT(dofChanged()) );
		connect(bar,SIGNAL(sliderReleased()), this, SLOT(dofChanged()) );
		//connect(box,SIGNAL(clicked()), this, SLOT(fixBoxChanged()) );
	}	
	mainLayout->addSpacing(20);
}

void DOFControlDlg::dofChanged()
{
	std::vector<double> dof;
	Hand *h = mWorld->getCurrentHand();
	dof.resize(h->getNumDOF());

	//get whole dof vector
	mWorld->getCurrentHand()->getDOFVals(&dof[0]);

	//modify those changed
	for(size_t i = 0; i < mBarList.size(); ++i)
	{
		//std::cout << mBarList[i]->value() << std::endl;
		int dofIndex = mDofMap[i];
		dof[dofIndex] = ((double)mBarList[i]->value() - (double)mBarList[i]->minimum()) / //offset from minimum
			( (double)mBarList[i]->maximum() - (double)mBarList[i]->minimum() ) * // get the percentage in bar space
			(h->getDOF(dofIndex)->getMax() - h->getDOF(dofIndex)->getMin()) + //convert to dof space
			h->getDOF(dofIndex)->getMin(); // offset
		QString value;
		value.setNum(dof[dofIndex]);
		mValueList[i]->setText(value);
	}

	h->forceDOFVals(&dof[0]);

}

void DOFControlDlg::fixBoxChanged()
{

}

int DOFControlDlg::getNumDOFs()
{
	double epsilon = 0.001;
	mDofMap.clear();
	Hand* h = mWorld->getCurrentHand();
	int k = 0;
	for(int i = 0; i < h->getNumDOF(); ++i){
		if( abs (h->getDOF(i)->getMax() - h->getDOF(i)->getMin()) < epsilon )
			continue;
		mDofMap.push_back(i);
		++k;
	}
	return k;
}

void DOFControlDlg::readRobotDOF()
{
	std::vector<double> dof;
	Hand *h = mWorld->getCurrentHand();
	dof.resize(h->getNumDOF());
	//get the dof vals
	h->getDOFVals(&dof[0]);
	for(size_t i = 0 ; i < mBarList.size(); ++i)
	{
		int dofIndex = mDofMap[i];
		double tmp = (double)mBarList[i]->maximum() - (double)mBarList[i]->minimum(); //span of the bar
		double dofPercentage = (dof[dofIndex] - h->getDOF(dofIndex)->getMin()) / (h->getDOF(dofIndex)->getMax() - h->getDOF(dofIndex)->getMin()); //percentage in dof space
		tmp *= dofPercentage;
		tmp +=(double)mBarList[i]->minimum(); //offset

		mBarList[i]->setValue((int)tmp);
		std::cout << mBarList[i]->value() << std::endl;
	}
}