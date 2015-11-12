#include "arizonaProjectDlg.h"

#include <QFileDialog>

#include "graspitGUI.h"
#include "ivmgr.h"

#include "Arizona_Test.h"
#include "Arizona_Process.h"
#include "world.h"


void ArizonaProjectDlg::exitButton_clicked()
{
 QDialog::accept();
}

void ArizonaProjectDlg::selectRawFileButton_clicked()
{
 QString s = QFileDialog::getOpenFileName(
  QString(getenv("GRASPIT"))+QString("/misc/arizona"),
  "Raw Experiment Data (*.txt)",
  this,
  "open file dialog",
  "Choose a file to open" );
 if(s == QString::null)return;
    rawFileSelectedTextLabel->setText(s);
}

void ArizonaProjectDlg::processButton_clicked()
{
	if(!mAp->load_and_preprocess(rawFileSelectedTextLabel->text())){
			std::cout << "Failed to load and pre-process the experiment data" << std::endl;
			return;
		}
	else{
		std::cout << "Successfully loaded and pre-processed the experiment data" << std::endl;
	}
}

void ArizonaProjectDlg::EllipsoidFileButton_clicked()
{
	QString s = QFileDialog::getOpenFileName(
		QString(getenv("GRASPIT"))+QString("/misc/arizona"),
		"Ellipsoid Data (*.txt)",
		this,
		"open file dialog",
		"Choose a file to open" );
	if(s == QString::null)
		return;
	ellipsoidFileSelectedTextLabel->setText(s);
}

void ArizonaProjectDlg::loadObjectFileButton_clicked()
{
	QString s = QFileDialog::getOpenFileName(
		QString(getenv("GRASPIT"))+QString("/models/objects/"), "Grasp Data (*.xml)",
				this, "open file dialog", "Choose a file to open" );
	if(s == QString::null) return;
 
	objectFileSelectedTextLabel->setText(s);
	mAp->setObject( (GraspableBody*) graspItGUI->getIVmgr()->getWorld()->importBody("GraspableBody",s) );

	if (mAp->getTotalExp()) {
		showGWSButton->setEnabled(TRUE);
		testAllButton->setEnabled(TRUE);
		nextExpButton->setEnabled(TRUE);
		prevExpButton->setEnabled(TRUE);

		mAp->getTest()->setTestData(mAp->getCurrentRawData(), ellipsoidFileSelectedTextLabel->text(), 
									isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked());
		updateDlg();
	}
}

void ArizonaProjectDlg::destinationFolderButton_clicked()
{
 QString dir = QFileDialog::getExistingDirectory(QString("C:\\"));
 if(dir == QString::null)
  return;
 destinationFolderLabel->setText(dir);
}

void ArizonaProjectDlg::init()
{
 mAp = new ArizonaProcess();
}

void ArizonaProjectDlg::saveVgrButton_clicked()
{
 mAp->saveExpToVGR(rawFileSelectedTextLabel->text(),
  ellipsoidFileSelectedTextLabel->text(),
  destinationFolderLabel->text());
}



void ArizonaProjectDlg::prevExpButton_clicked()
{
 mAp->getTest()->setTestData(mAp->getPrevRawData(), ellipsoidFileSelectedTextLabel->text(), 
           isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked());
 updateDlg();
}


void ArizonaProjectDlg::nextExpButton_clicked()
{
 mAp->getTest()->setTestData(mAp->getNextRawData(), ellipsoidFileSelectedTextLabel->text(), 
        isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked());
 updateDlg();
}


void ArizonaProjectDlg::is3DCheckBox_clicked()
{
 mAp->getTest()->set3D(isBuildIn3DCheckBox->isChecked());
 updateDlg();
}


void ArizonaProjectDlg::testAllButton_clicked()
{
	// need to find a way to integrate it
	if(radioQual->isChecked()){
		std::cout << "quality test" << std::endl;
		mAp->testAllQual(isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked(), ellipsoidFileSelectedTextLabel->text(), fnTextBox->text());
	}
	else if(radioForce->isChecked()){
		std::cout << "force test" << std::endl;
		mAp->testAllForce(isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked(), ellipsoidFileSelectedTextLabel->text(), fnTextBox->text());
	}
	else if(radioContact->isChecked()){
		std::cout << "contact output" << std::endl;
		mAp->writeAllContact(isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked(), ellipsoidFileSelectedTextLabel->text(), fnTextBox->text());
	}
	else{
		std::cout << "Please choose a type to test..." << std::endl;
	}
}


void ArizonaProjectDlg::updateDlg()
{
 QString current, total, force;
 current.setNum(mAp->getCurrentExp() + 1);
 total.setNum(mAp->getTotalExp());
 expNumberLabel->setText(current + QString("/") + total);

 mAp->getTest()->initializeTest();

 QString qual;
 qual.setNum(mAp->getTest()->getQuality());
 qualityLabel->setText(QString("Quality: ") + qual);

 typeLabel->setText(QString("Type: " + mAp->getTest()->getCurrentType()));
 CMLabel->setText(QString("CM: ") + mAp->getTest()->getCMPattern());

 force.setNum(mAp->getTest()->getForce());
 forceLabel->setText(QString("Min force: " + force));
}

void ArizonaProjectDlg::showGWSButton_clicked()
{
 mAp->getTest()->showGWSProjection();
}


void ArizonaProjectDlg::isFlippedCheckBox_clicked()
{
	mAp->getTest()->setFlipped(isFlippedCheckBox->isChecked());
	mAp->getTest()->setTestData(mAp->getCurrentRawData(), ellipsoidFileSelectedTextLabel->text(), 
           isBuildIn3DCheckBox->isChecked(), isFlippedCheckBox->isChecked());
	updateDlg();
}
