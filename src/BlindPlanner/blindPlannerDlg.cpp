/*
* blindPlannerDlg.cpp
*
*  Created on: Jun 11, 2011
*      Author: dang
*/

#define VERSION2

#include "blindPlannerDlg.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <QTextStream>
#include <QFileDialog>

#include "robot.h"
#include "blindPlannerUtil.h"
#include "graspExperienceBase.h"
#include "handAdjust.h"
#include "qualityEstimator.h"
#include "handControlAPI.h"

#include "SemanticPlanner/SemanticMap.h"
#include "quality.h"
#include "SensorInterface.h"

#include "debug.h"

#ifdef LOCAL_EXPLORE_ENABLED
#include "localExplore.h"
#endif

#define RENDER false

QualityEstimator * BlindPlannerDlg::mQualityEstimator = NULL;
HandControlAPI * BlindPlannerDlg::mHandControlAPI = NULL;

void BlindPlannerDlg::loadExperienceButton_clicked()
{

	std::string objName(graspItGUI->getIVmgr()->getWorld()->getGB(0)->getName().ascii());
	
#ifdef FILTER
	if( mGeb->loadGraspExperienceBase(experienceFilePath->text().toStdString(), -1, objName.c_str()) )
#else
	if( mGeb->loadGraspExperienceBase(experienceFilePath->text().toStdString()) )
#endif
	{
		std::cout << "Experience data base loaded successfully" << std::endl;
	}
	else
	{
		std::cout << "Experience data base not loaded" << std::endl;
	}
    loadExperienceButton->setEnabled(false);

}

void BlindPlannerDlg::loadPoseButton_clicked()
{
	std::string objName(graspItGUI->getIVmgr()->getWorld()->getGB(0)->getName().ascii());
	std::cout << "clicked" << poseFilePath->text().ascii() << std::endl;

#ifdef FILTER
	if( mGeb->loadHandPoses(poseFilePath->text().toStdString(), objName.c_str()) )
#else
	if( mGeb->loadHandPoses(poseFilePath->text().toStdString()) )
#endif
	{
		std::cout << "Pose data base loaded successfully" << std::endl;
	}
	else
	{
		std::cout << "Pose data base not loaded" << std::endl;
	}
    loadPoseButton->setEnabled(false);
}

void BlindPlannerDlg::setPoseButton_clicked()
{
	//go to the pose specified by graspId
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran( mGeb->getPose(graspId->text().toStdString()) );
}


void BlindPlannerDlg::compareButton_clicked()
{
    compareButton->setEnabled(false);

	mCurrentGrasp.setContactList(getCurrentGraspContactInfoList());
	mCurrentGrasp.setSpread(getCurrentHandSpread());
	mCurrentNNList = mGeb->getKNNBasedOnTactile(mCurrentGrasp);

    previousNNButton->setEnabled(true);
    nextNNButton->setEnabled(true);
    updateStatus();
	//mCurrentGrasp.printMe();

    adjustButton->setEnabled(true);
#ifdef VERSION2
    distanceLineEdit->setEnabled(true);
    epsilonLineEdit->setEnabled(true);
#endif
}

void BlindPlannerDlg::previousNNButton_clicked()
{
	mCurrentNNIndex = (mCurrentNNIndex == 0) ? mCurrentNNList.size() - 1 : mCurrentNNIndex - 1;
	updateStatus();
}

void BlindPlannerDlg::nextNNButton_clicked()
{
	mCurrentNNIndex = (mCurrentNNIndex == mCurrentNNList.size() - 1) ? 0 : mCurrentNNIndex + 1;
	updateStatus();
}

void BlindPlannerDlg::updateStatus()
{

	char status[50];

	//update database label
	if(mGeb)
	{
		sprintf(status, "%d grasps loaded", mGeb->getNumGrasps());
		numGraspsLabel->setText(status);
	}
	//mCurrentNNList[mCurrentNNIndex].printMe();

	//update nn label
	if(mCurrentNNList.size() > 0)
	{
		sprintf(status, "NNID: %s",mCurrentNNList[mCurrentNNIndex].getInfoString().c_str());
		nnIDLabel->setText(status);
	}

	//update index label
	sprintf(status, "%d/%d", mCurrentNNIndex, mK);
	indexLabel->setText(status);

	//update the distance label
    sprintf(status, "Distance: %lf", mCurrentNNList[mCurrentNNIndex].getTactileSimilarityMeasure(mCurrentGrasp));
	distanceLabel->setText(status);

	//mCurrentNNList[mCurrentNNIndex].printMe();
	//print out the contact info
	//std::cout << "\nNN: \n";
	//mCurrentNNList[mCurrentNNIndex].printMe();
	//std::cout << "\nMe: \n";
	//mCurrentGrasp.printMe();
}

void BlindPlannerDlg::perturbButton_clicked()
{
	double tx, ty, tz, qw, qx, qy, qz;
	sscanf(perturbLineEdit->text().ascii(), "<%lf,%lf,%lf>[%lf,%lf,%lf,%lf]", &tx, &ty, &tz, &qw, &qx, &qy, &qz);
	std::cout << tx << ", " << 
		ty << ", " << 
		tz << ", " << 
		qw << ", " << 
		qx << ", " << 
		qy << ", " << 
		qz << ", " << std::endl;
	transf t(Quaternion(qw, qx, qy, qz), vec3(tx, ty, tz));
	Hand* h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	h->setTran( t * h->getTran() );
	//std::cout << t << std::endl;
	//std::cout << t.inverse() << std::endl;
	std::cout << h->getTran() << std::endl;

	//get hand joints
	double joints[8];
	h->getJointValues(joints);
	for(int ji = 0; ji < 8; ji ++)
	{
		std::cout << joints[ji] << ", ";
	}
	std::cout << std::endl;
}

double BlindPlannerDlg::qualityEstimate()
{
	//examine the quality
	if(!mQualityEstimator)
		mQualityEstimator = new QualityEstimator();
	if(!mHandControlAPI)
		mHandControlAPI = new HandControlAPI();

	std::vector<double> sensors = getTactileSensorReadings();
	if(sensors.size() > 0)
	{
		std::cout << "to set sensor values" << std::endl;
		mQualityEstimator->setTactileReadings(sensors);
	} else {
		std::cout << "no sensor values detected" << std::endl;
	}
	if(numPadsWithTactileContacts() == 0)
	{
		std::cout << "no tactile contacts detected" << std::endl;
		return -2;
	}
	//setup the svm model trained from libsvm
	mQualityEstimator->setModel(svmFilePath->text().ascii());
	//setup the path to the codebook and normalizer
	mQualityEstimator->setCodebook(codebookFilePath->text().ascii());
	//setup the current grasp tactile sensing data
	mQualityEstimator->setTactileLocations(mHandControlAPI->getTactileLocations());

	double e;
	e =  mQualityEstimator->estimateStability();
	return e;
}

void BlindPlannerDlg::SVMEvalButton_clicked()
{
	double e = qualityEstimate();
	std::cout << "quality: " << e << std::endl;
	if(e > 0.5)
	{
		std::cout << "positive quality" << std::endl;
	}
	else
	{
		std::cout << "negative quality" << std::endl;
	}
	//done estimating the quality

    compareButton->setEnabled(true);

}

void BlindPlannerDlg::adjustButton_clicked()
{
	/*
	adjustment version 1: use tactile contacts to look for NN
	perturbe for each of the NN to look for another one that is even more similar to the current one online
	return the adjustment between the even more similar one and the NN
	*/
    adjustButton->setEnabled(false);
#ifdef VERSION_ONLINE
	GraspExperienceEntry gee;
	gee.setContactList(getCurrentGraspContactInfoList());
	gee.setSpread(getCurrentHandSpread());
	mHA.init(mGeb);
	mHA.setTactileNNList(gee, mCurrentNNList);
	mT = mHA.getHandAdjustmentONLINE(&mSpread);
#endif

	/*
	adjustment version 2: assuming we have already had a dense sampling of grasps around each pose and their corresponding qualities
	use tactile contacts to look for NN
	look around NN based on pose similarities and find a better one
	*/
#ifdef VERSION2
    distanceLineEdit->setEnabled(false);
    epsilonLineEdit->setEnabled(false);
	GraspExperienceEntry gee;
	gee.setContactList(getCurrentGraspContactInfoList());
	gee.setSpread(getCurrentHandSpread());
	mHA.init(mGeb);
	mHA.setTactileNNList(gee, mCurrentNNList);
	mT = mHA.getHandAdjustment2(mCurrentNNList[0], distanceLineEdit->text().toDouble(), epsilonLineEdit->text().toDouble());
#endif

	/*
	adjustment version 3: assuming we have precomputed all the tactile experience based on perturbed grasps
	*/
#ifdef VERSION_PRECOMP
	GraspExperienceEntry gee;
	gee.setContactList(getCurrentGraspContactInfoList());
	gee.setSpread(getCurrentHandSpread());
	mHA.init(mGeb);
	mT = mHA.getHandAdjustmentUsePrecomputedPerturbation(gee, &mSpread);
#endif

    applyAdjustmentButton->setEnabled(true);
}

void BlindPlannerDlg::applyAdjustmentButton_clicked()
{
    applyAdjustmentButton->setEnabled(false);
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * hand = w->getCurrentHand();
	transf handT = hand->getTran();
	hand->setTran(mT * handT);

	//reset the hand pose to zero and clear the breakaway flags
	resetHandPoseToZero();

	//shape the spread angle
	if(mSpread < 0.0001)
		mSpread = 0.0001;
	hand->forceDOFVal(0,mSpread);

	//move out of collision and grasp
	if(!w->noCollision())
	{
		std::cout << "collision found, now try to go back with spread angle: " << mSpread << std::endl;
		hand->findInitialContact(200);
	}

	//if the object is so close to the hand, we might as well just go forward to hit it
	//this corresponds to real experiments in the sense that the object moves in hand with
	//a slight offset
	if(!hand->approachToContact(5.0))
		hand->approachToContact(-5.0);
	hand->autoGrasp(!graspItGUI->useConsole() && RENDER);
}

void BlindPlannerDlg::loadObjectButton_clicked()
{
	GraspExperienceEntry gee;
	gee.setContactList(getCurrentGraspContactInfoList());
	gee.setSpread(getCurrentHandSpread());
	mHA.setTactileNNList(gee, mCurrentNNList);
	mHA.loadNNGraspFromCGDB(mCurrentNNIndex);
}

void BlindPlannerDlg::allInOneAdjustButton_clicked()
{
	compareButton_clicked();
	adjustButton_clicked();
	applyAdjustmentButton_clicked();
}


int BlindPlannerDlg::mExpId = 0;
void BlindPlannerDlg::testButton_clicked()
{
	//just loop through the poses
	int id_i, id_j;
	std::vector<transf> ap;
	SemanticMap sm;
	if(mExpId == 0)
	{
		position cog = graspItGUI->getIVmgr()->getWorld()->getGB(0)->getCoG() * graspItGUI->getIVmgr()->getWorld()->getGB(0)->getTran();
		std::cout << cog.x() << ", " << cog.y() << ", " << cog.z() << std::endl;
		ap = sm.getApproachingDirections(vec3(cog.x(), cog.y(), cog.z()),
			30*M_PI/180.0, 30*M_PI/180.0, 5*M_PI/180.0, //for the test, we do not want a very dense sampling
			//cokecan and gillette
			M_PI - 2.5*M_PI/180.0, M_PI - 2.5*M_PI/180.0 + 1*M_PI/180.0, //y
			-2.5*M_PI/180.0, 0.5*M_PI -2.5*M_PI/180.0 + 1*M_PI/180); //x
		//canteen
		//0.5*M_PI, M_PI + 1*M_PI/180.0, //y
		//0, 0.5*M_PI + 1*M_PI/180); //x

		std::cout << "loading: " << ap.size() << std::endl;
	}

	Hand * h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	for(size_t i = 0; i < ap.size(); ++i)
	{
		h->setTran(ap[i]);
		//open the hand
		h->autoGrasp(!graspItGUI->useConsole() && RENDER, -1);
		//reset the breakaways
		resetHandDOFBreakAwayFlags();
		//approach the object
		h->approachToContact(300);
		//close the fingers
		h->autoGrasp(!graspItGUI->useConsole() && RENDER, 1);
		id_i = i;
		id_j = -1;
		testStrategy();

		//back up
		for(int j = 0; j < 20; ++j)
		{
			std::cout << i << ", " << j << std::endl;
			h->autoGrasp(!graspItGUI->useConsole() && RENDER, -1);
			//reset the breakaways
			resetHandDOFBreakAwayFlags();
			//every other 10 mm
			transf t(Quaternion::IDENTITY, vec3(0,0,-10.0));
			h->setTran(t * h->getTran());
			h->autoGrasp(!graspItGUI->useConsole() && RENDER, 1);
			if(h->getNumContacts() < 1)
				break;
			id_j = j;
			testStrategy();
		}
	}
}

#define NOVEL_OBJ_EXP

void BlindPlannerDlg::sendToRobotButton_clicked()
{
/*
	if(mPoseError.size() < 1 || poseErrorComboBox->count() != mPoseError.size())
	{
		std::cout << "No pose error defined" << std::endl;
		return;
	}
	*/
	//execute the current grasp without offset in GraspIt!
	double joints[8];
	World * world = graspItGUI->getIVmgr()->getWorld();
	transf final = world->getCurrentHand()->getTran();
	world->getCurrentHand()->getJointValues(joints);

	//execute grasps with pose errors
	transf offset = mPoseError[poseErrorComboBox->currentIndex()];
	//execute grasps without pose errors as inside GraspIt!
	//transf offset = transf::IDENTITY;

	std::string msg;
#ifdef NOVEL_OBJ_EXP
	//this line should be deleted for general purposes
	//final = transf(Quaternion(0.683056, 0.187071, -0.682094, 0.182174), vec3(-675.895, 27.222, -221.70)); //the spray
	//final = transf(Quaternion(0.683056, 0.187071, -0.682094, 0.182174), vec3(-675.895, 27.222, -196.70)); //the wineglass (same as box)
	//final = transf(Quaternion(0.683056, 0.187071, -0.682094, 0.182174), vec3(-675.895, 27.222, -196.70)); //the box (same as wineglass)
	final = transf(Quaternion(0.683056, 0.187071, -0.682094, 0.182174), vec3(-675.895, 27.222, -246.70)); //the blue mug
	//final = transf(Quaternion(0.683056, 0.187071, -0.682094, 0.182174), vec3(-675.895, 27.222, -226.70)); //the canteen


	//final = transf(Quaternion(-0.001477, 0.342044, 0.939681, 0.001962), vec3(-687.577, -32.909, -136.093)); //the flashlight
	//final = transf(Quaternion(-0.002586, 0.460765, 0.887344, 0.017602), vec3(-605.523, 92.932, -133.435)); //the flashlight top
	//final = transf(Quaternion(0.021555, -0.517127, 0.820088, 0.01093), vec3(-584.543, -110.2, -217.766)); //the disk case top
	//translate in world's x,y
	//rotate along world's z
	transf orientationOffset(offset.rotation(), vec3::ZERO);
	transf newOrientation = final * orientationOffset;
	transf translateOffset(Quaternion::IDENTITY, offset.translation());
	transf newTranslation = final * translateOffset;
	transf newPose(newOrientation.rotation(), newTranslation.translation());

	msg = getGraspCommand(newPose, joints, isAdjustmentCheckbox->isChecked(), "M", 1.0); //apply grasp in the Staubli arm's base
#endif

#ifdef REGULAR_EXP
	msg = getGraspCommand(final * offset, joints, isAdjustmentCheckbox->isChecked()); //apply the offset in the object's coordinate system
#endif

	std::cout << msg.c_str() << std::endl;
	world->setCommand(msg.c_str());
	world->setCommandActive(true);
}

void BlindPlannerDlg::loadPoseErrorButton_clicked()
{
	//record the original pose
	mIdealGraspPose = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();
	//char path[128] = "poseerror.txt";

	QString path = QFileDialog::getOpenFileName(this, tr("Open Pose Error File"),
                                                 "",
                                                 tr("Files (*.txt)"));
	std::ifstream file(path.ascii());
	if(!file.is_open())
	{
		std::cout << "File not found: " << path.ascii() << std::endl;
		return;
	}

	mPoseError.clear();
	poseErrorComboBox->clear();

	std::string line;
	for(int i = 0; 1; ++i)
	{
		getline(file, line);
		if(line.empty())
			break;
		//double x, y, z, qw, qx, qy, qz;
		//sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &x, &y, &z, &qw, &qx, &qy, &qz);
		double x, y, theta;
		sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &theta);
		//in mm and degree
		transf  t = transf(Quaternion(theta * M_PI / 180.0, vec3(0,0,1)), vec3::ZERO) * 
			transf(Quaternion::IDENTITY, vec3(x, y, 0));
		mPoseError.push_back(t);
		//insert into the combo box
		poseErrorComboBox->insertItem(i, line.c_str());
	}
	file.close();
}

void BlindPlannerDlg::generatePoseErrorButton_clicked()
{
	FILE * fp = fopen("poseerror.txt", "a");
	double x = rand();
	double y = rand();
	double theta = rand();
	x = x / RAND_MAX * 60.0 - 30.0; // +- 30mm
	y = y / RAND_MAX * 60.0 - 30.0; // +- 30mm
	theta = theta / RAND_MAX * 40.0 - 20.0; // +- 20degree
	transf t = transf(Quaternion(theta * M_PI / 180.0, vec3(0,0,1)), vec3::ZERO) * 
		transf(Quaternion::IDENTITY, vec3(x, y, 0));
	std::cout << x << ", " << y << ", " << theta << std::endl;
	fprintf(fp, "%lf %lf %lf\n", x, y, theta);
	fclose(fp);
}

void BlindPlannerDlg::showGraspWithPoseErrorButton_clicked()
{
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(mIdealGraspPose * mPoseError[poseErrorComboBox->currentIndex()]);
	if(!graspItGUI->getIVmgr()->getWorld()->noCollision())
	{
			std::cout << "collision in pre-grasp position" << std::endl;
			//move back only when necessary
			if(!graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->findInitialContact(100))
			{
				std::cout << "cannot resolve" << std::endl;
			}
	}
	std::cout << "pose is: " << graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran() << std::endl;
}

void BlindPlannerDlg::showIdealGraspButton_clicked()
{
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(mIdealGraspPose);
}

void BlindPlannerDlg::dynamicExamButton_clicked()
{
	bool mDynamicsError;
	Hand * hand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();

	//test for collisions to make *absolutely sure we are OK
	CollisionReport colReport;
	int numCols = hand->getWorld()->getCollisionReport(&colReport);
	if (numCols) {
		//*code = DYNAMIC_APPROACH_FAILED;
		//return false;
		return;
	}

	QObject::connect(hand->getWorld(), SIGNAL(dynamicsError(const char*)),
					 this, SLOT(dynamicsError(const char*)));
	//this is more of a hack to cause the hand to autograsp in dynamics way
	//the world's callback for dynamics should never get called as everything 
	//gets done from inside here
	//hand->getWorld()->resetDynamics();
	//I think this happens in each dynamics step anyway
	//hand->getWorld()->resetDynamicWrenches();
	hand->getWorld()->turnOnDynamics();
	
	//this should set the desired values of the dof's
	hand->autoGrasp(true);

	//loop until dynamics is done
	mDynamicsError = false;
	int steps=0; int stepFailsafe = 250;
	while (1) {
		hand->getWorld()->stepDynamics();
		if (mDynamicsError) break;
		if (++steps > stepFailsafe) break;
		if (!(steps%50)) {DBGA("Dynamic step " << steps);}
		//we can't check if autograsp is done, so we just run for a pre-sepcified number of loops
	}
	QObject::disconnect(hand->getWorld(), SIGNAL(dynamicsError(const char*)),
				    	this, SLOT(dynamicsError(const char*)));
	//turn of dynamics; world dynamics on shouldn't have done anything anyway
	hand->getWorld()->turnOffDynamics();
	if (mDynamicsError) {
		DBGA("Dynamics error!");
		//*code = DYNAMIC_ERROR;
		//return false;
		return;
	} 
	DBGA("Dynamic autograsp complete in " << steps << " steps");
	//*code = DYNAMIC_SUCCESS;
}

void BlindPlannerDlg::autoGraspAtFirstContact_clicked()
{
	Hand * hand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	hand->autoGrasp(true, 1.0, true);
}

void BlindPlannerDlg::testStrategy()
{
	World * world = graspItGUI->getIVmgr()->getWorld();
	FILE * flog_quality, * flog_handpose;
	flog_quality = fopen("strategy_test.txt", "a");
	//record the grasp info
	fprintf(flog_quality, "%s_%d ", world->getGB(0)->getName().ascii(), mExpId);
	fclose(flog_quality);
	mExpId++;
	//record the hand pose before any test
	transf t = world->getCurrentHand()->getTran();

	//begin the test
	recordQuality();
	recordHandPose();
	for(int i = 0; i < 5; ++i)
	{
		allInOneAdjustButton_clicked();
		recordQuality();
		recordHandPose();
	}
	//end the record
	flog_quality = fopen("strategy_test.txt", "a");
	fprintf(flog_quality, "\n");
	fclose(flog_quality);
	flog_handpose = fopen("handpose.txt", "a");
	fprintf(flog_handpose, "\n");
	fclose(flog_handpose);

	//restore the hand pose
	world->getCurrentHand()->autoGrasp(!graspItGUI->useConsole() && RENDER, -1);
	resetHandDOFBreakAwayFlags();
	world->getCurrentHand()->setTran(t);
}

std::vector<double> BlindPlannerDlg::getTargetGraspQuality()
{
	GraspExperienceEntry e = getTargetGrasp();
	std::vector<double> res;
	res.resize(2);
	res[0] = e.getEpsilonQuality();
	res[1] = e.getVolumeQuality();
	return res;
}

GraspExperienceEntry BlindPlannerDlg::getTargetGrasp()
{
	GraspExperienceEntry e = mGeb->getEntry(mHA.getAdjustmentInfo().neighborIdPerNN[mHA.getAdjustmentInfo().targetNN]);
	return e;
}

double BlindPlannerDlg::getCurrentDistanceToGEDB()
{
	//compute on the fly
	//mCurrentGrasp.setContactList(getCurrentGraspContactInfoList());
	//mCurrentGrasp.setSpread(getCurrentHandSpread());
	//mCurrentNNList = mGeb->getKNNBasedOnTactile(mCurrentGrasp);

	//return what ever we recorded
	return mCurrentNNList[0].getDistance();
}

/*
Record the distance from the actual grasp to the closenest one in the database
*/
void BlindPlannerDlg::recordDistance(std::string surfix, bool newLine)
{
	FILE * flog_distance;

	std::string filename = "distance_";
	filename.append(surfix);
	filename.append(".txt");
	flog_distance = fopen(filename.c_str(), "a");

	double distance = getCurrentDistanceToGEDB();

	fprintf(flog_distance,"%s: %lf ", mCurrentNNList[0].getInfoString().c_str(), distance);
	if(newLine)
		fprintf(flog_distance,"\n");
	fclose(flog_distance);
}

/*
This records the quality of the grasp we are trying to achieve.
The grasp has the most similar tactile contacts with the current actual grasp after a certain perturbation.
*/
void BlindPlannerDlg::recordTargetNNQuality(std::string surfix, bool newLine)
{
	World * world = graspItGUI->getIVmgr()->getWorld();

	FILE * flog_quality;

	std::string filename = "target_quality_";
	filename.append(surfix);
	filename.append(".txt");

	flog_quality = fopen(filename.c_str(), "a");

	std::vector<double> quality;
	quality = getTargetGraspQuality();

	GraspExperienceEntry target = getTargetGrasp();

	fprintf(flog_quality,"%s: %lf %lf ", target.getInfoString().c_str(), quality[0], quality[1]);
	if(newLine)
		fprintf(flog_quality,"\n");
	fclose(flog_quality);

}

void BlindPlannerDlg::recordQuality(std::string surfix, bool newLine)
{
	World * world = graspItGUI->getIVmgr()->getWorld();

	FILE * flog_quality;

	std::string filename = "quality_";
	filename.append(surfix);
	filename.append(".txt");

	flog_quality = fopen(filename.c_str(), "a");
	// if there is no collision, then begin computation
	Hand* h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();

	if(!mEpsQual)
		mEpsQual = new QualEpsilon( h->getGrasp(), "Examine_dlg_qm","L1 Norm");

	if(!mVolQual)
		mVolQual = new QualVolume( h->getGrasp(), "Examine_dlg_qm","L1 Norm");
	h->getWorld()->findAllContacts();
	h->getWorld()->updateGrasps();

	double eq = mEpsQual->evaluate();
	double vq = mVolQual->evaluate();
	fprintf(flog_quality,"%lf %lf ", eq, vq);
	if(newLine)
		fprintf(flog_quality,"\n");
	fclose(flog_quality);

}

void BlindPlannerDlg::recordSVMQuality(std::string surfix, bool newLine)
{
	FILE * flog_svm;
	std::string filename = "svmQual_";
	filename.append(surfix);
	filename.append(".txt");

	flog_svm = fopen(filename.c_str(), "a");
	fprintf(flog_svm, "%lf ", qualityEstimate());
	if(newLine)
		fprintf(flog_svm,"\n");
	fclose(flog_svm);

}

void BlindPlannerDlg::recordHandPose(std::string surfix, bool newLine)
{
	FILE * flog_handpose;
	World * world = graspItGUI->getIVmgr()->getWorld();

	std::string filename = "handpose_";
	filename.append(surfix);
	filename.append(".txt");

	flog_handpose = fopen(filename.c_str(), "a");
	transf p = world->getCurrentHand()->getTran();
	fprintf(flog_handpose, "%s_%d ", world->getGB(0)->getName().ascii(), mExpId);
	fprintf(flog_handpose,"%lf %lf %lf %lf %lf %lf %lf %lf, ",
		p.translation().x(),
		p.translation().y(),
		p.translation().z(),
		p.rotation().w,
		p.rotation().x,
		p.rotation().y,
		p.rotation().z,
		world->getCurrentHand()->getDOF(0)->getVal());
	if(newLine)
		fprintf(flog_handpose,"\n");
	fclose(flog_handpose);
}

void BlindPlannerDlg::rotateButton_clicked()
{
	World * w = graspItGUI->getIVmgr()->getWorld();
	GraspableBody * body = w->getGB(0);
	transf t(Quaternion(0.5 * M_PI, vec3(1,0,0)), vec3(0,0,0));
	body->setTran( body->getTran() * t);
}

void BlindPlannerDlg::generateTestPoseButton_clicked()
{
	SemanticMap sm;
	position cog = graspItGUI->getIVmgr()->getWorld()->getGB(0)->getCoG() * graspItGUI->getIVmgr()->getWorld()->getGB(0)->getTran();
	mTestPose = sm.getApproachingDirections(vec3(cog.x(), cog.y(), cog.z()), 5*M_PI/180.0,
		30*M_PI/180.0, 30*M_PI/180.0,
		//cokecan and gillette
		M_PI - 2.5*M_PI/180.0, M_PI - 2.5*M_PI/180.0 + 1*M_PI/180.0, //y
		-2.5*M_PI/180.0, 0.5*M_PI -2.5*M_PI/180.0 + 1*M_PI/180); //x
	//canteen
	//0.5*M_PI, M_PI + 1*M_PI/180.0, //y
	//0, 0.5*M_PI + 1*M_PI/180); //x
}

void BlindPlannerDlg::gotoTestPoseButton_clicked()
{
	Hand* h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
	h->setTran(mTestPose[testPoseId->text().toInt()]);
	//open the hand
	h->autoGrasp(!graspItGUI->useConsole() && RENDER, -1);
	//reset the breakaways
	resetHandDOFBreakAwayFlags();
	//approach the object
	h->approachToContact(300);
}

bool BlindPlannerDlg::isAnIdentityAdjustment()
{
	double diff = fabs(mT.rotation().w - 1) + 
		fabs(mT.rotation().x) +
		fabs(mT.rotation().y) +
		fabs(mT.rotation().z) +
		fabs(mT.translation().x()) + 
		fabs(mT.translation().y()) + 
		fabs(mT.translation().z()) + 
		fabs(mSpread - graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getDOF(0)->getVal());
	//std::cout << "diff: " << diff << std::endl;
	return diff < 0.001;
}

void BlindPlannerDlg::distanceButton_clicked()
{
	std::cout << "-----------------------" << std::endl;
	mCurrentGrasp.printMe();
	std::cout << "-----------------------" << std::endl;
	mGeb->getEntry(graspId->text().toStdString()).printMe();
	std::cout << "distance is: " << mGeb->getEntry(graspId->text().toStdString()).getTactileSimilarityMeasure( mCurrentGrasp ) << std::endl;
}

void BlindPlannerDlg::writeoutButton_clicked()
{

	//generate file name
	std::string fname("tmp_tactile.txt");
	FILE *fresult = fopen(fname.c_str(),"w");
	QTextStream qts(fresult);

	World * w = graspItGUI->getIVmgr()->getWorld();
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		qts << sensorInd << " ";
		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);
	}
	fclose(fresult);

}

//void BlindPlannerDlg::sensorCB(void *data,SoSensor*)
//{
//	std::cout << "in" << std::endl;
//	BlindPlannerDlg* bp = (BlindPlannerDlg*) data;
//	bp->commandMonitorTextBrowser->setText(graspItGUI->getIVmgr()->getWorld()->getCommand().c_str());
//}


void BlindPlannerDlg::generatePerturbationListButton_clicked()
{
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	mOriginalPose = h->getTran();

	position diff(0,0,0);
	std::list<Contact *> contactList = w->getGB(0)->getContacts();
	for(std::list<Contact *>::iterator cp = contactList.begin(); cp != contactList.end(); ++cp)
	{
		Contact* ct = *(cp);

		//compute the contact within hand coordinate system
		diff = diff + ct->getPosition() * w->getGB(0)->getTran() * h->getTran().inverse();//contactInObject * objectInWorld * worldInHand
		//std::cout << diff[0] << ", " << diff[1] << ", " << diff[2] << std::endl;
	}
	vec3 diff2(diff[0]/contactList.size(), diff[1]/contactList.size(), diff[2]/contactList.size());
	double contactRange = diff2.len();
	mPerturbationList = HandAdjust::getPerturbationListBasedOnContacts(15, 15, 15, 10, contactRange);
}

void BlindPlannerDlg::previousPerturbationButton_clicked()
{
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	--mCurrentPerturbationIndex;
	if(mCurrentPerturbationIndex < 0)
		mCurrentPerturbationIndex = mPerturbationList.size() - 1;
	h->setTran(mPerturbationList[mCurrentPerturbationIndex] * mOriginalPose);
	char status[128];
	sprintf(status, "%d/%d", mCurrentPerturbationIndex, mPerturbationList.size());
	perturbationIndexLabel->setText(status);
}

void BlindPlannerDlg::nextPerturbationButton_clicked()
{
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	++mCurrentPerturbationIndex;
	mCurrentPerturbationIndex %= mPerturbationList.size();
	h->setTran(mPerturbationList[mCurrentPerturbationIndex] * mOriginalPose);
	char status[128];
	sprintf(status, "%d/%d", mCurrentPerturbationIndex, mPerturbationList.size());
	perturbationIndexLabel->setText(status);
}

void BlindPlannerDlg::zeroPerturbationButton_clicked()
{
	/*
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	mCurrentPerturbationIndex = -1;
	h->setTran(mOriginalPose);
	*/

	//LocalExplore le;
	//le.startExplore();
	//le.fitSurfaces();
}

void BlindPlannerDlg::outputPerturbedPoseButton_clicked()
{
	//generate file name
	std::string fname("test_perturbed_pose.txt");
	FILE *fresult = fopen(fname.c_str(),"w");
	QTextStream qts(fresult);
	for(int i = 0; i < mPoseError.size(); ++i)
	{
		transf p = mIdealGraspPose * mPoseError[i]; //mPoseError is used to describe the pose error of the object, rather than the hand
		qts << p.translation().x() << "," << 
			p.translation().y() << "," << 
			p.translation().z() << "," << 
			p.rotation().w << "," <<
			p.rotation().x << "," <<
			p.rotation().y << "," <<
			p.rotation().z << "," <<
			getCurrentHandSpread() << "\n";
	}
	qts.flush();
	fclose(fresult);
}
