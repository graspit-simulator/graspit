#include "semanticPlannerDlg.h"

#include "graspit_db_model.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "bBox.h"

#include <direct.h>


#include <sstream>

#include "Utilities.h"
#include "Views.h"
#include "SensorInterface.h"

#include "egPlanner.h"
#include "guidedPlanner.h"
#include "searchEnergy.h"
#include "searchState.h"
#include "eigenGrasp.h"
#include "BlindPlanner\blindPlannerUtil.h"
#include "BlindPlanner\graspExperienceEntry.h"
#include "graspviewer.h"

#define SEMANTIC_FOLDER "C:\\output_data\\graspit_test_scans"

//helper functor that decide the order of two grasps based on their average test score
bool shorterInDistance2(compareItem i1, compareItem i2){
	return i1.distance < i2.distance;
}

SemanticPlannerDlg::SemanticPlannerDlg(QWidget *parent) :
QDialog(parent),
ui(new Ui::SemanticPlannerDlg),
mTheta(0),
mPhi(-90),
mNNIdx(0),
mCurrentSolution(0),
mGraspViewer(NULL)
{
	ui->setupUi(this);
	QObject::connect(ui->loadScanbookButton, SIGNAL(clicked()), this, SLOT(loadScanbookButton_clicked()));
	QObject::connect(ui->scanButton, SIGNAL(clicked()), this, SLOT(scanButton_clicked()));
	QObject::connect(ui->encodeButton, SIGNAL(clicked()), this, SLOT(encodeButton_clicked()));
	QObject::connect(ui->getNNButton, SIGNAL(clicked()), this, SLOT(getNNButton_clicked()));
	QObject::connect(ui->bestButton, SIGNAL(clicked()), this, SLOT(bestButton_clicked()));
	QObject::connect(ui->previousNeighborButton, SIGNAL(clicked()), this, SLOT(previousNeighborButton_clicked()));
	QObject::connect(ui->nextNeighborButton, SIGNAL(clicked()), this, SLOT(nextNeighborButton_clicked()));
	QObject::connect(ui->setHandButton, SIGNAL(clicked()), this, SLOT(setHandButton_clicked()));
	QObject::connect(ui->planButton, SIGNAL(clicked()), this, SLOT(planButton_clicked()));
	QObject::connect(ui->voteButton, SIGNAL(clicked()), this, SLOT(voteButton_clicked()));
	QObject::connect(ui->tactileButton, SIGNAL(clicked()), this, SLOT(tactileButton_clicked()));
	QObject::connect(ui->adjustHandButton, SIGNAL(clicked()), this, SLOT(adjustHandButton_clicked()));
	QObject::connect(ui->loadSemanticGraspsButton, SIGNAL(clicked()), this, SLOT(loadSemanticGraspsButton_clicked()));
	QObject::connect(ui->pauseButton, SIGNAL(clicked()), this, SLOT(pauseButton_clicked()));
	QObject::connect(ui->previousButton, SIGNAL(clicked()), this, SLOT(previousButton_clicked()));
	QObject::connect(ui->nextButton, SIGNAL(clicked()), this, SLOT(nextButton_clicked()));
	QObject::connect(ui->refineButton, SIGNAL(clicked()), this, SLOT(refineButton_clicked()));
	QObject::connect(ui->storeButton, SIGNAL(clicked()), this, SLOT(storeButton_clicked()));
	QObject::connect(ui->showExamplePreGraspButton, SIGNAL(clicked()), this, SLOT(showExamplePreGraspButton_clicked()));
	QObject::connect(ui->rankButton, SIGNAL(clicked()), this, SLOT(rankButton_clicked()));
	QObject::connect(ui->similarityMeasureButton, SIGNAL(clicked()), this, SLOT(similarityMeasureButton_clicked()));
	QObject::connect(ui->executeButton, SIGNAL(clicked()), this, SLOT(executeButton_clicked()));
	QObject::connect(ui->showCorrespondenceButton, SIGNAL(clicked()), this, SLOT(showCorrespondenceButton_clicked()));
	QObject::connect(ui->loadCorrespondenceButton, SIGNAL(clicked()), this, SLOT(loadCorrespondenceButton_clicked()));
	QObject::connect(ui->loadSGCorrButton, SIGNAL(clicked()), this, SLOT(loadSGCorrButton_clicked()));
	QObject::connect(ui->computeTransformButton, SIGNAL(clicked()), this, SLOT(computeTransformButton_clicked()));


	aSep = new SoSeparator;
	aRot = new SoRotation;
	bRot = new SoRotation;
	aTransl = new SoTranslation;
	arrow = new SoArrow;

	mHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
}

SemanticPlannerDlg::~SemanticPlannerDlg()
{
	delete ui;
}

void SemanticPlannerDlg::loadScanbookButton_clicked()
{
	mSemanticMap.setCodebookFilePath(ui->codebookFilePathInlineEdit->text().latin1());
	mSemanticMap.loadCodebook();

	char status[100];
	sprintf(status, "loaded: %d, %d %d, %d, %d", mSemanticMap.getNumSamples(), mSemanticMap.getNumBins(),
		mSemanticMap.getIntervalD(), mSemanticMap.getIntervalH(), mSemanticMap.getIntervalV());

	ui->codebookStatusLabel->setText(status);
}

void SemanticPlannerDlg::loadSemanticGraspsButton_clicked()
{

	////std::vector<BoundingBox> bvs;
	////vec3 center, corner;
	////graspItGUI->getIVmgr()->getWorld()->getBvs(graspItGUI->getIVmgr()->getWorld()->getGB(0), 0, &bvs);
	////center = bvs[0].getTran().translation();
	////vec3 size = bvs[0].halfSize * bvs[0].getTran();

	////std::cout << center.x() << ", " << center.y() << ", " << center.z() << std::endl;
	////std::cout << size.x() << ", " << size.y() << ", " << size.z() << std::endl;
	////for(int x = -1; x <2; x += 2){
	////	for(int y = -1; y <2; y += 2){
	////		for(int z = -1; z <2; z += 2)
	////		{
	////			corner = vec3(center.x() + x * size.x(), center.y() + y * size.y(), center.z() + z * size.z());
	////			std::cout << corner.x() << ", " << corner.y() << ", " << corner.z() << std::endl;
	////		}
	////	}
	////}

	////return;

	mSemanticMap.setSemanticGraspFilePath(ui->semanticGraspPathInlineEdit->text().toStdString());
	mSemanticMap.loadSemanticGrasp();
}

void SemanticPlannerDlg::scanButton_clicked()
{
	scan2();
	mSemanticMap.setTargetScan(sm.getDepth());
}

void SemanticPlannerDlg::encodeButton_clicked()
{
	mSemanticMap.encodeTargetScan();
}

void SemanticPlannerDlg::getNNButton_clicked()
{
	mNeighbors = mSemanticMap.getNN();
}

void SemanticPlannerDlg::bestButton_clicked()
{
	if(mNeighbors.size() == 0)
	{
		std::cout << "No neighbors\n";
		return;
	}
	mNNIdx = 0;
	displayArrow(mNeighbors[mNNIdx].sampleIdx);
}

void SemanticPlannerDlg::previousNeighborButton_clicked()
{
	if(mNeighbors.size() == 0)
	{
		std::cout << "No neighbors\n";
		return;
	}
	mNNIdx = (mNNIdx == 0) ? mNeighbors.size() - 1 : mNNIdx - 1;
	displayArrow(mNeighbors[mNNIdx].sampleIdx);
	//position cog = mCurrentLoadedModel->getGraspableBody()->getCoG();
	//transf p = mSemanticMap.getOriginalPose(mNeighbors[mNNIdx].sampleIdx, vec3(cog[0],cog[1],cog[2]));
	//displayArrow(p);
}

void SemanticPlannerDlg::nextNeighborButton_clicked()
{
	if(mNeighbors.size() == 0)
	{
		std::cout << "No neighbors\n";
		return;
	}
	mNNIdx++;
	mNNIdx %= mNeighbors.size();
	displayArrow(mNeighbors[mNNIdx].sampleIdx);

	//position cog = mCurrentLoadedModel->getGraspableBody()->getCoG();
	//transf p = mSemanticMap.getOriginalPose(mNeighbors[mNNIdx].sampleIdx, vec3(cog[0],cog[1],cog[2]));
	//displayArrow(p);
}

void SemanticPlannerDlg::setHandButton_clicked()
{
	std::cout << "maximum radius is : " << mCurrentLoadedModel->getGraspableBody()->getMaxRadius() << std::endl;
	position p = mCurrentLoadedModel->getGraspableBody()->getCoG();
	std::cout << "cog: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
	//	transf initP = transf(Quaternion(), vec3(p[0], p[1], -mCurrentLoadedModel->getGraspableBody()->getMaxRadius() * 0.75)), pose;
	transf pose;

	double offset_theta, offset_phi;

	offset_theta = 0.0;
	offset_phi = 0.0;

	{
		double theta_degree = ui->thetaLineEdit->text().toDouble();
		double theta = (theta_degree + offset_theta) * M_PI / 180.0;
		{
			double phi_degree = ui->phiLineEdit->text().toDouble();
			double phi = (phi_degree + offset_phi) * M_PI / 180.0;
			pose = mSemanticMap.getOriginalPose(theta, phi, vec3(p[0],p[1],p[2]));
			transf local = transf(Quaternion(M_PI/2, vec3::Z), vec3::ZERO);
			graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(pose);

		}
	}
}

void SemanticPlannerDlg::planButton_clicked()
{

}

void SemanticPlannerDlg::voteButton_clicked()
{
	mVotingWinner = mSemanticMap.getVotingWinner(mNeighbors, ui->numberOfVotingNeighbors->text().toInt());
}

void SemanticPlannerDlg::tactileButton_clicked()
{
	collectTactileFeedback();
}

void SemanticPlannerDlg::adjustHandButton_clicked()
{

	//////transf handT = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();
	//////transf move = transf(Quaternion(30*M_PI/180.0,vec3(0,0,1)),vec3(0,0,0));
	//////graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(handT * move);

	////adjust the hand's pose to the best one
	//int bestSampleIdx = ui->bestIdxLineEdit->text().toInt();
	//position cog = mCurrentLoadedModel->getGraspableBody()->getCoG();
	//transf tranPose = mSemanticMap.getBestPoseInCurrentModel(mVotingWinner.sampleIdx, bestSampleIdx, vec3(cog[0],cog[1],cog[2]));

	//transf currentPose = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();
	//graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(currentPose * tranPose);

	////make the rolling angle
	//transf t = mHand->getTran();
	//transf ad = transf(Quaternion(ui->rotLineEdit->text().toDouble(),vec3(0,0,1)),vec3(0,0,0));
	//mHand->setTran(ad * t);

	////move the dofs to the idea one for grasp planning
	//showExamplePreGraspButton_clicked();

	SemanticGrasp sg = mSemanticMap.getSemanticGraspList()[0];
	int bestSampleIdx = sg.getApproachingDirectionID();
	position cog = mCurrentLoadedModel->getGraspableBody()->getCoG();
	transf tranPose = mSemanticMap.getBestPoseInCurrentModel(mVotingWinner.sampleIdx, bestSampleIdx, vec3(cog[0],cog[1],cog[2]));

	transf currentPose = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(currentPose * tranPose);

	//make the rolling angle
	transf t = mHand->getTran();
	//transf ad = transf(Quaternion(ui->rotLineEdit->text().toDouble(),vec3(0,0,1)),vec3(0,0,0));
	transf ad = sg.getRollingAdjustment();
	mHand->setTran(ad * t);

	//move the dofs to the idea one for grasp planning
	showExamplePreGraspButton_clicked();
}

void SemanticPlannerDlg::pauseButton_clicked()
{
	if (!mPlanner->isActive()){
		mPlanner->startPlanner();
	} else {
		mPlanner->pausePlanner();
	}
}

void SemanticPlannerDlg::refineButton_clicked()
{
	plan();
}

void SemanticPlannerDlg::rankButton_clicked()
{
	mDistList.clear();
	std::vector<GraspExperienceEntry> graspList;
	for(int i = 1; i < mPlanner->getListSize(); i+=2)
	{
		//instantialize this grasp candidate
		mPlanner->showGrasp(i);
		update();
		mHand->autoGrasp(true, 1);
		GraspExperienceEntry gee = GraspExperienceEntry::getCurrentGraspAsGraspExperienceEntry();

		compareItem ci;
		ci.sampleIdx = i;
		SemanticGrasp sg = mSemanticMap.getSemanticGraspList()[0];
		ci.distance = gee.getTactileSimilarityMeasure(sg.getGraspExperienceEntry());
		mDistList.push_back(ci);
	}
	std::sort(mDistList.begin(), mDistList.end(), shorterInDistance2);//increasing order

	for(int i = 0; i < mDistList.size(); ++i)
	{
		std::cout << mDistList[i].sampleIdx << " " << mDistList[i].distance << std::endl;
	}
}

void SemanticPlannerDlg::nextButton_clicked()
{
	mCurrentSolution = (mCurrentSolution == mPlanner->getListSize() - 1) ? 0 : mCurrentSolution + 1;
	mPlanner->showGrasp(mCurrentSolution);
	update();
	mPlanner->getGrasp(mCurrentSolution)->printState();
}

void SemanticPlannerDlg::previousButton_clicked()
{
	mCurrentSolution = (mCurrentSolution == 0) ? mPlanner->getListSize() - 1 : mCurrentSolution - 1;
	mPlanner->showGrasp(mCurrentSolution);
	update();
	mPlanner->getGrasp(mCurrentSolution)->printState();
}

void SemanticPlannerDlg::storeButton_clicked()
{
	FILE* fp_g, *fp_gp, *fp_te;
	char params_filename[128], grasp_filename[128], te_filename[128];
	sprintf(grasp_filename, "%s\\%s", SEMANTIC_FOLDER, ui->SGFileNameInlineEdit->text().latin1());
	sprintf(params_filename, "%s\\%s%s",SEMANTIC_FOLDER, "params_", ui->SGFileNameInlineEdit->text().latin1());
	sprintf(te_filename, "%s\\%s%s",SEMANTIC_FOLDER, "te_", ui->SGFileNameInlineEdit->text().latin1());

	fp_g = fopen(grasp_filename, "w");
	fp_gp = fopen(params_filename, "w");
	fp_te = fopen(te_filename, "w");

	//one grasp
	fprintf(fp_g, "1\n");
	fprintf(fp_g, "%s\n", mHand->getName().toStdString().c_str());
	fprintf(fp_g, "test_semantics\n45\n90\n");

	//print dof
	fprintf(fp_g, "%d ", mHand->getNumDOF());
	for(size_t i = 0; i < mHand->getNumDOF(); ++i)
	{
		fprintf(fp_g, "%lf ", mHand->getDOF(i)->getVal());
	}
	fprintf(fp_g, "\n");

	fprintf(fp_g, "%s\n", params_filename);
	//tactile
	std::vector<double> tactile = getTactileSensorReadings();
	fprintf(fp_g, "%d ", tactile.size());
	for(size_t i = 0; i < tactile.size(); ++i)
	{
		fprintf(fp_g, "%lf ", tactile[i]);
	}

	fprintf(fp_g, "\n%s", te_filename);

	//planner seeding params
	GraspPlanningState * sol = new GraspPlanningState(mPlanner->getGrasp(mCurrentSolution));
	sol->writeToFile(fp_gp);

	//tactile experience
	//dof value, only for Barrett hand, should be considered more general
	fprintf(fp_te, "%lf\n", mHand->getDOF(0)->getVal());
	std::vector<ContactInfo> cl = getCurrentGraspContactInfoList();
	fprintf(fp_te, "ID_0 -1 -1 %d ", cl.size());
	for(size_t i = 0; i < cl.size(); ++i)
	{
		vec3 loc = cl[i].location;
		Quaternion rot = cl[i].orientation;
		fprintf(fp_te, "%lf %lf %lf %lf %lf %lf %lf %lf ", cl[i].force, loc.x(), loc.y(), loc.z(), rot.w, rot.x, rot.y, rot.z);
	}

	fclose(fp_g);
	fclose(fp_gp);
	fclose(fp_te);
}

void SemanticPlannerDlg::similarityMeasureButton_clicked()
{
	GraspExperienceEntry gee = GraspExperienceEntry::getCurrentGraspAsGraspExperienceEntry();
	SemanticGrasp sg = mSemanticMap.getSemanticGraspList()[0];
	double distance = gee.getTactileSimilarityMeasure(sg.getGraspExperienceEntry());

	std::cout << "---------------------" << std::endl;
	sg.getGraspExperienceEntry().printMe();
	std::cout << "---------------------" << std::endl;
	gee.printMe();
	std::cout << "---------------------" << std::endl;
	std::cout << "distance: " << distance << std::endl;
}

void SemanticPlannerDlg::showExamplePreGraspButton_clicked()
{
	//std::vector<SemanticGrasp> sgList = mSemanticMap.getSemanticGraspList();
	//std::vector<double> amps = sgList[0].getDOFs(), dofs;
	//dofs.resize(4);
	//mHand->getEigenGrasps()->getDOF(&amps[0], &dofs[0]);
	//mHand->forceDOFVals(&dofs[0]);
}

//execute the current grasp in GraspIt!
void SemanticPlannerDlg::executeButton_clicked()
{
	double joints[8];
	World * world = graspItGUI->getIVmgr()->getWorld();
	transf final = world->getCurrentHand()->getTran();
	world->getCurrentHand()->getJointValues(joints);

	std::string msg;
	msg = getGraspCommand(final, joints);
	std::cout << msg.c_str() << std::endl;
	world->setCommand(msg.c_str());
	world->setCommandActive(true);
}


void SemanticPlannerDlg::loadSGCorrButton_clicked()
{
	if(!mGraspViewer)
		mGraspViewer = new GraspViewer(graspItGUI->getIVmgr()->getViewer());
	mGraspViewer->loadWorld(ui->SGCorrFilePathInlineEdit->text());
}

void SemanticPlannerDlg::loadCorrespondenceButton_clicked()
{
	const char * path = ui->correspondenceFilePathInlineEdit->text().ascii();
	std::ifstream file(path);
	if(!file.is_open())
	{
		std::cout << "File not found: " << path << std::endl;
		return;
	}
	std::vector<position> originalV;
	std::vector<int> corr;

	std::string line;
	double x, y, z;
	int m;
	//file opened
	for(;;)
	{
		getline(file, line);
		if(line.empty())
			break;
		sscanf(line.c_str(),"%lf %lf %lf %d", &x, &y, &z, &m);
		originalV.push_back(position(x,y,z));
		corr.push_back(m);
	}
	file.close();

	mShapeCorrespondence.setCorrespondence(corr);
	mShapeCorrespondence.setOriginalVertices(originalV);
	mShapeCorrespondence.initFLANN();
}

void SemanticPlannerDlg::showCorrespondenceButton_clicked()
{
	World * w = mGraspViewer->getWorld();
	GraspableBody * gbOriginal = mGraspViewer->getGraspableBody();
	GraspableBody * gbTarget = graspItGUI->getIVmgr()->getWorld()->getGB(0);
	std::list<Contact*> clist = gbOriginal->getContacts();
	std::list<Contact*>::const_iterator it;
	std::vector<augmentedVec3> mappedVertices;

	//storing the target object vertices
	std::vector<position> vertices;
	vertices = gbTarget->getRawGeometryVertices();
	mShapeCorrespondence.setTargetVertices(vertices);

	std::vector<vec3> queryContacts;

	for (it = clist.begin(); it!=clist.end(); it++) {
		position loc = (*it)->getLocation();
		std::cout <<
			loc.x() << ", " <<
			loc.y() << ", " <<
			loc.z() << std::endl;
		queryContacts.push_back(vec3(loc.x(), loc.y(), loc.z()));
	}
	mappedVertices = mShapeCorrespondence.getMappedVertices(queryContacts, 30);
	paintCorrespondence(mappedVertices);
}

void SemanticPlannerDlg::computeTransformButton_clicked()
{
	mShapeCorrespondence.getTransform();
}

void SemanticPlannerDlg::paintCorrespondence(std::vector<augmentedVec3> vertices)
{
	SoSeparator* root = graspItGUI->getIVmgr()->getWorld()->getIVRoot();

	if(mVisualizer.obj)
		root->removeChild(mVisualizer.obj);
	mVisualizer.obj = new SoSeparator;
	mVisualizer.ps = new SoIndexedPointSet;
	mVisualizer.coordinate = new SoCoordinate3;
	mVisualizer.material = new SoMaterial;
	mVisualizer.binding = new SoMaterialBinding;
	mVisualizer.style = new SoDrawStyle;

	mVisualizer.style->pointSize = 5.0;
	mVisualizer.binding->value = SoMaterialBinding::PER_VERTEX_INDEXED;

	double maximum = 0;
	for(size_t k = 0; k < vertices.size(); ++k)
	{
		if(vertices[k].value > maximum)
			maximum = vertices[k].value;
	}

	for(size_t i = 0; i < vertices.size(); ++i)
	{
		mVisualizer.coordinate->point.set1Value(i, vertices[i].vector.x(), vertices[i].vector.y(), vertices[i].vector.z());
		mVisualizer.ps->coordIndex.set1Value(i, i);
		mVisualizer.ps->materialIndex.set1Value(i, i);
		mVisualizer.material->diffuseColor.set1Value(i, SbColor(vertices[i].value / maximum, 0,0));
	}

	mVisualizer.obj->addChild(mVisualizer.coordinate);
	mVisualizer.obj->addChild(mVisualizer.material);
	mVisualizer.obj->addChild(mVisualizer.binding);
	mVisualizer.obj->addChild(mVisualizer.style);
	mVisualizer.obj->addChild(mVisualizer.ps);
	root->addChild(mVisualizer.obj);
}

void SemanticPlannerDlg::scan2()
{
	//this defines half the fov
	double fov_horizontal = 19.8;
	double fov_vertical = 23.75;
	sm.setOptics(fov_horizontal, fov_vertical, 144, 176);

	//record the original pose of the hand
	transf pos = mHand->getTran();

	double t[4][4];
	pos.toColMajorMatrix(t);
	vec3 dir (t[2][0],t[2][1],t[2][2]);
	//up direction is barrett's y axis
	vec3 up(t[1][0],t[1][1],t[1][2]);
	vec3 loc;

	// move the hand back to -5000;
	vec3 newLoc(pos.translation().x() -4800*dir[0],
		pos.translation().y() -4800*dir[1],
		pos.translation().z() -4800*dir[2]);
	transf newPose = transf(pos.rotation(), newLoc);
	graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->setTran(newPose);

	// look for bounding box
	std::vector<BoundingBox> bvs;
	vec3 center, corner;
	for(int i = 5000; i > 0; i -= 5) //one step back
	{
		vec3 tmp(newPose.translation().x() + i*dir[0],
			newPose.translation().y() + i*dir[1],
			newPose.translation().z() + i*dir[2]);
		graspItGUI->getIVmgr()->getWorld()->getBvs(mCurrentLoadedModel->getGraspableBody(), 0, &bvs);
		center = bvs[0].getTran().translation();
		//std::cout << "center is: " << center.x() << " " << center.y() << " " << center.z() << std::endl;
		//std::cout << "rotation is: " << bvs[0].getTran().rotation().x << " " << bvs[0].getTran().rotation().y << " " << bvs[0].getTran().rotation().z << std::endl;
		//std::cout << "halfsize is: " << bvs[0].halfSize.x() << " " << bvs[0].halfSize.y() << " " << bvs[0].halfSize.z() << std::endl;

		vec3 size = bvs[0].halfSize;// * bvs[0].getTran();

		bool isWithin = true;
		//check all the corneres of the bounding box to see if they are within the field of view
		for(int x = -1; x <2; x += 2){
			for(int y = -1; y <2; y += 2){
				for(int z = -1; z <2; z += 2)
				{
					corner = vec3(center.x() + x * size.x(), center.y() + y * size.y(), center.z() + z * size.z());
					if(!sm.checkWithin(tmp,corner,fov_vertical,fov_horizontal,up,dir))
						//if it is not within the fov continue to move backwards
						isWithin = false;
				}
			}
		}
		if(isWithin){
			loc = tmp;
			break;
		}
	}

	//std::cout << " " << "dir: " << dir.x() << " " << dir.y() << " " << dir.z() << "\n"  << 
	//"up: " << up.x() << " " << up.y() << " " << up.z() << "\n" <<
	//"loc: " << loc.x() << " " << loc.y() << " " << loc.z() << std::endl;

	//looking towards the object and save the scan
	sm.setupCameraPose(loc,dir,up, vec3( mCurrentLoadedModel->getGraspableBody()->getCoG().x(),
		mCurrentLoadedModel->getGraspableBody()->getCoG().y(),
		mCurrentLoadedModel->getGraspableBody()->getCoG().z()) );
	sm.scan2(mCurrentLoadedModel->GeometryPath(), "c:\\output_data\\graspit_test_scans\\", mCurrentLoadedModel->RescaleFactor());

	//move the hand back
	mHand->setTran(pos);

}

void SemanticPlannerDlg::scan1()
{
	std::string filename_suffix = mCurrentLoadedModel->ModelName();
	std::string fn_model, fn_raw;
	fn_model = filename_suffix;
	fn_model.append("_m.txt");
	fn_raw = filename_suffix;
	fn_raw.append("_r.txt");

	//this defines half the fov
	double fov_horizontal = 19.8;
	double fov_vertical = 23.75;
	sm.setOptics(fov_horizontal, fov_vertical, 144, 176);

	transf pos = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getTran();
	double t[4][4];
	pos.toColMajorMatrix(t);
	vec3 dir (t[2][0],t[2][1],t[2][2]);
	//up direction is barrett's y axis
	vec3 up(t[1][0],t[1][1],t[1][2]);
	vec3 loc;

	// look for bounding box
	std::vector<BoundingBox> bvs;
	vec3 center, corner;
	for(int i = 5000; i > 0; i -= 5) //one step back
	{
		vec3 tmp(pos.translation().x() + i*dir[0],
			pos.translation().y() + i*dir[1],
			pos.translation().z() + i*dir[2]);
		graspItGUI->getIVmgr()->getWorld()->getBvs(mCurrentLoadedModel->getGraspableBody(), 0, &bvs);
		center = bvs[0].getTran().translation();
		//std::cout << "center is: " << center.x() << " " << center.y() << " " << center.z() << std::endl;
		//std::cout << "rotation is: " << bvs[0].getTran().rotation().x << " " << bvs[0].getTran().rotation().y << " " << bvs[0].getTran().rotation().z << std::endl;
		//std::cout << "halfsize is: " << bvs[0].halfSize.x() << " " << bvs[0].halfSize.y() << " " << bvs[0].halfSize.z() << std::endl;

		vec3 size = bvs[0].halfSize * bvs[0].getTran();

		bool isWithin = true;
		//check all the corneres of the bounding box to see if they are within the field of view
		for(int x = -1; x <2; x += 2){
			for(int y = -1; y <2; y += 2){
				for(int z = -1; z <2; z += 2)
				{
					corner = vec3(center.x() + x * size.x(), center.y() + y * size.y(), center.z() + z * size.z());
					if(!sm.checkWithin(tmp,corner,fov_vertical,fov_horizontal,up,dir))
						//if it is not within the fov continue to move backwards
						isWithin = false;
				}
			}
		}
		if(isWithin){
			loc = tmp;
			break;
		}
	}

	//std::cout << " " << "dir: " << dir.x() << " " << dir.y() << " " << dir.z() << "\n"  << 
	//"up: " << up.x() << " " << up.y() << " " << up.z() << "\n" <<
	//"loc: " << loc.x() << " " << loc.y() << " " << loc.z() << std::endl;

	//looking towards the object and save the scan
	sm.setupCameraPose(loc,dir,up, vec3( mCurrentLoadedModel->getGraspableBody()->getCoG().x(),
		mCurrentLoadedModel->getGraspableBody()->getCoG().y(),
		mCurrentLoadedModel->getGraspableBody()->getCoG().z()) );
	sm.scanToFile(fn_raw, fn_model);

#define SAVE_PARAM
#ifdef SAVE_PARAM
	FILE *fp;
	std::string fn;
	fn.append("c:\\output_data\\task_");
	fn.append(mCurrentLoadedModel->ModelName().c_str());
	fn.append(".txt");
	//std::cout << fn.c_str() << std::endl;
	fp = fopen(fn.c_str(),"a");

	//print the output name
	fprintf(fp,"%s\n",filename_suffix.c_str());
	//print the scaling factor
	fprintf(fp,"%lf\n",mCurrentLoadedModel->RescaleFactor());
	//print out the center of gravity
	position p = mCurrentLoadedModel->getGraspableBody()->getCoG();
	fprintf(fp,"%lf %lf %lf\n",p.x(), p.y(), p.z());

	sm.saveParamsToFile(fp);
	fclose(fp);
#endif
}

void SemanticPlannerDlg::displayArrow(int idx)
{

	position cog = mCurrentLoadedModel->getGraspableBody()->getCoG();
	std::vector<double> originalLongitudeLatitude = mSemanticMap.getLongitudeLatitude(idx);
	transf t = mSemanticMap.getOriginalPose(originalLongitudeLatitude[0], originalLongitudeLatitude[1], vec3(cog[0],cog[1],cog[2]));



	graspItGUI->getIVmgr()->getWorld()->getIVRoot()->removeChild(aSep);
	aSep = new SoSeparator;
	aRot = new SoRotation;
	bRot = new SoRotation;
	aTransl = new SoTranslation;
	arrow = new SoArrow;
	arrow->height = 28;
	arrow->cylRadius = 2.5;
	arrow->coneRadius = 5.25;
	arrow->coneHeight = 11;
	//t = graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->getPalm()->getTran();
	vec3 axis;
	double angle;
	t.rotation().ToAngleAxis(angle, axis);

	bRot->rotation.setValue(SbRotation(SbVec3f(axis[0],axis[1],axis[2]), angle));

	aRot->rotation.setValue(SbRotation(SbVec3f(1,0,0), 0.5*M_PI));

	aTransl->translation.setValue(SbVec3f(t.translation().x(), t.translation().y(), t.translation().z()));

	//then apply the hand's trnaslation
	aSep->addChild(aTransl);
	//then apply the hand's rotation
	aSep->addChild(bRot);
	//first rotate so that the arrow matches with the hand's approach direction
	aSep->addChild(aRot);
	aSep->addChild(arrow);

	graspItGUI->getIVmgr()->getWorld()->getIVRoot()->addChild(aSep);
}

void SemanticPlannerDlg::collectTactileFeedback()
{
#if 0
	////generate file name
	//std::string fname("c:\\tactile_output\\");
	//fname.append(mCurrentLoadedModel->ModelName());
	////fname.append("_");
	////itoa(j,num,10);
	////fname.append(num);
	//fname.append(".txt");
	//FILE * fresult = fopen(fname.c_str(),"w");
	//QTextStream qts(fresult);

	QString sensorReading;
	QTextStream qts(&sensorReading);

	World * w = graspItGUI->getIVmgr()->getWorld();
	for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++){
		qts << sensorInd << " ";
		w->getSensor(sensorInd)->updateSensorModel();
		w->getSensor(sensorInd)->outputSensorReadings(qts);
	}
	std::cout << sensorReading.latin1() << std::endl;

	//fclose(fresult);

#endif

	std::vector<double> t = getTactileSensorReadings();
	for(size_t i = 0; i < t.size(); ++i)
	{
		std::cout << t[i] << "\t";
	}
	std::cout << std::endl;



}

void SemanticPlannerDlg::update()
{
	if(mPlanner->getListSize() == 0)
	{
		//std::cout << "no solution so far\n";
		return;
	}
	//std::cout << mPlanner->getListSize() << std::endl;
	char solutionStatus[128];
	sprintf(solutionStatus,"%d / %d", mCurrentSolution, mPlanner->getListSize());
	ui->solutionLabel->setText(solutionStatus);
	if(ui->showArrowCheckBox->isChecked())
		const_cast<GraspPlanningState*>(mPlanner->getGrasp(mCurrentSolution))->showVisualMarker();
	else{
		if(aSep)
		{
			graspItGUI->getIVmgr()->getWorld()->getIVRoot()->removeChild(aSep);
			aSep = NULL;
		}

		mHand->showVirtualContacts(false);
		mCurrentLoadedModel->getGraspableBody()->showAxes(false);
		const_cast<GraspPlanningState*>(mPlanner->getGrasp(mCurrentSolution))->hideVisualMarker();
	}
	//if(mPlanner->getListSize() < mNumGrasps)
	//	return;
	//stop the planner in 2 seconds
	//int mMaxTime = 2;// this is a trick, maybe not good
	//DBGP("begin to leave...");
	//mPlanner->setMaxTime(mMaxTime);
}

void SemanticPlannerDlg::complete()
{
	//mNext = true;
	//schedule the sensor to give us the exit signal in 3 seconds
	//this should give the planner time to finish
	//if(mTimerSensor) mTimerSensor->unschedule();
	//delete mTimerSensor;
	//mTimerSensor = new SoTimerSensor(sensorCB, this);
	//mTimerSensor->setInterval( SbTime( 3.0 ));
	//mTimerSensor->schedule();
}

void SemanticPlannerDlg::sensorCB(void *data, SoSensor*)
{
	//called when a planning is done
	//
}

void SemanticPlannerDlg::plan()
{
	std::cout << "begin to plan" << std::endl;
	//initialize a planner
	mPlanner = new GuidedPlanner(mHand);
	//mPlanner = new SimAnnPlanner(mHand);
	//set parameters
	mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
	mPlanner->setContactType(CONTACT_PRESET);
	mPlanner->setMaxSteps(70000);

	mHandObjectState = new GraspPlanningState(mHand);
	mHandObjectState->setObject(mCurrentLoadedModel->getGraspableBody());
	//constrain the hand to the approach direction
	mHandObjectState->setPositionType(SPACE_APPROACH);
	mHandObjectState->setPostureType(POSE_EIGEN);
	transf refTran = mHand->getTran();
	mHandObjectState->setRefTran(refTran, false);
	mHandObjectState->reset();
	((GuidedPlanner*)mPlanner)->setModelState(mHandObjectState);


	//set the reference hand-object-state
	FILE* fp = fopen(mSemanticMap.getSemanticGraspList()[0].getGraspParamsFilePath().c_str(),"r");
	bool success = true;
	if (!fp) {
		DBGA("Failed to open input file!");
		success = false;
	}else if (!mPlanner->getTargetState()->readFromFile(fp)) {
		DBGA("Failed to read target from input file!");
		success = false;
	} else {
		DBGA("Target values loaded successfully");
	}
	fclose(fp);
	mPlanner->setInput(INPUT_FILE, success);

	//EG 0 and EG 1 should be loaded from the external file
	mPlanner->getTargetState()->getVariable("EG 0")->setConfidence(0.999);
	mPlanner->getTargetState()->getVariable("EG 0")->setFixed(true);
	//the other joint angles can be more free
	mPlanner->getTargetState()->getVariable("EG 1")->setConfidence(0.5);
	mPlanner->getTargetState()->getVariable("EG 1")->setFixed(false);

	QObject::connect(mPlanner,SIGNAL(update()),this,SLOT(update()));
	QObject::connect(mPlanner,SIGNAL(complete()),this,SLOT(complete()));
	mPlanner->resetPlanner();
	mPlanner->startPlanner();
}