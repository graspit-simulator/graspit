#include "eigenGridsDlg.h"

#include <QFileDialog>

#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoPackedColor.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoScale.h>

#include <fstream>
#include <vector>

#include "body.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"

#include "octree.h"
#include "dataTypes.h"

void EigenGridsDlg::init()
{
	mMean = NULL;
	mDisplay = NULL;
	typeBox->insertItem("Binary");
	typeBox->insertItem("Carved");
	slider1->setRange(-100,100); slider1->setValue(0); slider1->setSteps(1,10);
	slider2->setRange(-100,100); slider2->setValue(0); slider2->setSteps(1,10);
	slider3->setRange(-100,100); slider3->setValue(0); slider3->setSteps(1,10);
	slider4->setRange(-100,100); slider4->setValue(0); slider4->setSteps(1,10);
	slider5->setRange(-100,100); slider5->setValue(0); slider5->setSteps(1,10);
	slider6->setRange(-100,100); slider6->setValue(0); slider6->setSteps(1,10);
	updateControls();
	gDisplayBody = NULL;
}


void EigenGridsDlg::destroy()
{
	reset();
	//TODO: fix this!!!
	/*	
	if (gDisplayBody) {
		World *world = graspItGUI->getIVmgr()->getWorld();
		//this should also delete it 
		world->destroyElement(gDisplayBody);
		gDisplayBody = NULL;
	}
	*/	
}

void EigenGridsDlg::reset()
{
	if (mMean) delete mMean; mMean = NULL;
	if (mDisplay) delete mDisplay; mDisplay = NULL;

	while (!mEigens.empty()) {
		delete mEigens.back();
		mEigens.pop_back();
	}
}

void EigenGridsDlg::updateControls() {
	if (mMean) loadButton->setEnabled(TRUE);
	else loadButton->setEnabled(FALSE);
	
	if ( mEigens.size() >= 7) loadButton->setEnabled(FALSE);

	if (mMean) {
		goButton->setEnabled(TRUE);
	} else {
		goButton->setEnabled(FALSE);
	}
	
	connect(slider1,SIGNAL(sliderMoved(int)), this, SLOT(goButton_clicked()) );	
	if ( mEigens.size() >= 1 ) slider1->setEnabled(TRUE);
	else slider1->setEnabled(FALSE);
	connect(slider2,SIGNAL(sliderMoved(int)), this, SLOT(goButton_clicked()) );	
	if ( mEigens.size() >= 2 ) slider2->setEnabled(TRUE);
	else slider2->setEnabled(FALSE);
	connect(slider3,SIGNAL(sliderMoved(int)), this, SLOT(goButton_clicked()) );	
	if ( mEigens.size() >= 3 ) slider3->setEnabled(TRUE);
	else slider3->setEnabled(FALSE);
	connect(slider4,SIGNAL(sliderMoved(int)), this, SLOT(goButton_clicked()) );	
	if ( mEigens.size() >= 4 ) slider4->setEnabled(TRUE);
	else slider4->setEnabled(FALSE);
	connect(slider5,SIGNAL(sliderMoved(int)), this, SLOT(goButton_clicked()) );	
	if ( mEigens.size() >= 5 ) slider5->setEnabled(TRUE);
	else slider5->setEnabled(FALSE);
	connect(slider6,SIGNAL(sliderMoved(int)), this, SLOT(goButton_clicked()) );	
	if ( mEigens.size() >= 6 ) slider6->setEnabled(TRUE);
	else slider6->setEnabled(FALSE);
}

void EigenGridsDlg::exitButton_clicked()
{
	QDialog::accept();
}


void EigenGridsDlg::loadButton_clicked()
{
	scan_utils::Octree<float>* octree = loadNewOctree();
	assert(mMean);
	if (octree) {
		if (octree->getMaxDepth() != mMean->getMaxDepth()) {
			fprintf(stderr,"New tree has different depth than mean tree!\n");
			delete octree;
			return;
		}
		mEigens.push_back(octree);
	}
	updateControls();
}


void EigenGridsDlg::loadMeanButton_clicked()
{
	scan_utils::Octree<float>* octree = loadNewOctree();
	if (octree) {
		if (mMean) delete mMean;
		mMean = octree;
	}
	updateControls();
}


void EigenGridsDlg::clearButton_clicked()
{
	reset();
	updateControls();
}


void EigenGridsDlg::updateDisplayGrid()
{
	if (!mDisplay) {
		//this should not be hard-coded. Maybe a copy constructor?
		//use an empty value of -1.0 to be rendered as empty for carved grids
		mDisplay = new scan_utils::Octree<float>(0, 0, 0, 320, 320, 320, 5, (float)-1.0);
	}
	if (mDisplay->getMaxDepth() != mMean->getMaxDepth()) {
		mDisplay->clear();
		mDisplay->setMaxDepth( mMean->getMaxDepth() );
	}
	std::vector<int> amplitudes;
	if ( mEigens.size() >= 1 ) amplitudes.push_back( slider1->value() );
	if ( mEigens.size() >= 2 ) amplitudes.push_back( slider2->value() );
	if ( mEigens.size() >= 3 ) amplitudes.push_back( slider3->value() );
	if ( mEigens.size() >= 4 ) amplitudes.push_back( slider4->value() );
	if ( mEigens.size() >= 5 ) amplitudes.push_back( slider5->value() );
	if ( mEigens.size() >= 6 ) amplitudes.push_back( slider6->value() );

	for (int i=0; i<20; i++) {
		for (int j=0; j<20; j++) {
			for (int k=0; k<20; k++) {
				float val = mMean->cellGet(i+6,j+6,k+6);
				for (int grid = 0; grid < (int)mEigens.size(); grid++) {
					val += amplitudes[grid] * mEigens[grid]->cellGet(i+6, j+6, k+6);
				}
				mDisplay->cellInsert(i+6, j+6, k+6, val);
			}
		}
	}
}

bool isOccupied(float f){return f > (float)0.5;}
bool isEmpty(float f){return f < (float)-0.5;}
bool isUnknown(float f){ return !isOccupied(f) && !isEmpty(f);}

void EigenGridsDlg::showDisplayGrid()
{
	if (!gDisplayBody) createDisplayBody();
	gDisplayBody->getIVGeomRoot()->removeAllChildren();
	//create and display triangles
	std::list<scan_utils::Triangle> triangles;

	//	mDisplay->getTriangles(triangles, &isOccupied);
	mDisplay->getCellSliceTriangles(triangles, &isOccupied, -1, -1, 16);
	addTriangles(gDisplayBody, &triangles, 1.0f, 0.0f, 0.0f);
	triangles.clear();

	if (typeBox->currentText() == "Carved") {
	    //mDisplay->getTriangles( triangles, &isUnknown );
	    mDisplay->getCellSliceTriangles(triangles, &isUnknown, -1, -1, 16);
		addTriangles(gDisplayBody, &triangles, 0.0f, 0.0f, 1.0f);
		triangles.clear();
	}
}

void EigenGridsDlg::goButton_clicked()
{
	updateDisplayGrid();
	showDisplayGrid();
}

void EigenGridsDlg::createDisplayBody()
{
	World *world = graspItGUI->getIVmgr()->getWorld();
	if (gDisplayBody) {
		//this should also delete it 
		world->destroyElement(gDisplayBody);
	}
	gDisplayBody = new Body(world,"Eigengrids");
	gDisplayBody->addIVMat();
	gDisplayBody->addToIvc();
	world->addBody(gDisplayBody);
	fprintf(stderr,"Display body created!\n");
}

scan_utils::Octree<float>* EigenGridsDlg::loadNewOctree()
{

    QString fn( QFileDialog::getOpenFileName(this, QString(), 
											 QString("/data/simulated_grasps_scans/"),
											 "Octree files (*.txt)") );
	if (fn.isEmpty()) return NULL;
	std::fstream fs;
	fs.open(fn.latin1(),std::fstream::in);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",fn.latin1());
		return NULL;
	}

	scan_utils::Octree<float>* octree = new scan_utils::Octree<float>(0,0,0,0,0,0,1,(float)0.0);
	if (!octree->readFromFile(fs)) {
		fprintf(stderr,"Failed to read octree from file\n");
		delete octree; octree = NULL;
	}
	fs.close();
	return octree;
}


/* This code is duplicated from ScanDlg.ui.h so it should probably be placed somewhere else
 */
void EigenGridsDlg::addTriangles(Body *body, std::list<scan_utils::Triangle> *triangles, float r, float g, float b)
{
	unsigned int nTri = (unsigned int) triangles->size();

	SbVec3f *points = new SbVec3f[ 3*nTri ];
	int32_t *cIndex = new int32_t[ 4*nTri ];

	if (!points || !cIndex) {
		fprintf(stderr,"Failed to allocate memory for points and indeces!\n");
		triangles->clear(); delete triangles;
		if (points) delete [] points; if (cIndex) delete [] cIndex;
		return;

	}

	unsigned int i = 0;
	std::list<scan_utils::Triangle>::iterator it = triangles->begin();
	while(it!=triangles->end()) {

		points[3*i+0] = SbVec3f( (*it).p1.x,  (*it).p1.y,  (*it).p1.z);
		points[3*i+1] = SbVec3f( (*it).p2.x,  (*it).p2.y,  (*it).p2.z);
		points[3*i+2] = SbVec3f( (*it).p3.x,  (*it).p3.y,  (*it).p3.z);

		cIndex[4*i+0]=3*i+0;
		cIndex[4*i+1]=3*i+1;
		cIndex[4*i+2]=3*i+2;
		cIndex[4*i+3]=-1;

		it++;
		i++;
	}

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 3*nTri, points );

	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 4*nTri, cIndex);

	SoMaterial *mat = new SoMaterial;
	mat->diffuseColor = SbColor(r, g, b);
	mat->ambientColor = SbColor(r/4.0, g/4.0, b/4.0);
	mat->emissiveColor = SbColor(r/2.0, g/2.0, b/2.0);
	mat->transparency = 0.0f;

	body->getIVGeomRoot()->addChild(mat);
	body->getIVGeomRoot()->addChild(coords);
	body->getIVGeomRoot()->addChild(ifs);
}
