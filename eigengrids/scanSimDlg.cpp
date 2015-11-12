#include "scanSimDlg.h"

#include <Q3FileDialog>

#include <fstream>
#include <vector>

#include "dataTypes.h"

#include "scanSimulator.h"
#include "matvec3D.h"

void ScanSimDlg::exitButton_clicked()
{
	QDialog::accept();
}

void ScanSimDlg::init()
{
	pxEdit->setText("100");
	pyEdit->setText("80");
	pzEdit->setText("0");
	dxEdit->setText("-1");
	dyEdit->setText("-1");
	dzEdit->setText("0");
	uxEdit->setText("0");
	uyEdit->setText("1");
	uzEdit->setText("0");
}

void ScanSimDlg::destroy()
{
	//yes, I know, it leaks memory... should delete mScan, but I need to return it...
}

void ScanSimDlg::goButton_clicked()
{
	bool ok;
	float px = pxEdit->text().toFloat(&ok);
	float py = pyEdit->text().toFloat(&ok);
	float pz = pzEdit->text().toFloat(&ok);
	float dx = dxEdit->text().toFloat(&ok);
	float dy = dyEdit->text().toFloat(&ok);
	float dz = dzEdit->text().toFloat(&ok);
	float ux = uxEdit->text().toFloat(&ok);
	float uy = uyEdit->text().toFloat(&ok);
	float uz = uzEdit->text().toFloat(&ok);
	if (!ok) {
		fprintf(stderr,"Parameter conversion failed\n");
		return;
	}

	ScanSimulator *sim = new ScanSimulator();
	sim->setPosition( position(px,py,pz) , vec3(dx,dy,dz), vec3(ux,uy,uz) );
	//SCANNER_COORDINATES is default
	sim->setType(ScanSimulator::WORLD_COORDINATES);

	std::vector<position> cloud;
	std::vector<RawScanPoint> rawData;
	sim->scan(&cloud, &rawData);
	
	int numPts = cloud.size();
	fprintf(stderr,"Scanned %d points\n",numPts);
	float *pts = new float[ 3 * numPts ];
	for (int i=0; i<numPts; i++) {
		pts[3*i+0] = cloud[i].x();
		pts[3*i+1] = cloud[i].y();
		pts[3*i+2] = cloud[i].z();
	}
	
	/*
	int numPts = rawData.size();
	fprintf(stderr,"Raw data %d points\n",numPts);
	float *pts = new float[ 3 * numPts ];
	for (int i=0; i<numPts; i++) {
		vec3 dir;
		sim->computeRayDirection( rawData[i].hAngle, rawData[i].vAngle, dir);
		if ( rawData[i].distance > 0) {
			dir = rawData[i].distance * normalise(dir);
		} else {
			dir = 1000 * normalise(dir);
		}
		
		pts[3*i+0] = px + dir.x();
		pts[3*i+1] = py + dir.y();
		pts[3*i+2] = pz + dir.z();
	}
	*/
	/*
	//old version that used smartScan
	if (mScan) delete mScan;
	mScan = new SmartScan();
	mScan->setPoints(numPts, pts);
	position sp; vec3 sd, su;
	sim->getPosition(sp,sd,su);
	mScan->setScanner(sp.x(),sp.y(),sp.z(),
			  sd.x(),sd.y(),sd.z(),
			  su.x(),su.y(),su.z());
	*/
	delete [] pts;
	cloud.clear();
	rawData.clear();
	delete sim;
}

