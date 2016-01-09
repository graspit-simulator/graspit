//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: graspRecord.cpp,v 1.3 2009/04/21 14:53:08 cmatei Exp $
//
//######################################################################

#include "graspRecord.h"
#include "gloveInterface.h"
#include "debug.h"


// add locale in order to back up existing locale settings and force them to US
// when using fprintf and scanf commands
#include <locale.h>

GraspRecord::GraspRecord(int size)
{
	mSize = size;
	mPose = new CalibrationPose(mSize);

	mTran = transf::IDENTITY;
	mObjectName = mRobotName = QString("not_set");
}

GraspRecord::~GraspRecord()
{
	delete mPose;
}

void GraspRecord::writeToFile(FILE *fp)
{

    // change locale to make sure all floats in the files read (e.g. with fscanf) are 
    // expected to have a "dot" floating point (not comma). Backup previous locale setting
    // first in order to restore current setting below.
    std::locale usLocale("en_US.UTF-8");
    std::locale previousLocale=std::locale::global(usLocale);

	//write names
	fprintf(fp,"%s\n",mObjectName.latin1());
	fprintf(fp,"%s\n",mRobotName.latin1());
	//write transform
	Quaternion q = mTran.rotation();
	fprintf(fp,"%f %f %f %f ",q.x, q.y, q.z, q.w);
	vec3 t = mTran.translation();
	fprintf(fp,"%f %f %f\n",t.x(), t.y(), t.z());
	//write pose
	mPose->writeToFile(fp);

    std::locale::global(previousLocale);
}

void GraspRecord::readFromFile(FILE *fp)
{
	float x,y,z,w;
	//read names
	char name[1000];
	do {
	  if(fgets(name, 1000, fp) == NULL) {
	    DBGA("GraspRecord::readFromFile - failed to read record name");
	    return;
	  } 
	} while (name[0]=='\n' || name[0]=='\0' || name[0]==' ');
	mObjectName = QString(name);
	mObjectName = mObjectName.stripWhiteSpace();
	fprintf(stderr,"object: %s__\n",mObjectName.latin1());
	do {
	  if(fgets(name, 1000, fp) == NULL) {
	    DBGA("GraspRecord::readFromFile - failed to read robot name");
	    return;
	  }
	} while (name[0]=='\n' || name[0]=='\0' || name[0]==' ');
	mRobotName = QString(name);
	mRobotName = mRobotName.stripWhiteSpace();
	fprintf(stderr,"robot: %s__\n",mRobotName.latin1());

    // change locale to make sure all floats in the files read (e.g. with fscanf) are 
    // expected to have a "dot" floating point (not comma). Backup previous locale setting
    // first in order to restore current setting below.
    std::locale usLocale("en_US.UTF-8");
    std::locale previousLocale=std::locale::global(usLocale);

	//read transform
	if( fscanf(fp,"%f %f %f %f",&x, &y, &z, &w) <= 0) {
	  DBGA("GraspRecord::readFromFile - failed to read record orientation.");
	  return;
	}
	Quaternion q(w, x, y, z);
	if( fscanf(fp,"%f %f %f",&x, &y, &z) <= 0) {
	  DBGA("GraspRecord::readFromFile - failed to read record location");
	}

    std::locale::global(previousLocale);

	vec3 t(x,y,z);
	mTran.set(q,t);
	//read pose
	mPose->readFromFile(fp);
	mSize = mPose->getSize();
}

void loadGraspListFromFile(std::vector<GraspRecord*> *list, const char *filename)
{
	FILE *fp = fopen(filename, "r");
	if (fp==NULL) {
		fprintf(stderr,"Unable to open file %s for reading\n",filename);
		return;
	}

    // change locale to make sure all floats in the files read (e.g. with fscanf) are 
    // expected to have a "dot" floating point (not comma). Backup previous locale setting
    // first in order to restore current setting below.
    std::locale usLocale("en_US.UTF-8");
    std::locale previousLocale=std::locale::global(usLocale);

	GraspRecord *newGrasp;
	int nGrasps;
	if(fscanf(fp,"%d",&nGrasps)) { 
	  DBGA("loadGraspListFromFile - failed to read number of grasps");
      std::locale::global(previousLocale);
	  return;
	}

    std::locale::global(previousLocale);

	for(int i=0; i<nGrasps; i++) {
		newGrasp = new GraspRecord(0);
		newGrasp->readFromFile(fp);
		list->push_back(newGrasp);
	}

	fclose(fp);
}

void writeGraspListToFile (std::vector<GraspRecord*> *list, const char *filename)
{
	FILE *fp = fopen(filename, "w");
	if (fp==NULL) {
		fprintf(stderr,"Unable to open file %s for reading\n",filename);
		return;
	}

    // change locale to make sure all floats in the files read (e.g. with fscanf) are 
    // expected to have a "dot" floating point (not comma). Backup previous locale setting
    // first in order to restore current setting below.
    std::locale usLocale("en_US.UTF-8");
    std::locale previousLocale=std::locale::global(usLocale);

	fprintf(fp,"%d\n",(int)list->size());
	for (int i=0; i<(int)list->size(); i++) {
		(*list)[i]->writeToFile(fp);
	}

    std::locale::global(previousLocale);

	fclose(fp);
}
