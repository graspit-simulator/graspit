#include "Arizona_Raw_Exp.h"
#include "qfile.h"
#include "qtextstream.h"

#include "debug.h"

#define EXPECTED 30

ArizonaRawExp::ArizonaRawExp(){
	mCenter = vec3(0.0,0.0,1.0);
	mRadius = 34.4;
}

ArizonaRawExp::~ArizonaRawExp(){
		mFingerTips.clear();
		mMarkers.clear();
		mNormals.clear();
		mContacts.clear();
		mQuats.clear();
}

bool ArizonaRawExp::parse(QString line){
//	std::cout << "begin to parse" << std::endl;
	char sep = '\t';
	if(line.count(sep) < EXPECTED){
		std::cout << "Wrong file format at line: " << line.latin1() << std::endl;
		return false;
	}
	mID = atoi(line.section(sep,0,0));
	
	QString condition = line.section(sep,1,1);
	if(!strcmp(condition.latin1(),"RANDOM")) {
		mCondition = RANDOM;
	} else if ( !strcmp(condition.latin1(), "BLOCKED") ) {
		mCondition = BLOCKED;
	} else if ( !strcmp(condition.latin1(), "TEST") ) {
		mCondition = TEST;
	} else {
		std::cout << "Failed to understand experiment type: " << condition.latin1() << std::endl;;
		return false;
	}

	mCenterOfMassPattern = atoi(line.section(sep,2,2));
	mTrial = atoi(line.section(sep,3,3));

	for(int i = 0; i < 3; i ++){ // five fingers
		QString temp = line.section(4 + 3*i,4+3*i);
		mFingerTips.push_back(vec3(line.section(sep,4+i*3,4+i*3).toDouble(),
			line.section(sep,5+i*3,5+i*3).toDouble(),
			line.section(sep,6+i*3,6+i*3).toDouble()));
	}

	for(int i = 0; i < 4; i ++){ // four markers
		mMarkers.push_back(vec3(line.section(sep,19+i*3,19+i*3).toDouble(),
			line.section(sep,20+i*3,20+i*3).toDouble(),
			line.section(sep,21+i*3,21+i*3).toDouble()));
	}

//	std::cout << "parsed successfully" << std::endl;
	return true;
}

vec3 ArizonaRawExp::getCenter(){
	vec3 temp = getBase();
	temp = vec3 (temp.x(), temp.y(), temp.z() + 89.79);
	return temp;
}

vec3 ArizonaRawExp::getBase(){
	vec3 temp = (mMarkers[2] + mMarkers[3]) / 2.0;
	temp = vec3 (temp.x(), temp.y() - 90.0, temp.z() - 3.0);//specify the offset
	return temp;
}

vec3 ArizonaRawExp::normalizePoint(vec3 center, vec3 point, double radius){
	vec3 temp;
	temp = point - center;
	if(fabs(temp.len() - radius) > 15.0)
		DBGA("large offset for contact found: " << temp.len() - radius);
	temp = radius * normalise(temp);
	return center + temp;
}

vec3 ArizonaRawExp::getNormal(vec3 center, vec3 point){
	vec3 temp = point - center;
	return normalise(temp);
}

Quaternion ArizonaRawExp::getQuaternion(vec3 orientation){
	vec3 temp;
	temp = vec3(0,0,1.0) * orientation;
	temp = normalise(temp);
	double angle;
	angle = acos(orientation % vec3(0,0,1.0));
	angle = angle > 0 ? angle: 180 + angle;
	Quaternion quat = Quaternion(angle,temp);
//	std::cout << "begin to print" << std::endl;
//	std::cout << quat.w << " " << quat.x << " " << quat.y << " " << quat.z << std::endl;
	return quat;
}

Quaternion ArizonaRawExp::getQuaternion(int i){
	return mQuats[i];
}

void ArizonaRawExp::processLine(){
	mCenter = getCenter();
	mBase = getBase();

	setContacts();
	setNormals();
	setQuaternions();
}

void ArizonaRawExp::setContacts(){
	mContacts.clear();
	for(int i = 0; i <(int)mFingerTips.size(); i ++){
		mContacts.push_back(normalizePoint(mCenter, mFingerTips[i], mRadius) - mBase);
	}
}

void ArizonaRawExp::setNormals(){
	mNormals.clear();
	for(int i = 0; i < (int)mFingerTips.size(); i ++){
		mNormals.push_back(getNormal(mCenter,mFingerTips[i]));
	}
}

void ArizonaRawExp::setQuaternions(){
	mQuats.clear();
	for(int i = 0; i < (int)mFingerTips.size(); i ++){
		mQuats.push_back(getQuaternion(getNormal(mCenter,mFingerTips[i])));
	}
}

void ArizonaRawExp::writeToFile(QString ellipsoidFileName, QString outputFileName){
	FILE *fp;
	fp = fopen(outputFileName.latin1(),"w");
	fprintf(fp, "%d\n", (int)mFingerTips.size());
	for(int i = 0; i < (int)mFingerTips.size(); i++){
        printEllipsoid(ellipsoidFileName,fp);
		fprintf(fp,"%f %f %f %f\n", mQuats[i].w, mQuats[i].x, mQuats[i].y, mQuats[i].z);
		fprintf(fp,"%f %f %f\n", mContacts[i].x(), mContacts[i].y(), mContacts[i].z());
		fprintf(fp,"%f %f %f\n", mNormals[i].x(), mNormals[i].y(), mNormals[i].z());
		fprintf(fp,"1\n\n");
	}
	fclose(fp);
}

void ArizonaRawExp::printEllipsoid(QString ellipsoidFileName, FILE *fp){
	QFile file(ellipsoidFileName);
	QString line;

	if( !file.open(IO_ReadOnly )){
		std::cout << "failed to open the file: " << ellipsoidFileName.latin1() << std::endl;
		return;
	}
	QTextStream stream( &file );
	while ( !stream.atEnd() ) 
	{
		line = stream.readLine();
		fputs(line.latin1(),fp);
		fputc('\n',fp);
	}
	file.close();
}