#include "Arizona_Process.h"
#include "qfile.h"
#include "qtextstream.h"
#include "qstring.h"
#include "contact.h"
#include "debug.h"

ArizonaProcess::ArizonaProcess()
{
	mAt = new ArizonaTest();
	mCurrent = 0;
	mTotal = 0;
}

ArizonaProcess::~ArizonaProcess()
{
	if(mAt)
		delete mAt;
	for(int i = 0; i < (int)mRawData.size(); i ++){
		delete mRawData[i];
	}
	mRawData.clear();
}

bool ArizonaProcess::load_and_preprocess(QString rawFileName){
	QFile file(rawFileName);
	QString line;

	for(int i = 0; i < (int)mRawData.size(); i ++){
		delete mRawData[i];
	}
	mRawData.clear();

	if( !file.open(IO_ReadOnly )){
		std::cout << "failed to open the file: " << rawFileName.latin1() << std::endl;
		return FALSE;
	}
	QTextStream stream( &file );
	while ( !stream.atEnd() ) 
	{
		line = stream.readLine();
		ArizonaRawExp *are = new ArizonaRawExp();
		if(are->parse(line)){ //  successfully parsed
			are->processLine();
			mRawData.push_back(are);
		} else {
			std::cout << "invalid line detected: " << line.latin1() << std::endl;
			delete are;
		}
	}
	mTotal = (int)mRawData.size();
	std::cout << "successfully loaded " << mRawData.size() << "lines of data" << std::endl;
	return true;
}

void ArizonaProcess::saveExpToVGR(QString rawFileName, QString ellipsoidFileName, QString destinationFileFolder){

	QString name = rawFileName.section('/',-1);
	name = name.section('.',0,0);
	for(int i = 0; i < (int) mRawData.size(); i ++){
		QString postfix;
		postfix.setNum(i + 1);
		QString fn = destinationFileFolder + QString("/") + name + postfix + QString(".vgr");
		mRawData[i]->writeToFile(ellipsoidFileName,fn);
	}
	std::cout << "virtual grasp file saved..." << std::endl;
	return;
}

void ArizonaProcess::setObject(GraspableBody *object){
	mAt->setObject(object);
}

ArizonaRawExp* ArizonaProcess::getNextRawData(){
	mCurrent ++;
	mCurrent %= (int)mRawData.size();
	return mRawData[mCurrent];
}

ArizonaRawExp* ArizonaProcess::getCurrentRawData(){
	return mRawData[mCurrent];
}

ArizonaRawExp* ArizonaProcess::getPrevRawData(){
	mCurrent --;
	if(mCurrent < 0)
		mCurrent += (int)mRawData.size();
	return mRawData[mCurrent];
}

void ArizonaProcess::testAllQual(bool isBuildIn3D, bool isFlipped, QString ellipsoidFileName, QString fn){
	FILE *fp;
	fp = fopen(fn.latin1(), "w");
	for(int i = 0; i < (int)mRawData.size(); i ++){
		mAt->setTestData(mRawData[i], ellipsoidFileName, isBuildIn3D, isFlipped);
		mAt->initializeTest();
		mAt->writeQuality2File(fp);
	}
	fclose(fp);
}

void ArizonaProcess::testAllForce(bool isBuildIn3D, bool isFlipped, QString ellipsoidFileName, QString fn){
	FILE* fp;
	fp = fopen(fn.latin1(), "w");
	for(int i = 0; i < (int) mRawData.size(); i ++){
		mAt->setTestData(mRawData[i],ellipsoidFileName,isBuildIn3D, isFlipped);
		mAt->initializeTest();
		mAt->writeForce2File(fp);
	}
	fclose(fp);
}

void ArizonaProcess::writeAllContact(bool isBuildIn3D, bool isFlipped, QString ellipsoidFileName, QString fn){
	FILE* fp;
	fp = fopen(fn.latin1(), "w");
	for(int i = 0; i < (int) mRawData.size(); i ++){
		mAt->setTestData(mRawData[i],ellipsoidFileName,isBuildIn3D, isFlipped);
		mAt->initializeTest();
		mAt->writeContact2File(fp);
	}
	fclose(fp);
}