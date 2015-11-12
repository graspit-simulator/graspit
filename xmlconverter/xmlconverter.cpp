//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Norases (Saint) Vesdapunt
//
// $Id: xmlconverter.cpp,v 1.5 2009/07/06 21:05:53 cmatei Exp $
//
//######################################################################

#include <QFile>
#include <QStringList>

#include "xmlconverter.h"
#include "tinyxml.h"

int main(int argc, char *argv[]){
	if(argc<2){
		DBGA("Usage: xmlconverter filename");
		return FAILURE;
	}
	for(int i=1;i<argc;i++){
		QString filename = argv[i];
		filename.replace("\\","/");
		DBGA("Converting "<<filename.latin1());
		QString fileType = filename.section('.',-1,-1);
		if(fileType == "cfg"){
			if(convertCfg2Xml(filename)==FAILURE){
				DBGA("Error Converting CFG to XML");
				return FAILURE;
			}
		}
		else if(fileType=="wld"){
			if(convertWld2Xml(filename)==FAILURE){
				DBGA("Error Converting  to XML");
				return FAILURE;		
			}
		}
		else if(fileType=="iv"||fileType=="IV"||fileType=="wrl"||fileType=="WRL"||fileType=="off"||fileType=="OFF"){
			if(convertObj2Xml(filename)==FAILURE){
				DBGA("Error Converting IV/WRL/OFF to XML");
				return FAILURE;		
			}
		} else {
			DBGA("Incompaptible file format: cfg/wld/iv/wrl/off only");
			return FAILURE;
		}
	}
	return SUCCESS;
}
int convertObj2Xml(QString filename){
	filename.replace("\\","/");
	QString fileType = filename.section('.',-1,-1);
	if(!(fileType=="iv"||fileType=="IV"||fileType=="wrl"||fileType=="WRL"||fileType=="off"||fileType=="OFF")){
		DBGA("Not an IV/WRL/OFF file");
		return FAILURE;
	}
	QString line,defStr,valueStr;
	QString loadI[9];
	QStringList l;
	bool ok;
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		DBGA("could not open object file: " << filename.toStdString());
		return FAILURE;
	}
	QTextStream stream(&file);
	QString	xml = "<?xml version=\"1.0\" ?>\n";
	xml += "<root>\n";
	double youngMod, mBirdNumber,loadMass;
	while (nextCommentLine(&stream, &line)) {
		defStr = line.section('#',1).section(' ',0,0);
		//material line
		if (defStr=="material")	{
			valueStr = line.section(' ',1,1);
			if (!valueStr.isEmpty()) {
				xml+="\t<material>"+valueStr+"</material>\n";
			}
		}
		//Young's modulus line
		if (defStr=="youngs"){
			valueStr = line.section(' ',1,1);
			if(!valueStr.isEmpty()){
				youngMod = valueStr.toDouble(&ok);
				if (!ok || youngMod <= 0) {
					DBGA("Invalid Young's modulus");
					file.close();
					return FAILURE;
				}
				xml+="\t<youngs>"+valueStr+"</youngs>\n";
			}	
		}

		if (defStr=="useFlockOfBirds") {
			valueStr = line.section(' ',1,1);
			if(valueStr.isEmpty()){
				DBGA("Bird Number not specified.");
				file.close();
				return FAILURE;
			}
			mBirdNumber=valueStr.toDouble(&ok);
			if(!ok){
				DBGA("Invalid format of Bird Number");			
			}
			xml+="\t<useFlockOfBirds>"+valueStr+"</useFlockOfBirds>\n";
		}
		if (defStr=="mass")	{
			valueStr = line.section(' ',1,1);	
			if(!valueStr.isEmpty()){
				loadMass = valueStr.toDouble(&ok);
				if (!ok||loadMass <= 0) {
					DBGA("Invalid mass in dynamic body file");
					file.close();
					return FAILURE;
				}
				xml+="\t<mass>"+valueStr+"</mass>\n";
			}
		}
		if (defStr=="cog") {
			valueStr = line.section(' ',1);
			valueStr = valueStr.stripWhiteSpace().simplifyWhiteSpace();
			l = QStringList::split(' ', valueStr);
			if(l.count()!= 3){
				DBGA("Invalid number of arguments in cog info");
				file.close();
				return FAILURE;
			}
			for(int i=0;i<3;i++){
				l[i].toDouble(&ok);
				if(!ok){
					DBGA("Invalid cog info: not double");
					file.close();
					return FAILURE;	
				}
			}
			xml+="\t<cog>"+valueStr+"</cog>\n";
		}
		if (defStr=="inertia_matrix") {
			xml+="\t<inertia_matrix>";
			for(int i=0;i<9;i+=3){
				line = stream.readLine().section('#',1);
				if(line.isNull()||line.isEmpty()){
					DBGA("Invalid number of arguments in inertia matrix info");
					file.close();
					return FAILURE;	
				}
				QTextStream(&line,QIODevice::ReadOnly) >> loadI[i] >> loadI[i+1] >> loadI[i+2];
				for(int j=i;j<=i+2;j++){
					loadI[j].toDouble(&ok);
					if(!ok){
						DBGA("Invalid inertia matrix info: not double");
						file.close();
						return FAILURE;		
					}
				}
			}
			for(int i=0;i<9;i++){
				xml+=loadI[i];
				if(i!=8){
					xml+=" "; 
				}
			}
			xml+="</inertia_matrix>\n";
		}
	}
	QString geometryFilename = filename.section('/',-1,-1).section('.',-2,-2)+"."+fileType;
	if(fileType=="iv"||fileType=="IV"||fileType=="wrl"||fileType=="WRL"){
		xml += "\t<geometryFile type=\"Inventor\">"+geometryFilename+"</geometryFile>\n";
	}
	else if(fileType=="off"||fileType=="OFF"){
		xml += "\t<geometryFile type=\"off\">"+geometryFilename+"</geometryFile>\n";
	}
	xml += "</root>";
	file.close();
	TiXmlDocument doc(filename.section('.',-2,-2)+".xml");
	doc.SetTabSize(8);
	doc.Parse(xml);
	doc.SaveFile();
	return SUCCESS;
}
int convertCfg2Xml(QString filename){
	filename.replace("\\","/");
	if(filename.section('.',-1,-1)!="cfg"){
		DBGA("Not a CFG file");
		return FAILURE;
	}
	QString line, fileType;
	QStringList l;
	bool ok;
	int numDOF,numChains;
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		DBGA("could not open robot configuration file: " << filename.toStdString());
		return FAILURE;
	}
	QString rootPath="";
	if(filename.contains("/")){
		rootPath = filename.section('/',0,-2);
	}
	if (rootPath.size()>0&&rootPath.at(rootPath.size()-1)!='/') {
		rootPath += "/";
	}
	QString ivDir = rootPath+"iv/";
	QTextStream stream(&file);
	QString	xml = "<?xml version=\"1.0\" ?>\n";
	xml += "<robot type = \"";
	stream.seek(0);
	if (!nextValidLine(&stream, &line)) {
		DBGA("Unexpected end of file looking for class type");
		file.close();
		return FAILURE;
	}
	xml += line.stripWhiteSpace().section(' ',0,0)+"\">\n\t<palm>";
	if (!nextValidLine(&stream, &line)) {
		DBGA("Unexpected end of file looking for base");
		file.close();
		return FAILURE;
	}
	line =  line.stripWhiteSpace().simplifyWhiteSpace();
	line = line.section(' ',0,0);
	QString palmName = line;
	fileType = palmName.section('.',-1,-1);
	if(fileType!="xml"){
		if(!palmName.contains(".")){
			DBGA("Failed to load base file: "<<palmName.latin1());		
			return FAILURE;
		}
		if(convertObj2Xml(ivDir+palmName)==FAILURE){
			DBGA("Failed to load base file: "<<palmName.latin1());
			return FAILURE;	
		}
		palmName = palmName.section(".",-2,-2)+".xml";
	}
	xml += palmName+"</palm>\n";
	//read number of DOFs 
	if (!nextValidLine(&stream, &line)){
		DBGA("Unexpected end of file looking for number of DOFs");
		file.close();
		return FAILURE;
	}
	numDOF = line.toInt(&ok);
	if (!ok || numDOF < 1) {
		DBGA("Wrong number of DOFs specified: " << line.latin1());
		file.close();
		return FAILURE;
	}
	//read each dof information
	for (int f=0;f<numDOF;f++) {
		if (!nextValidLine(&stream, &line)) {
			DBGA("Unexpected end of file looking for DOF " << f);
			file.close();
			return FAILURE;
		}
		line = line.simplifyWhiteSpace().stripWhiteSpace();
		QTextStream strStream(&line,QIODevice::ReadOnly);
		char dofType;
		strStream >> dofType;
		l = QStringList::split(' ', line);
		if(l.count()<6){
			DBGA("Too few arguments in DOF " << f);
			file.close();
			return FAILURE;
		}
		xml+="\t<dof type = \""+QString(dofType)+"\">\n";
		xml+= "\t\t<defaultVelocity>"+l[1]+"\n\t\t</defaultVelocity>\n";
		xml+= "\t\t<maxEffort>"+l[2]+"\n\t\t</maxEffort>\n";
		xml+= "\t\t<Kp>"+l[3]+"\n\t\t</Kp>\n";
		xml+= "\t\t<Kd>"+l[4]+"\n\t\t</Kd>\n";
		xml+= "\t\t<draggerScale>"+l[5]+"\n\t\t</draggerScale>\n";
		if(dofType=='b'){//break away
			if(l.count()<7){
				DBGA("Too few arguments in DOF " << f);
				file.close();
				return FAILURE;
			}
			xml+= "\t\t<breakAwayTorque>"+l[6]+"\n\t\t</breakAwayTorque>\n";
		}
		xml+="\t</dof>\n";
	}
	if (!nextValidLine(&stream, &line)) {
		DBGA("Unexpected end of file looking for number of chains");
		file.close();
		return FAILURE;
	}
	numChains  = line.toInt(&ok);	
	if (!ok || numChains < 1) {
		DBGA("Wrong number of chains: " << line.latin1()); 
		file.close();
		return FAILURE;
	}
	int numJ, numL;
	for (int f=0; f<numChains; f++) {
		xml+="\t<chain>\n";
		if (!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for number of joints");	
			file.close();
			return FAILURE;
		}
		numJ = line.toInt(&ok);
		if (!ok || numJ < 1) {
			DBGA("number of joints < 1");
			file.close();
			return FAILURE;
		}

		if (!nextValidLine(&stream, &line)) {
			DBGA("No more lines after number of joints");
			file.close();
			return FAILURE;
		}

		numL = line.toInt(&ok);
		if (!ok || numL < 1) {
			DBGA("Number of links < 1");
			file.close();
			return FAILURE;
		}
		

		convertTransform(stream,xml);
		for (int j=0; j<numJ; j++) {//Joints
			if (!nextValidLine(&stream, &line)) {
				DBGA("Unexpected end of file looking for Joint "<<j);
				file.close();
				return FAILURE;
			}
			line = line.simplifyWhiteSpace().stripWhiteSpace();
			l = QStringList::split(QChar(' '),line);
			if(l.count()<6){
				DBGA("Joint "<<j<<" error: Too few arguments");
			}
			QString jointType;
			if(line[0]=='d')
				jointType ="Revolute";
			else
				jointType = "Prismatic";
			xml+="\t\t<joint type = \""+jointType+"\">\n";
			xml+="\t\t\t<theta>"+l[0]+"</theta>\n";
			xml+="\t\t\t<d>"+l[1]+"</d>\n";
			xml+="\t\t\t<a>"+l[2]+"</a>\n";
			xml+="\t\t\t<alpha>"+l[3]+"</alpha>\n";
			xml+="\t\t\t<minValue>"+l[4]+"</minValue>\n";
			xml+="\t\t\t<maxValue>"+l[5]+"</maxValue>\n";

			if(l.count()>6){
				xml+="\t\t\t<viscousFriction>"+l[6]+"</viscousFriction>\n";
			}
			if(l.count()>7){
				xml+="\t\t\t<CoulombFriction>"+l[7]+"</CoulombFriction>\n";
			}
			if(l.count()>8){
				xml+="\t\t\t<springStiffness>"+l[8]+"</springStiffness>\n";
			}
			if(l.count()>9){
				xml+="\t\t\t<restValue>"+l[9]+"</restValue>\n";
			}				
			xml+="\t\t</joint>\n";
		}
		for (int j=0; j<numL; j++) {//Links

			if(!nextValidLine(&stream, &line)){
				DBGA("Unexpected end of file looking for Link "<<j<<"'s type");
				file.close();
				return FAILURE;				
			}
			QString dynJointType(line);
			dynJointType = dynJointType.stripWhiteSpace().simplifyWhiteSpace();
			if(!nextValidLine(&stream, &line)){
				DBGA("Unexpected end of file looking for Link "<<j<<"'s filename");
				file.close();
				return FAILURE;				
			}
			QString linkName(line);
			linkName = linkName.stripWhiteSpace().simplifyWhiteSpace();
			linkName = linkName.section(' ',0,0);
			fileType = linkName.section('.',-1,-1);
			if(fileType!="xml"){
				if(!linkName.contains(".")){
					DBGA("Failed to load link file: "<<linkName.latin1());
					return FAILURE;
				}
				if(convertObj2Xml(ivDir+linkName)==FAILURE){
					DBGA("Failed to load link file: "<<linkName.latin1());
					return FAILURE;
				}
				linkName = linkName.section(".",-2,-2)+".xml";
			}
			xml+="<link dynamicJointType =\""+dynJointType+"\">"+linkName+"</link>\n";
		}
		xml+="\t</chain>\n";
	}

	//convert additional info
	stream.seek(0);
	if (findString(&stream,"Approach")) {
		xml+="\t<approachDirection>\n";
		if(!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for approach direction info: reference location");
			file.close();		
			return FAILURE;				
		}
		xml+="\t\t<referenceLocation>"+line+"</referenceLocation>\n";
		if(!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for approach direction info: direction");
			file.close();
			return FAILURE;				
		}		
		xml+="\t\t<direction>"+line+"</direction>\n";
		xml+="\t</approachDirection>\n";
	}
	stream.seek(0);
	if (findString(&stream,"EigenGrasps")) {
		if(!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for eigenGrasps filename");
			file.close();
			return FAILURE;				
		}
		line = line.stripWhiteSpace().section(" ",0,0);
		xml+="\t<eigenGrasps>"+line+"</eigenGrasps>\n";
	}
	stream.seek(0);
	if (findString(&stream,"FlockOfBirds")) {
		if(!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for bird number");
			file.close();
			return FAILURE;				
		}
		line = line.stripWhiteSpace().section(" ",0,0);
		xml+="\t<flockOfBirds number=\""+line+"\">\n";
		convertTransform(stream,xml);
		xml+="\t</flockOfBirds>\n";
	}
	stream.seek(0);
	if (findString(&stream,"VirtualContacts")) {
		if(!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for virtualContacts filename");
			file.close();		
			return FAILURE;				
		}
		line = line.stripWhiteSpace().section(" ",0,0);		
		xml+="\t<virtualContacts>"+line+"</virtualContacts>\n";
	}	
	stream.seek(0);
	if (findString(&stream,"CyberGlove")) {
		if(!nextValidLine(&stream, &line)){
			DBGA("Unexpected end of file looking for cyberGlove filename");
			file.close();		
			return FAILURE;				
		}
		line = line.stripWhiteSpace().section(" ",0,0);
		xml+="\t<cyberGlove>"+line+"</cyberGlove>\n";
	}
	xml+="</robot>";
	file.close();
	TiXmlDocument doc(filename.section('.',-2,-2)+".xml");
	doc.SetTabSize(8);
	doc.Parse(xml);
	doc.SaveFile();
	return SUCCESS;
}

int convertWld2Xml(QString filename){
	filename.replace("\\","/");
	if(filename.section('.',-1,-1)!="wld"){
		DBGA("Not a WLD file");
		return FAILURE;
	}
	QString line;
	QStringList l;
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		DBGA("could not open world file: " << filename.latin1());
		return FAILURE;
	}
	QTextStream stream(&file);
	QString	xml = "<?xml version=\"1.0\" ?>\n";
	xml += "<world>\n";
	stream.seek(0);

	QString buf, elementType, matStr, elementPath, elementName,mountFilename;
	int prevRobNum,chainNum,nextRobNum;

	while(nextValidLine(&stream, &elementType)) {
		if (elementType.isEmpty()) continue;
		elementType = elementType.stripWhiteSpace();
		if (elementType == "Obstacle") {
			xml+="\t<obstacle>\n";
			if(!nextValidLine(&stream, &elementName)){
				DBGA("Unexpected end of file looking for the obstacle filename");
				file.close();
				return FAILURE;
			}
			elementName = elementName.stripWhiteSpace();
			xml+="\t\t<filename>"+elementName+"</filename>\n";
			if(!nextValidLine(&stream, &matStr)){
				DBGA("Unexpected end of file looking for the obstacle material");
				file.close();
				return FAILURE;
			}
			if (convertTransform(stream,xml)==FAILURE) {
				DBGA("Obstacle Transform Error");
				file.close();
				return FAILURE;
			}
			xml+="\t</obstacle>\n";
		}
		else if (elementType == "GraspableBody") {
			xml+="\t<graspableBody>\n";
			if(!nextValidLine(&stream, &elementName)){
				DBGA("Unexpected end of file looking for the graspable body filename");
				file.close();
				return FAILURE;
			}
			elementName = elementName.stripWhiteSpace();
			xml+="\t\t<filename>"+elementName+"</filename>\n";
			if (convertTransform(stream,xml)==FAILURE) {
				DBGA("Graspable Body Transform Error");
				file.close();
				return FAILURE;
			}
			xml+="\t</graspableBody>\n";
		}
		else if (elementType == "Robot") {
			xml+="\t<robot>\n";				
			if(!nextValidLine(&stream, &elementName)){
				DBGA("Unexpected end of file looking for the obstacle filename");
				file.close();
				return FAILURE;
			}
			elementName = elementName.stripWhiteSpace();
			xml+="\t\t<filename>"+elementName+"</filename>\n";
			QString dofVals;
			if(!nextValidLine(&stream, &dofVals)){
				DBGA("Unexpected end of file looking for robot's DOF Values");
				file.close();
				return FAILURE;
			}
			xml+="\t\t<dofValues>"+dofVals+"</dofValues>\n";
			if (convertTransform(stream,xml)==FAILURE) {
				DBGA("Robot Transform Error");
				file.close();
				return FAILURE;
			}
			xml+="\t</robot>\n";						
		}
		else if (elementType == "Connection") {
			if(!nextValidLine(&stream, &line)){
				DBGA("Unexpected end of file looking for the connection info");
				file.close();
				return FAILURE;
			}
			line = line.simplifyWhiteSpace().stripWhiteSpace();
			QTextStream lineStream(&line,QIODevice::ReadOnly);
			lineStream >> prevRobNum >> chainNum >> nextRobNum;
			if (prevRobNum < 0 ||  nextRobNum < 0 || chainNum < 0) {
				DBGA("Negative chain connection info");
				file.close();
				return FAILURE;
			}
			QString text_int;
			xml += "\t<connection>";
			xml += "\t\t<parentRobot>"+text_int.setNum(prevRobNum)+"</parentRobot>\n";
			xml += "\t\t<parentChain>"+text_int.setNum(chainNum)+"</parentChain>\n";
			xml += "\t\t<childRobot>"+text_int.setNum(nextRobNum)+"</childRobot>\n";
			line = stream.readLine();
			QTextStream lineStream2(&line,QIODevice::ReadOnly);
			lineStream2 >> mountFilename;
			if(!mountFilename.isEmpty())
				xml+="\t\t<mountFilename>"+mountFilename+"</mountFilename>\n";
			if (convertTransform(stream,xml)==FAILURE) {
				DBGA("Robot Transform Error");
				file.close();
				return FAILURE;
			}
			xml += "\t</connection>\n";
		} else if (elementType == "Camera" ) {
			if(!nextValidLine(&stream, &line)){
				DBGA("Unexpected end of file looking for the camera info");
				file.close();
				return FAILURE;
			}
			QTextStream lineStream(&line,QIODevice::ReadOnly);
			QString px, py, pz, q1, q2, q3, q4, fd;
			lineStream >> px >> py >> pz >> q1 >> q2 >> q3 >> q4;
			if(lineStream.status()!=QTextStream::Ok){
				DBGA("Failed to read camera info");
				file.close();
				return FAILURE;
			}
			fd = "100";
			xml+="\t<camera>\n";
			xml+="\t\t<position>"+px+" "+py+" "+pz+"</position>\n";
			xml+="\t\t<orientation>"+q1+" "+q2+" "+q3+" "+q4+"</orientation>\n";
			xml+="\t\t<focalDistance>"+fd+"</focalDistance>\n";
			xml+="\t</camera>\n";
		}  else if (elementType == "CameraFull" ) {
			QString px, py, pz, q1, q2, q3, q4, fd;
			if(!nextValidLine(&stream, &line)){
				DBGA("Unexpected end of file looking for the camera info");
				file.close();
				return FAILURE;
			}
			QTextStream lineStream(&line);
			lineStream >> px >> py >> pz >> q1 >> q2 >> q3 >> q4 >> fd;
			if(lineStream.status()!=QTextStream::Ok){
				DBGA("Failed to read camera info");
				file.close();
				return FAILURE;
			}
			xml+="\t<camera>\n";		
			xml+="\t\t<position>"+px+" "+py+" "+pz+"</position>\n";
			xml+="\t\t<orientation>"+q1+" "+q2+" "+q3+" "+q4+"</orientation>\n";
			xml+="\t\t<focalDistance>"+fd+"</focalDistance>\n";		
		    xml+="\t</camera>\n";
		}
		else {
			DBGA(elementType.latin1()<<" is not a valid WorldElement type");
			file.close();
			return FAILURE;
		}
	}
	file.close();
	xml+="</world>";
	TiXmlDocument doc(filename.section('.',-2,-2)+".xml");
	doc.SetTabSize(8);
	doc.Parse(xml);
	doc.SaveFile();
	return SUCCESS;
}

int convertTransform(QTextStream &stream,QString &xml){
	QString line, text_double;
	QChar axis;
	double theta,x,y,z;
	bool ok;
	QStringList strings;


	while (1) {
		line = stream.readLine();
		if ( line.isNull() ) break;
		if ( !line.isEmpty() && line[0]!='#' ) break;
	}
	if(line.isNull()) return SUCCESS;//no transform
	xml+="\t\t<transform>\n";
	line = line.stripWhiteSpace().simplifyWhiteSpace();
	do{
		if ( !line.isNull() &&(line[0]=='r' || line[0]=='t' || line[0]=='T' || line[0]=='R')){
			if (line[0]=='r') {  /* rotation */
				strings = QStringList::split(QChar(' '),line);
				if (strings.count() < 3) return FAILURE;
				theta = strings[1].toDouble(&ok); if (!ok) return FAILURE;
				axis = strings[2][0];

				if (strings[0]=="rr") /* convert radian to degree */
					theta *= 180.0/M_PI;
				if (!(axis == 'x' || axis == 'y' || axis == 'z')) return FAILURE;
				xml+="\t\t\t<rotation>"+text_double.setNum(theta)+" "+axis+"</rotation>\n";
			}
			else if (line[0]=='t') { /* translation */
				strings = QStringList::split(QChar(' '),line);
				if (strings.count() < 4) return FAILURE;
				x = strings[1].toDouble(&ok); if (!ok) return FAILURE;
				y = strings[2].toDouble(&ok); if (!ok) return FAILURE;
				z = strings[3].toDouble(&ok); if (!ok) return FAILURE;
				xml+="\t\t\t<translation>"+strings[1]+" "+strings[2]+" "+strings[3]+"</translation>\n";				
			}
			else if (line[0]=='T') { /* full transform: (Quaternion)[Translation] */
				QString trStr = line.section(' ',1,-1);
		     	if(trStr.isNull()){
						DBGA("Too few arguments in full transform input");
						return FAILURE;				
					}
					xml+="\t\t\t<fullTransform>"+trStr+"</fullTransform>\n";

			}
			else if (line[0]=='R') { /* Rotation matrix */
					xml+="\t\t\t<rotationMatrix>";
					QString temp;	
					for(int i=1;i<3;i++){
						if(!nextValidLine(&stream, &line)){
							DBGA("Too few arguments in rotation matrix input");
							return FAILURE;
						}
						temp+= line+" ";
					}
					temp = temp.simplifyWhiteSpace().stripWhiteSpace();
					QStringList strings = QStringList::split(QChar(' '),temp);
					if(strings.count()!=9){
						DBGA("Wrong number of arguments in rotation matrix input");
						return FAILURE;
					}
					xml+= temp+"</rotationMatrix>\n";
			}
			line=stream.readLine();
		}
	}while ( !line.isNull() && (line[0]=='r' || line[0]=='t' || line[0]=='T' || line[0]=='R'));
	xml+="\t\t</transform>\n";
	return SUCCESS;
}

int nextValidLine(QTextStream *stream, QString *line)
{
	while (1) {
		*line = stream->readLine();
		if ( line->isNull() ) break;
		if ( !line->isEmpty() && (*line)[0]!='#' ) break;
	}
	if ( line->isNull() ) return 0;
	else return 1;
}

int nextCommentLine(QTextStream *stream, QString *line)
{
	while (1) {
		*line = stream->readLine();
		if ( line->isNull() ) break;
		if ( !line->isEmpty() && (*line)[0]=='#' ) break;
	}
	if ( line->isNull() ) return 0;
	else return 1;
}

int findString(QTextStream *stream, QString target)
{
	QString line;
	while ( nextValidLine(stream,&line) )
	{
		if ( !(QString::compare(line,target)) )
			return 1;
	}
	return 0;
}
