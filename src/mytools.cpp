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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: mytools.cpp,v 1.8 2009/09/29 22:26:16 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the load_pixmap() function.  Other misc. functions could go here too.
*/

#include <Q3MimeSourceFactory>
#include <qmime.h>
#include <q3dragobject.h>
#include <QPixmap>

#include "debug.h"
#include "mytools.h"
#include "tinyxml.h"
#include "matvec3D.h"
#include "matvecIO.h"

/*!
  Read, decode, and return the pixmap of the given name from the
  QMimeSourceFactory.
*/
QPixmap load_pixmap(const QString &name)
{
    const QMimeSource *m = Q3MimeSourceFactory::defaultFactory()->data( name );
    if ( !m )
	return QPixmap();
    QPixmap pix;
    Q3ImageDrag::decode( m, pix );
    return pix;
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

/*! If the two paths have no common root, the returned path is
	identical to \a absolutePath. 
	Code adapted from;
	http://mrpmorris.blogspot.com/2007/05/convert-absolute-path-to-relative-path.html
*/
QString 
relativePath(QString absolutePath, QString relativeToDir )
{
	absolutePath.replace("\\","/");
	relativeToDir.replace("\\","/");
	QStringList absoluteDirectories = absolutePath.split( '/', QString::SkipEmptyParts );
	QStringList relativeDirectories = relativeToDir.split( '/', QString::SkipEmptyParts );

	//Get the shortest of the two paths
	int length = std::min(absoluteDirectories.count(), relativeDirectories.count());
		
	//Use to determine where in the loop we exited
	int lastCommonRoot = -1;
	int index;

	DBGP("Absolute path: " << absolutePath.latin1());
	DBGP("Relative to  : " << relativeToDir.latin1());

	//Find common root
	for (index = 0; index < length; index++) {
		if (absoluteDirectories[index] == relativeDirectories[index]) {
			lastCommonRoot = index;
		} else {
			break;
		}
	}
	DBGP("Last common root: " << lastCommonRoot);

	//If we didn't find a common prefix then return full absolute path
	if (lastCommonRoot == -1) {
		return absolutePath;
	}
	
	//Build up the relative path
	QString relativePath;

	//Add on the ..
	for (index = lastCommonRoot + 1; index < relativeDirectories.count(); index++) {
		if (relativeDirectories[index].length() > 0) {
			relativePath.append("../");
		}
	}

	//Add on the folders
	for (index = lastCommonRoot + 1; index < absoluteDirectories.count() - 1; index++) {
		relativePath.append(absoluteDirectories[index] ).append( "/" );
	}
	relativePath.append(absoluteDirectories[absoluteDirectories.count() - 1]);

	DBGP("Relative path: " << relativePath.latin1());
	return relativePath;
}

/*! Given a pointer to an XML node \a root, this will
	cycle through all the children and return the first
	whose definition string matches \a defStr. If no
	such child is found, returns NULL.
*/
const TiXmlElement*
findXmlElement(const TiXmlElement *root, QString defStr)
{
	defStr = defStr.stripWhiteSpace();
	const TiXmlElement* child = root->FirstChildElement();
	while(child!=NULL){
		if(child->Value()==defStr) break;
		child = child->NextSiblingElement();
	}
	return child;
}

/*! Given a pointer to an XML node \a root, this will
	cycle through all the children and return all children
	whose definition string matches \a defStr. If no
	such child is found, returns NULL.
*/
std::list<const TiXmlElement*>
findAllXmlElements(const TiXmlElement *root, QString defStr)
{
	defStr = defStr.stripWhiteSpace();
	std::list<const TiXmlElement*> children;
	const TiXmlElement* child = root->FirstChildElement();
	while(child!=NULL){
		if(child->Value()==defStr) {
			children.push_back(child);
		}
		child = child->NextSiblingElement();
	}
	return children;
}

/*! Given a pointer to an XML node \a root, this will
	cycle through all the children and count how many children
	whose definition string matches \a defStr. If no
	such child is found, returns 0.
*/
int countXmlElements(const TiXmlElement *root, QString defStr)
{
	int count = 0;
	const TiXmlElement* child = root->FirstChildElement();
	while(child!=NULL){
		if(child->Value()==defStr) count++;
		child = child->NextSiblingElement();
	}
	return count;
}

/*! Given a pointer to an XML node \a root, this will
	cycle through all the children and return true if such child 
	whose definition string matches \a defStr exists. If such child exists,
	this will set \a val to the double value in the first child it found.
	If no such child is found, returns false. 

*/
bool getDouble(const TiXmlElement *root, QString defStr, double& val){
	const TiXmlElement* child = root->FirstChildElement();
	while(child!=NULL){
		if(child->Value()==defStr){
			QString valueStr = child->GetText();
			val = valueStr.toDouble();
			return true;
		}
		child = child->NextSiblingElement();
	}
	return false;
}


/*! Given a pointer to an XML node \a root, this will
	cycle through all the children and return true if such child 
	whose definition string matches \a defStr exists. If such child exists,
	this will set \a val to the int value in the first child it found.
	If no such child is found, returns false. 

*/
bool getInt(const TiXmlElement *root, QString defStr,int &val){
	const TiXmlElement* child = root->FirstChildElement();
	while(child!=NULL){
		if(child->Value()==defStr){
			QString valueStr = child->GetText();
			val = valueStr.toInt();
			return true;
		}
		child = child->NextSiblingElement();
	}
	return false;
}
bool getPosition(const TiXmlElement *root, vec3 &pos){
	bool ok1, ok2, ok3;
	if(root == NULL){
		QTWARNING("The given root is not a Position Element");
		return false;
	}
	QString rootValue = root->Value();
	rootValue = rootValue.stripWhiteSpace();
	if(!(rootValue=="position"||rootValue=="orientation")){
		QTWARNING("The given root is not a Position Element");
		return false;
	}
	QString valueStr = root->GetText();
	valueStr = valueStr.simplifyWhiteSpace().stripWhiteSpace();
	QStringList l;
	l = QStringList::split(' ', valueStr);
	if(l.count()!=3){
			QTWARNING("Invalid position input");
			return false;
	}		
	double x,y,z;		
	x = l[0].toDouble(&ok1);
	y = l[1].toDouble(&ok2);
	z = l[2].toDouble(&ok3);
	if(!ok1 || !ok2 || !ok3){
		QTWARNING("Invalid position input");
		return false;
	}
	pos.set(x,y,z);
	return true;
}

bool getTransform(const TiXmlElement *root, transf &totalTran)
{
	bool ok1, ok2, ok3;
	bool ok[9];
	if(root == NULL){
		QTWARNING("The given root is not a Transform Element");
		return false;
	}
	QString rootValue = root->Value();
	rootValue = rootValue.stripWhiteSpace();
	if(rootValue!="transform"){
		QTWARNING("The given root is not a Transform Element");
		return false;
	}

	totalTran = transf::IDENTITY;

	const TiXmlElement* child = root->FirstChildElement();
	while(child!=NULL){
		transf newTran;
		QString valueStr = child->GetText();
		valueStr = valueStr.simplifyWhiteSpace().stripWhiteSpace();
		QStringList l;
		l = QStringList::split(' ', valueStr);
		QString defString = child->Value();
		defString = defString.stripWhiteSpace();
		if(defString=="translation"){
			if(l.count()!=3){
				QTWARNING("Invalid translation transformation input");
				return false;
			}		
			double x,y,z;		
			x = l[0].toDouble(&ok1);
			y = l[1].toDouble(&ok2);
			z = l[2].toDouble(&ok3);
			if(!ok1 || !ok2 || !ok3){
				QTWARNING("Invalid translation transformation input");
				return false;
			}
			newTran.set(Quaternion::IDENTITY, vec3(x,y,z));
		}
		else if(defString=="rotation"){
			if(l.count()!=2){
				QTWARNING("Invalid rotation transformation input");
				return false;
			}
			double rotationAngle = l[0].toDouble(&ok1);
			if(!ok1){
				QTWARNING("Invalid rotation transformation input");
				return false;
			}
			rotationAngle *= M_PI / 180.0;
			QString rotationAxis = l[1];
			if (rotationAxis=="x") {
				newTran.set(Quaternion(rotationAngle, vec3::X), vec3(0,0,0));
			} else if (rotationAxis=="y") {
				newTran.set(Quaternion(rotationAngle, vec3::Y), vec3(0,0,0));
			} else if (rotationAxis=="z") {
				newTran.set(Quaternion(rotationAngle, vec3::Z), vec3(0,0,0));
			} else {
				QTWARNING("Invalid rotation transformation input");
				return false;
			}
		}
		else if(defString=="rotationMatrix"){
			if(l.count()!=9){
				QTWARNING("Invalid rotation transformation input");
				return false;
			}
			double R[9];
			for(int i=0;i<9;i++){
				R[i]=l[i].toDouble(&ok[i]);
				if(!ok[i]){
					QTWARNING("Invalid rotation transformation input");
					return false;	
				}
			}
			newTran.set(Quaternion(mat3(R).transpose()), vec3(0,0,0));
		}
		else if(defString=="fullTransform"){
			QTextStream lineStream(&valueStr,QIODevice::ReadOnly);
			lineStream>>newTran;
		}
		totalTran = newTran*totalTran;
		child = child->NextSiblingElement();
	}
	return true;
}
