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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: mytools.h,v 1.12 2009/07/02 21:06:08 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Various useful macros and functions for dealing with errors and other common operations.
*/

#ifndef _MY_TOOLS_HXX_
#define _MY_TOOLS_HXX_

#include <qmessagebox.h>
#include <qpixmap.h>
#include <QString>
#include <QTextStream>
#include <iostream>

class TiXmlElement;
class transf;
class matvecIO;
class vec3;

#define SUCCESS 0
#define FAILURE -1
#define TERMINAL_FAILURE -2

void show_errors(int, char* = NULL);

#ifndef MAX
#define MAX(A,B)	((A) > (B) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A,B)	((A) < (B) ? (A) : (B))
#endif

#ifndef ROUND
#define ROUND(A)        ((A) >= 0 ? (int)((A)+.5) : -(int)(.5-(A)))
#endif

//std::ostream& operator<< (std::ostream& os, QString& str)
//{
//	return os << str.latin1();
//}

inline void printQString(QString q)
{
	std::cerr << q.latin1() << std::endl;
}

#ifndef BATCH_PROCESSING
//! Puts up a QT warning box with the given message
#define QTWARNING(MSG_) QMessageBox::warning(NULL,"GraspIt!",MSG_,QMessageBox::Ok, \
											 Qt::NoButton,Qt::NoButton)
#else
//If batch processing is enabled we supress error messages that require user attention
//alternatively, we could redirect this into a log file
#define QTWARNING(MSG_) printQString(MSG_);
#endif

//for inlining only when building in release mode
#ifdef GRASPIT_RELEASE
#define INLINE_RELEASE inline
#else
#define INLINE_RELEASE
#endif

QPixmap load_pixmap(const QString &name);

//useful macros for standardizing error printing
#define pr_error(EXPR_)                         \
{                                               \
    fprintf(stderr,">>!>> ");                   \
    fprintf(stderr,EXPR_);                      \
    fprintf(stderr,"\n");                       \
}

// EXPR_ must be of the form: (stderr,"..%...%...%...",args)
#define pr_errArgs(EXPR_)                       \
{                                               \
    fprintf(stderr,">>!>> ");                   \
    fprintf EXPR_;                              \
    fprintf(stderr,"\n");                       \
}

//! Finds the next line in a stream that is not blank or a comment
int nextValidLine(QTextStream *stream, QString *line);

//! Finds the next line that is an Inventor comment (starts with a #)
int nextCommentLine(QTextStream *stream, QString *line);

//! Finds a keyword on a line in a stream and positions the stream after it
int findString(QTextStream *stream, QString target);

//! Returns the path \a absolutePath made relative to the path \a relativeTo
QString relativePath(QString absolutePath, QString relativeToDir);

//! Returns the child of the XML node \a root of the type specified in \a defStr
const TiXmlElement* findXmlElement(const TiXmlElement *root, QString defStr);

//! Returns the children of the XML node \a root of the type specified in \a defStr
std::list<const TiXmlElement*> findAllXmlElements(const TiXmlElement *root, QString defStr);

//! Returns the number of chilren in the XML node \a root of the type specified in \a defStr
int countXmlElements(const TiXmlElement *root, QString defStr);

//! Returns the double value in the child f the XML node \a root of the type specified in \a defStr
bool getDouble(const TiXmlElement *root, QString defStr, double& val);

//! Returns the int value in the child f the XML node \a root of the type specified in \a defStr
bool getInt(const TiXmlElement *root, QString defStr, int& val);

//! Returns the position vector specified by the XML node \a root 
bool getPosition(const TiXmlElement *root, vec3 &pos);

//! Returns the total transformation specified by the XML node \a root 
bool getTransform(const TiXmlElement *root,transf &totalTran);
#endif
