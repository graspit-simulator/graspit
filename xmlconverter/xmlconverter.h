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
// $Id: xmlconverter.h,v 1.2 2009/06/17 23:53:20 saint Exp $
//
//######################################################################

#include <QString>
#include <QTextStream>
#include <iostream>

#define SUCCESS 0
#define FAILURE -1
#define M_PI 3.14159265358979323846
#define DBGA(STMT) std::cerr<<STMT<<std::endl;

//! Finds the next line in a stream that is not blank or a comment
int nextValidLine(QTextStream *stream, QString *line);

//! Finds the next line that is an Inventor comment (starts with a #)
int nextCommentLine(QTextStream *stream, QString *line);

//! Finds a keyword on a line in a stream and positions the stream after it
int findString(QTextStream *stream, QString target);

int convertCfg2Xml(QString filename);
int convertWld2Xml(QString filename);
int convertObj2Xml(QString filename);

int convertTransform(QTextStream &stream,QString& xml);
