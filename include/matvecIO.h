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
// Author(s):  Andrew T. Miller 
//
// $Id: matvecIO.h,v 1.4 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Prototypes of QTextStream input and output operators for various matrices and vectors.
*/

#ifndef _MATVECIO_H_
#include <QTextStream>
#include "matvec3D.h"


QTextStream& operator>>(QTextStream &is, vec3 &v);
QTextStream& operator<<(QTextStream &os, const vec3 &v);  

QTextStream& operator>>(QTextStream &is, position &p);
QTextStream& operator<<(QTextStream &os, const position &p);

QTextStream& operator>>(QTextStream &is, mat3 &m);
QTextStream& operator<<(QTextStream &os, const mat3 &m);  

QTextStream& operator>>(QTextStream &is, Quaternion &q);
QTextStream& operator<<(QTextStream &os, const Quaternion &q);

QTextStream& operator>>(QTextStream &is, transf &tr);
QTextStream& operator<<(QTextStream &os, const transf &tr);

int readTransRotFromQTextStream(QTextStream &is,transf &tr);

#define _MATVECIO_H_
#endif





