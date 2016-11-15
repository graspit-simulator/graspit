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





