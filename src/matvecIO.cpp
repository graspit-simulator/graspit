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
// Author(s): Andrew T. Miller 
//
// $Id: matvecIO.cpp,v 1.5 2009/03/25 22:10:04 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the QTextStream input and output operators for various matrices and vectors.
*/

#include "matvecIO.h"
#include <qstringlist.h>
//Added by qt3to4:
#include <QTextStream>
#include "mytools.h"

/*!
  Reads a vec3 from a QTextStream.  The format should be "[x y z]".
*/
QTextStream&
operator>>(QTextStream &is, vec3 &v)
{
  QChar ch;
  double x,y,z;
  is >> ch >> x >> y >> z >> ch;
  v[0] = x; v[1] = y; v[2] = z;
  
  return is;
}

/*!
  Writes a vec3 to a QTextStream in the format "[x y z]".
*/
QTextStream &
operator<<(QTextStream &os, const vec3 &v)
{
  int oldFlags = os.setf(QTextStream::showpos);
  os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ']';
  os.flags(oldFlags);
  return os;
}

/*!
  Reads a position from a QTextStream.  The format should be "[x y z]".
*/
QTextStream&
operator>>(QTextStream &is, position &p)
{
  QChar ch;
  double x,y,z;
  is >> ch >> x >> y >> z >> ch;
  p[0] = x; p[1] = y; p[2] = z;

  return is;
}

/*!
  Writes a position to a QTextStream in the format "[x y z]".
*/
QTextStream &
operator<<(QTextStream &os, const position &p)
{
  int oldFlags = os.setf(QTextStream::showpos);
  os << '[' << p[0] << ' ' << p[1] << ' ' << p[2] << ']';
  os.flags(oldFlags);
  return os;
}

/*!
  Reads a mat3 from a QTextStream.  The format should be:
  \verbatim
  [m1,1 m1,2 m1,3]
  [m2,1 m2,2 m2,3]
  [m3,1 m3,2 m3,3]
  \endverbatim
*/
QTextStream &
operator>>(QTextStream &is, mat3 &m)
{
  is >> m[0] >> m[3] >> m[6];
  is >> m[1] >> m[4] >> m[7];
  is >> m[2] >> m[5] >> m[8];
  return is;
}

/*!
  Writes a mat3 to a QTextStream in the format:
  \verbatim
  [m1,1 m1,2 m1,3]
  [m2,1 m2,2 m2,3]
  [m3,1 m3,2 m3,3]
  \endverbatim
*/
QTextStream &
operator<<(QTextStream &os, const mat3 &m)
{
  int oldFlags = os.setf(QTextStream::showpos);
  os << '[' << m[0] << ' ' << m[3] << ' ' << m[6] << ']' << endl;
  os << '[' << m[1] << ' ' << m[4] << ' ' << m[7] << ']' << endl;
  os << '[' << m[2] << ' ' << m[5] << ' ' << m[8] << ']' << endl;
  os.flags(oldFlags);
  return os;
}

/*!
  Reads a Quaternion from a QTextStream.  The format should be 
  "(qw qx qy qz)".
*/
QTextStream &
operator>>(QTextStream &is, Quaternion &q)
{
  QChar ch;
  double w,x,y,z;
  is >> ch >> w >> x >> y >> z >> ch;
  q.w = w; q.x = x; q.y = y; q.z = z;
  return is;
}

/*!
  Writes a quaternion to a QTextStream in the format "(qw qx qy qz)".
*/
QTextStream&
operator<<(QTextStream &os, const Quaternion &q)
{
  int oldFlags = os.setf(QTextStream::showpos);
  os <<'('<< q.w <<' '<< q.x <<' '<< q.y <<' '<< q.z <<')';
  os.flags(oldFlags);
  return os;
}

/*!
  Reads a transf from a QTextStream.  The format should be 
  "(qw qx qy qz)[x y z]", where the 4 vector is a quaternion describing the
  rotation and the 3 vector is a position.
*/
QTextStream&
operator>>(QTextStream &is, transf &tr)
{
  Quaternion q;
  vec3 v;

  is >> q >> v;
  tr.set(q,v);

  return is;
}

/*!
  Writes a transf to a QTextStream in the format "(qw qx qy qz)[x y z]".
*/
QTextStream &
operator<<(QTextStream &os, const transf &tr)
{
  return os << tr.rotation() << tr.translation();
}

/*!
  Reads a transform from a QTextStream that could be in a number of possible
  formats.  The first character of the line determines the format:
    - "T" means a full transform read as (qw qx qy qz)[x y z]
    - "R" means a rotation matrix read in as a mat3
    - "t" means a translation read in as x y z
    - "r" means a rotation about a coordinate axis.  The angle and then a 
      character denoting the axis ("x", "y", or "z") is read in.  If the
      second character in the line is also an "r" then the angle is assumed
      to be in radians, otherwise it is in degrees.

  The total transform starts as the identitiy, and after a transform from a
  line is read, it is multipled to the total.  This continues until the first
  charater of a line is not one of the above 4 characters.  The total transform
  is returned in \a tr.  FAILURE is returned if a formatting error is
  encountered, otherwise SUCCESS is returned.
*/
int
readTransRotFromQTextStream(QTextStream &stream,transf &tr)
{
  QString line;
  QChar axis;
  double theta,x,y,z;
  transf tmpTr;
  vec3 tmpVec;
  bool ok;
  QStringList strings;
  
  tr = transf::IDENTITY;

	while (1) {
		line = stream.readLine();
		if ( line.isNull() ) break;
		if ( !line.isEmpty() && line[0]!='#' ) break;
	}

  do{
	if ( !line.isNull() &&(line[0]=='r' || line[0]=='t' || line[0]=='T' || line[0]=='R')){
      if (line[0]=='r') {  /* rotation */
		strings = QStringList::split(QChar(' '),line);
		if (strings.count() < 3) return FAILURE;
		theta = strings[1].toDouble(&ok); if (!ok) return FAILURE;
		axis = strings[2][0];

		if (strings[0]!="rr") /* radians */
		  theta *= M_PI/180.0;
	
		if (axis == 'x') tmpVec = vec3::X;
		else if (axis == 'y') tmpVec = vec3::Y;
		else if (axis == 'z') tmpVec = vec3::Z;
		else return FAILURE;

		tmpTr = rotate_transf(theta,tmpVec);
      }
      else if (line[0]=='t') { /* translation */
		strings = QStringList::split(QChar(' '),line);
		if (strings.count() < 4) return FAILURE;
		x = strings[1].toDouble(&ok); if (!ok) return FAILURE;
		y = strings[2].toDouble(&ok); if (!ok) return FAILURE;
		z = strings[3].toDouble(&ok); if (!ok) return FAILURE;
		tmpVec = vec3(x,y,z);
		//	printf("   Reading translation transform...\n");
		tmpTr = translate_transf(tmpVec);
      }
      else if (line[0]=='T') { /* full transform: (Quaternion)[Translation] */
		QString trStr = line.section(' ',1,-1);
		QTextStream lineStream(&trStr,QIODevice::ReadOnly);
		lineStream >> tmpTr;  
      }
      else if (line[0]=='R') { /* Rotation matrix */
		mat3 tmpMat;
		stream >> tmpMat;		
		tmpTr = transf(tmpMat,vec3::ZERO);
		line=stream.readLine();
      }
      tr = tmpTr * tr;
    }
    line=stream.readLine();
  }while ( !line.isNull() && (line[0]=='r' || line[0]=='t' || line[0]=='T' || line[0]=='R'));

  return SUCCESS;
}
