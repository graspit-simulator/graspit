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
// Author(s):  Corey Goldfeder
//
// $Id: qvariant_convert.h,v 1.3 2009/07/06 20:35:25 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines %QVariantConvert functions that cast a QT QVariant into a
         templated static type. 
 */

#ifndef DB_PLANNER_QVARIANT_CONVERT_H_
#define DB_PLANNER_QVARIANT_CONVERT_H_

#include <sstream>
#include <vector>
#include <QVariant>
using std::istringstream;
using std::vector;

namespace db_planner {
 
//! Convert QVariant data into a static type. 
/*! This will work for anything that QVariant can write as a 
    string and that std::stringstream can read back in. */
template <class FieldType>
static bool QVariantConvert(const QVariant& var, FieldType* result) {
	istringstream stream(var.toString().toStdString());
  stream >> *result;
  return true;
}

//! A QVariantConvert function for arrays written into a string.
/*! Input is a QVariant that is expected to be a string of values separated
    by spaces, as would be returned from a Postgresql query on an array field.
    The values are separated and then converted to the static templated type.
    WARNING: This may fail for arrays of strings with embedded commas. */
template <class FieldType>
static bool QVariantConvert(const QVariant& var, vector<FieldType>* result) {
  std::string data = var.toString().toStdString();
  const size_t last_char = data.length() - 1;
  if (data.size() >= 2 && data[0] == '{' && data[last_char] == '}') {
    for (size_t i = 1; i < last_char; ++i) if (data[i] == ',') data[i] = ' ';
    istringstream stream(data.substr(1, last_char - 1));
    FieldType field;
    while(stream >> field) result->push_back(field);
    return true;
  } 
  return false;
}

}  // end namespace db_planner


#endif  // DB_PLANNER_QVARIANT_CONVERT_H_
