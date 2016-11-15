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
// $Id: database.h,v 1.8 2009/07/02 21:07:37 cmatei Exp $
//
//######################################################################

/*! \file 
    \brief Declares the %Table and define %DatabaseConnection classes.
 */

#ifndef DB_PLANNER_DATABASE_H_
#define DB_PLANNER_DATABASE_H_

#include "qvariant_convert.h"
#include <map>
#include <string>
#include <vector>
#include <QtSql>
using std::map;
using std::string;
using std::vector;
class QVariant;

namespace db_planner {

//! A class to contain the results of a database query. 
class Table {
 private:
  //! A 2D vector that contains the actual data returned from the database.
  vector<vector<QVariant> > data_;  // This is column major!
  //! A map from column names to indices in the data_ vector.
  map<string, int> column_names_;
  int num_columns_, num_rows_;
 public:
  Table() : num_columns_(0), num_rows_(0) {}

  int NumColumns() const { return num_columns_; }

  int NumRows() const { return num_rows_; }

  //! Fill this Table from some query.
  bool Populate(QSqlQuery query);

  //! Get the column number corresponding to some column name.
  bool GetColumnIndex(const string& column_name, int* index) const {
    map<string, int>::const_iterator map_iter = column_names_.find(column_name);
    if (map_iter == column_names_.end()) return false;
    *index = map_iter->second;
    return true;
  }
               
  //! Get the value stored at (column_num, row_num) statically types as 
  //! the FieldType template parameter.
  template <class FieldType>
  bool GetField(const size_t column_num, 
                const size_t row_num, 
                FieldType* result) const {    
    return (column_num < data_.size())
        ? QVariantConvert(data_[column_num][row_num], result)
        : false;
  }
};

//! A RAII object for managing a database connection.
//! Each database connection is named, so multiply connections can exist by
//! having multiple objects of this type. Each object maintains all the 
//! information about its own connection and queries.
class DatabaseConnection {
 private:
  //! The QT database object that this manages.
  QSqlDatabase db_;
  //! A counter to distinguish connections.
  static int connection_num_;
  //! Shows if the connection was successful
  bool connected_;
 public:
  DatabaseConnection(const string& host_name,                     
                     const int port,
                     const string& user_name,
                     const string& password,
                     const string& database_name, 
                     const string& connection_type = "QPSQL");
  //! The destructor of the DatabaseConnection closes the underlying database connection.
  virtual ~DatabaseConnection() { db_.close(); }
  
  //! Queries the database with a SQL string and populates a Table with the results.
  /*! If the query succeeds the table is populated with the results. If not,
      the function returns false and table's contents are undefined.*/
  bool Query(const string& sql, Table* table = NULL) const;
  //! Shows if the connection to the database was successful
  bool isConnected() const {return connected_;}
};

}  // end namespace db_planner

#endif  // DB_PLANNER_DATABASE_H_
