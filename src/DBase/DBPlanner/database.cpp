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
// $Id: database.cpp,v 1.10 2009/12/01 18:51:18 cmatei Exp $
//
//######################################################################

/*! \file
    \brief Defines members of the %Table and %DatabaseConnection classes.
 */

#include <iostream>
#include <utility>
#include <QSqlQuery>
#include <QVariant>
#include "graspit/DBase/DBPlanner/database.h"
using std::cerr;
using std::make_pair;

//#define PROF_ENABLED
#include "graspit/profiling.h"

PROF_DECLARE(TABLE_POPULATE);

namespace db_planner {

//! Take the result of a SQL query and copy the results to a Table object.
/*! Note that this is always a copy, and does not allow for using a recordset or cursor. */
bool Table::Populate(QSqlQuery query) {
  PROF_TIMER_FUNC(TABLE_POPULATE);
  data_.clear();
  column_names_.clear();
  if (query.next()) {
    QSqlRecord first_record = query.record();
    num_columns_ = first_record.count();
    data_.resize(num_columns_);
    for (int i = 0; i < num_columns_; ++i) {
      column_names_.insert(make_pair(first_record.fieldName(i).toStdString(), i));
      //cerr << "Column name: " << first_record.fieldName(i).toStdString() << "\n";
    }
    do {
      for (int i = 0; i < num_columns_; ++i) {
        data_[i].push_back(query.value(i));
      }
    } while (query.next());
    if (!data_.empty()) { num_rows_ = data_[0].size(); }
    return true;
  }
  return false;
}

//! Constructor that opens a database connection.
/*! Multiple connections to the database can coexist as separate
    DatabaseConnection objects without any extra work. */
DatabaseConnection::DatabaseConnection(const string &host_name,
                                       const int port,
                                       const string &user_name,
                                       const string &password,
                                       const string &database_name,
                                       const string &connection_type)
  : db_(QSqlDatabase::addDatabase(QString(connection_type.c_str()),
                                  QString::number(++connection_num_))) {
  db_.setHostName(QString(host_name.c_str()));
  db_.setPort(port);
  db_.setUserName(QString(user_name.c_str()));
  db_.setPassword(QString(password.c_str()));
  db_.setDatabaseName(QString(database_name.c_str()));
  if (!db_.open()) {
    cerr << "Database " << database_name << "@" << host_name
         << ":" << port << " could not be opened.\n";
    cerr << "Error: " << db_.lastError().text().latin1() << std::endl;
    connected_ = false;
  } else {
    cerr << "Database successfully opened.\n";
    connected_ = true;
  }
}

//! Query the database using a SQL string and store the results in a provided Table.
/*! If the query is not expected to return any results, the table can be NULL */
bool DatabaseConnection::Query(const string &sql, Table *table) const {
  //std::cerr << "Query: " << sql << "\n";
  QSqlQuery query(db_);
  query.setForwardOnly(true);
  if (!query.exec(sql.c_str())) {
    std::cerr << "SQL Query failure on query: " << sql << std::endl;
    std::cerr << "Error: " << query.lastError().text().latin1() << std::endl;
    return false;
  } else {
    if (table) {
      if (!table->Populate(query)) {
        //std::cerr << "SQL Table Populate failure on query: " << sql << std::endl;
        return false;
      }
    }
    return true;
  }
  //return (query.exec(sql.c_str())) ? (table != NULL ? table->Populate(query)
  //                                         : true)
  //                         : false;
}


int DatabaseConnection::connection_num_ = 0;

}  // end namespace db_planner


