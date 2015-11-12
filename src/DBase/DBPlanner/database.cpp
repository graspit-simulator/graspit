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
// Author(s):  Corey Goldfeder
//
// $Id: database.cpp,v 1.9 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

/*! \file 
    \brief Defines members of the %Table and %DatabaseConnection classes.
 */

#include <iostream>
#include <utility>
#include <QSqlQuery>
#include <QVariant>
#include "database.h"
using std::cerr;
using std::make_pair;

//#define PROF_ENABLED
#include "profiling.h"

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
    data_.resize(num_columns_, vector<QVariant>(query.numRowsAffected()));
    for (int i = 0; i < num_columns_; ++i)
      column_names_.insert(make_pair(first_record.fieldName(i).toStdString(), i));
    do {
      for (int i = 0; i < num_columns_; ++i) 
        data_[i].push_back(query.value(i));
    } while(query.next());
    if (!data_.empty()) num_rows_ = data_[0].size();
   return true;
  }
  return false;
}

//! Constructor that opens a database connection. 
/*! Multiple connections to the database can coexist as separate 
    DatabaseConnection objects without any extra work. */
DatabaseConnection::DatabaseConnection(const string& host_name,                     
                                       const int port,
                                       const string& user_name,
                                       const string& password,
                                       const string& database_name, 
                                       const string& connection_type) 
    : db_(QSqlDatabase::addDatabase(QString(connection_type.c_str()), 
                                    QString::number(++connection_num_))) {
  db_.setHostName(QString(host_name.c_str()));
  db_.setPort(port);
  db_.setUserName(QString(user_name.c_str()));
  db_.setPassword(QString(password.c_str()));
  db_.setDatabaseName(QString(database_name.c_str()));
  /*  if (!db_.open()) {
    cerr << "Database " << database_name << "@" << host_name 
         << ":" << port << " could not be opened.\n";      
	cerr << "Error: " << db_.lastError().text().latin1() << std::endl;
	connected_ = false;
  } else {
	cerr << "Database successfully opened.\n";
	connected_ = true;
	}*/
  connected_=true;
}

//! Query the database using a SQL string and store the results in a provided Table.
/*! If the query is not expected to return any results, the table can be NULL */
bool DatabaseConnection::Query(const string& sql, Table* table) const {
  QSqlQuery query(db_);
  query.setForwardOnly(true);
  if (!query.exec(sql.c_str())) {
	  //std::cerr << "SQL Query failure: " << sql << std::endl;
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

  bool DatabaseConnection::QueryAndConnect(const string& sql, Table* table){
    if (!db_.open()) {
      cerr << "Database  could not be opened.\n Error: " 
	   << db_.lastError().text().latin1() << std::endl;
      connected_ = false;
      return false;
    }else{
   
    connected_ = Query(sql, table);
    db_.close();
    return connected_;
    }
  }

  bool DatabaseConnection::DBOpen(){
   return  db_.open();
  }
  bool DatabaseConnection::DBClose(){
    db_.close();
    return true;
  }


int DatabaseConnection::connection_num_ = 0;

}  // end namespace db_planner


