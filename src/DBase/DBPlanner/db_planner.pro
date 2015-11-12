TEMPLATE	= app
LANGUAGE	= C++

CONFIG		+= qt warn_on debug exceptions console rtti
QT		+= sql
DEFINES		+= COIN_DLL SOQT_DLL WIN32
INCLUDEPATH	+= $(COINDIR)/include
SOURCES 	+= cgdb_database_manager.cpp
HEADERS		+= aligner.h caching_aligner.h
HEADERS		+= features_extractor.h caching_features_extractor.h
HEADERS		+= neighbor_finder.h caching_neighbor_finder.h
HEADERS		+= database.h db_manager.h
HEADERS		+= grasp.h model.h
HEADERS		+= planner.h training_planner.h grasp_ranker.h
HEADERS		+= cgdb_database_manager.h


CONFIG(release) {
	DESTDIR = ../release
}
CONFIG(debug) {
	DESTDIR = ../debug
}