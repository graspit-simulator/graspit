TEMPLATE	= app
LANGUAGE	= C++

SOURCES +=  xmlconverter.cpp \ 
		../tinyxml/tinyxmlparser.cpp

HEADERS += ../tinyxml/tinyxml.h \
	     ../tinyxml/tinystr.h \
	     xmlconverter.h

INCLUDEPATH += ../tinyxml

QT +=  qt3support 

CONFIG += console