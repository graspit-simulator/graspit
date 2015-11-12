TEMPLATE	= vclib
LANGUAGE	= C++
CONFIG -= qt
CONFIG += staticlib
DEFINES -= UNICODE
DEFINES += _CRT_SECURE_NO_DEPRECATE

SOURCES +=  BarrettHand.cpp \ 
		BarrettHandThread.cpp \
		CyberGlove.cpp \
		Flock.cpp \
		flockThread.cpp \
		GloveTrans.cpp \
		SerialPort.cpp \
		include/soapCS8ServerV0Proxy.cpp \
		include/soapCS8ServerV1Proxy.cpp \
		include/soapCS8ServerV3Proxy.cpp \
	 	include/soapC.cpp \
		TX60L.cpp \
		$(GSOAP)/stdsoap2.cpp

HEADERS +=  BarrettHand.h \ 
		BarrettHandThread.h \
		CyberGlove.h \
		Flock.h \
		flockThread.h \
		GloveTrans.h \
		SerialPort.h \
		TX60L.h \
		include/soapCS8ServerV0Proxy.h \
		include/soapCS8ServerV1Proxy.h \
	 	include/soapCS8ServerV3Proxy.h \
		$(GSOAP)/stdsoap2.h

INCLUDEPATH += $(GSOAP)