CC = g++

QHULL = ../../../qhull

CFLAGS		= -g -I. -I$(QHULL) #-DPQPDEBUG

.SUFFIXES: .C .cpp

OBJECTS		= ../lib/PQP.o \
		  ../lib/BV.o \
		  ../lib/Build.o \
		  ../lib/TriDist.o

CLEAN		= $(OBJECTS) ../../lib/libPQP.so

#STATIC LIBRARY

#library: $(OBJECTS)
#	/bin/rm -f lib/libPQP.a
#	ar ruv lib/libPQP.a $(OBJECTS)
#	cp src/PQP.h include/
#	cp src/PQP_Compile.h include/
#	cp src/PQP_Internal.h include/
#	cp src/BV.h include/
#	cp src/Tri.h include/
#	cp src/PointDist.h include/
#	cp src/TriDist.h include/

#SHARED LIBRARY
library: $(OBJECTS)
	$(CC) -o ../../lib/libPQP.so -shared $(OBJECTS) -L$(QHULL) -lqhull -lm
#	cp src/PQP.h include/
#	cp src/PQP_Compile.h include/
#	cp src/PQP_Internal.h include/
#	cp src/BV.h include/
#	cp src/Tri.h include/
#	cp src/TriDist.h include/
#	cp src/PointDist.h include/
#	cp src/MatVec.h include/

../lib/BV.o: BV.cpp
	$(CC) $(CFLAGS) -c BV.cpp -o ../lib/BV.o
../lib/PQP.o: PQP.cpp
	$(CC) $(CFLAGS) -c PQP.cpp -o ../lib/PQP.o
../lib/Build.o: Build.cpp
	$(CC) $(CFLAGS) -c Build.cpp -o ../lib/Build.o
../lib/TriDist.o: TriDist.cpp
	$(CC) $(CFLAGS) -c TriDist.cpp -o ../lib/TriDist.o

clean:
	/bin/rm -f $(CLEAN)


