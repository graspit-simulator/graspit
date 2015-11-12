CC = g++
CCFLAGS = -g
VC_TOP     = ..
INCLUDE_PATH = -I../include -I../src -I../PQP_v1.1/src

TARGET = ../lib/libivcollide.a
OBJECTS = ivcollide.o

#LLDOPTS = -L../lib -L../RAPID

# Libraries to link with:
#LLDLIBS  =  -lm -lVCollide -lRAPID


$(TARGET): $(OBJECTS)
	$(AR) ruv $@ $(OBJECTS)

.cpp.o :
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -c $<

install: $(TARGET)
	mv $(TARGET) $(VC_TOP)/lib

clean: 
	rm -f *.o $(TARGET)

