CFLAGS = -g #-DGRASPITDBG #-O
CC = g++
VC_TOP     = ..

INCLUDE_PATH = -I../include -I$(VC_TOP)/PQP_v1.1/src

OBJS = NBody.o PairData.o VInternal.o VCollide.o

TARGET = ../lib/libVCollide.a

$(TARGET) : $(OBJS) 
	ar ruv $(TARGET) $(OBJS)

.cpp.o :
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -c $<

install: $(TARGET)
	mv $(TARGET) $(VC_TOP)/lib
	cp VCollide.H $(VC_TOP)/include
	cp VCol.h $(VC_TOP)/include

clean: 
	rm -f *.o $(TARGET)


