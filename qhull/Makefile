# Makefile for qhull and rbox
#
#       see README
#
#       make           to produce qhull, rbox
#       make qhullx    to produce qhull w/o using libqhull.a
#       make doc       to print documentation
#       make install   to copy qhull, rbox, qhull.1, rbox.1 to BINDIR, MANDIR
#       make new       to rebuild qhull and rbox from source
#
#       make printall  to print all files
#       make user_eg   to produce user_eg
#       make clean     to remove object files and core
#       make cleanall  to remove all generated files
#
#       PRINTMAN --  command for printing manual pages
#       PRINTC --  command for printing C files
#       BINDIR -- directory where to copy executables
#       MANDIR -- directory where to copy manual pages
#       CC --     ANSI C or C++ compiler
#       CCOPTS1 - options used to compile .c files
#       CCOPTS2 -- options used to link .o files
#
#       CFILES -- .c files for printing
#       HFILES -- .h files for printing
#       DFILES -- documentation files
#       MFILES -- man pages and html files
#       TFILES -- .txt versions of html html files
#       FILES -- all other files
#       OBJS -- specifies the object files of libqhull.a
#
BINDIR  = ./bin
MANDIR  = ./man/man1

# if you do not have enscript, try a2ps or just use lpr.  The files are text.
PRINTMAN = enscript -2rl
PRINTC = enscript -2r
# PRINTMAN = lpr
# PRINTC = lpr

#for Gnu's gcc compiler -O2 for optimization, -g for debugging, -Wall for check
#
CC     = gcc
CCOPTS1 = -g #-O2 -ansi

# for Sun's cc compiler, -fast or O2 for optimization, -g for debugging, -Xc for ANSI
#CC = cc
#CCOPTS1 = -Xc -v -fast

# for Silicon Graphics cc compiler, -O2 for optimization, -g for debugging
#CC = cc
#CCOPTS1 = -ansi -O2

# for Next cc compiler with fat executable
#CC = cc
#CCOPTS1 = -ansi -O2 -arch m68k -arch i386 -arch hppa

# for loader, ld
CCOPTS2 = $(CCOPTS1)

# OBJS in execution frequency order.  CFILES after qhull.c are alphabetical
OBJS = user.o global.o stat.o io.o geom2.o poly2.o \
       merge.o qhull.o geom.o poly.o set.o mem.o

CFILES= unix.c qhull.c geom.c geom2.c global.c io.c mem.c merge.c poly.c \
        poly2.c set.c stat.c user.c
HFILES= user.h qhull.h qhull_a.h geom.h io.h mem.h merge.h poly.h set.h stat.h
DFILES= Announce.txt REGISTER.txt COPYING.txt README.txt Changes.txt
TFILES= rbox.txt qhull.txt
FILES=  Makefile rbox.c user_eg.c q_egtest q_eg
MFILES= qhull.man rbox.man qh-man.htm qh-faq.htm \
        qh-rbox.htm qh-impre.htm qh-opt.htm qh-eg.htm qh-c.htm

all: rbox qhull 

unix.o:   qhull.h user.h mem.h
qhull.o:  $(HFILES)
geom.o:   $(HFILES)
geom2.o:  $(HFILES)
global.o: $(HFILES)
io.o:     $(HFILES)
mem.o:    mem.h 
merge.o:  $(HFILES)
poly.o:   $(HFILES)
poly2.o:  $(HFILES)
set.o:    set.h mem.h 
stat.o:   $(HFILES)
user.o:   $(HFILES)

.c.o:
	$(CC) -c $(CCOPTS1) $<

clean:
	rm -f *.o core

cleanall: clean
	rm -f libqhull.a rbox qhull user_eg >/dev/null

doc: 
	$(PRINTMAN) $(DFILES) $(TFILES)

install: all 
	cp qhull $(BINDIR)/qhull
	cp rbox $(BINDIR)/rbox
	cp qhull.man $(MANDIR)/qhull.1
	cp rbox.man $(MANDIR)/rbox.1

new:    cleanall all

printall: doc printh printc printf

printc:
	$(PRINTC) $(CFILES)

printf:
	$(PRINTC) $(FILES) 

printh:
	$(PRINTC) $(HFILES)

libqhull.a: $(OBJS)
	@echo if 'ar' or 'ranlib' fails, try 'make qhullx'
	ar r libqhull.a $(OBJS)
	@echo the next line may need to be removed.
	-test -x /bin/ranlib -o -x /usr/bin/ranlib && ranlib libqhull.a

qhull: unix.o libqhull.a
	$(CC) -o qhull $(CCOPTS2) unix.o -L. -lqhull -lm # -ldmalloc
	-chmod +x q_eg q_egtest
	-./rbox D4 | ./qhull

# compile qhull without using libqhull.a
qhullx: unix.o $(OBJS)
	$(CC) -o qhull $(CCOPTS2) unix.o $(OBJS) -lm # -ldmalloc
	-chmod +x q_eg q_egtest
	-./rbox D4 | ./qhull

rbox: rbox.o
	$(CC) -o rbox rbox.o $(CCOPTS2) -lm # -ldmalloc

user_eg: user_eg.o libqhull.a 
	$(CC)  -o user_eg $(CCOPTS2) user_eg.o  -L. -lqhull -lm # -ldmalloc

# end of Makefile
