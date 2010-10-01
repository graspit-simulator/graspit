Name

      qhull, rbox         Version 2.5     February 4, 1998
  
Convex hull, Delaunay triangulation, Voronoi vertices, Halfspace intersection
 
      Available from:
        <http://www.geom.umn.edu/locate/qhull>
        <ftp://geom.umn.edu/pub/software/qhull.tar.Z>
        <ftp://geom.umn.edu/pub/software/qhull2-5.zip>

      Version 1 (simplicial only):
        <ftp://geom.umn.edu/pub/software/qhull-1.0.tar.Z>
        <ftp://geom.umn.edu/pub/software/qhull.sit.hqx>
       
      News and a paper:
        <http://www.geom.umn.edu/~bradb/qhull-news.html>
        <ftp://geom.umn.edu/pub/software/qhull-96.ps.Z>

Purpose

  Qhull is a general dimension convex hull program that reads a set 
  of points from stdin, and outputs the smallest convex set that contains 
  the points to stdout.  It also generates Delaunay triangulations, Voronoi 
  diagrams, furthest-site Voronoi diagrams, and halfspace intersections.  

  Rbox is a useful tool in generating input for Qhull; it generates 
  hypercubes, diamonds, cones, circles, simplices, spirals, and random points.
  
  Qhull produces graphical output for Geomview.  This helps with
  understanding the output. <http://www.geom.umn.edu/locate/geomview>

    
Environment requirements

  Qhull and rbox should run on all 32-bit and 64-bit computers.  Use
  an ANSI C or C++ compiler to compile the program.  The software is 
  self-contained.  
  
  Qhull is copyrighted software.  Please read COPYING.txt and REGISTER.txt
  before using or distributing Qhull.


Qhull on Windows 95 and Windows NT

  The zip file, qhull2-5.zip, contains rbox.exe, qhull.exe, documentation
  files, and source files.
  
  To install Qhull:
  - Unzip the files into a directory.  You may use WinZip32 <www.hotfiles.com>
  - Open a DOS window for the directory.  You may double-click on qhull-go.bat.
    It calls doskey to enable the arrow keys.
  - Increase the size of the screen font to 8x12.
  - If the text is too dim, change the screen colors with shareware (crt.exe)
  - Execute 'qhull' for a synopsis and examples.
  - Execute 'rbox 10 | qhull' to compute the convex hull of 10 random points.
  - Execute 'rbox 10 | qhull i TO file' to write results to 'file'. 
  - If an error occurs, Windows 95 sends the error to stdout instead of stderr 
    
    
Compiling for Unix

  The Z file, qhull.tar.Z, contains documentation and source files for
  qhull and rbox.  
  
  To unpack the Z file
  - uncompress qhull.tar.Z
  - tar xf qhull.tar
  - cd qhull
  
  To compile qhull and rbox
  - in Makefile, check the CC, CCOPTS1, PRINTMAN, and PRINTC defines
      - the defaults are gcc and enscript
      - CCOPTS1 should include the ANSI flag.  It defines __STDC__
  - in user.h, check the definitions of qh_SECticks and qh_CPUclock.  
      - your system may use different functions.
  - type: make doc
      - this prints the documentation.  See also qh-man.htm, etc.
  - type: make
      - this builds: qhull rbox libqhull.a
  - if your compiler reports many errors, it is probably not a ANSI C compiler
      - you will need to set the -ansi switch or find another compiler
  - if your compiler warns about missing prototypes for fprintf() etc.
      - this is ok, your compiler should have given these in stdio.h
  - if your compiler warns about missing prototypes for memset() etc.
      - include memory.h in qhull_a.h
  - if your compiler reports "global.c: storage size of 'qh_qh' isn't known"
      - delete the initializer "={0}" in global.c, stat.c and mem.c
  - if your compiler warns about "stat.c: improper initializer"
      - this is ok, the initializer is not used
  - if you have trouble building libqhull.a with 'ar'
      - try 'make qhullx' 
  - if the code compiles, the qhull test case will automatically execute
      - if an error occurs, there's an incompatibility between machines
      - if you can, try a different compiler 
      - you can turn off the Qhull memory manager with qh_NOmem in mem.h
      - you can turn off compiler optimization (-O2 in Makefile)
      - if you find the source of the problem, please let us know
  - if you have Geomview, 
       - try  'rbox 100 | qhull G >a' and load 'a' into Geomview
       - run 'q_eg' for Geomview examples of Qhull output (see qh-eg.htm)
  - to install qhull, rbox, and their man pages:
      - define MANDIR and BINDIR
      - type 'make install'


Compiling for Windows 95 and Windows NT

  Qhull compiles as a console application in Visual C++ 5.0 at warning 
  level 3.  Make a project of all files except for rbox.c and user_eg.c.  
  Make another project for rbox.c. 

  Qhull compiles with Borland C++ 5.0 bcc32.  A Makefile is included.
  Execute 'make -f MBorland'.  If you use the Borland IDE, set the ANSI
  option in Options:Project:Compiler:Source:Language-compliance.
  
  Qhull compiles with Borland C++ 4.02 for Win32 and DOS Power Pack.  
  Use 'make -f MBorland -D_DPMI'.  Qhull 1.0 compiles with Borland 
  C++ 4.02.  For rbox 1.0, use "bcc32 -WX -w- -O2-e -erbox -lc rbox.c".  
  Use the same options for Qhull 1.0. [D. Zwick]
  
  Qhull compiles with Metrowerks C++ 1.7 with the ANSI option.

  If you turn on full warnings, the compiler will report a number of 
  unused variables, variables set but not used, and dead code.  These are
  intentional.  For example, variables may be initialized (unnecessarily)
  to prevent warnings about possible use of uninitialized variables.  

   
Compiling for the Power Macintosh

  Qhull compiles for the Power Macintosh with Metrowerk's C compiler.
  It uses the SIOUX interface to read point coordinates and return output.
  There is no graphical output.  Without options, qhull returns an abbreviated
  synopsis.  Instead of using SIOUX, Qhull may be embedded within an 
  application.  The distribution includes project files for qhull and rbox.

  Version 1 is available for Macintosh computers by ftp of qhull.sit.hqx
  It reads point coordinates from a standard file and returns output
  to a standard file.  There is no graphical output.


Compiling for other machines
 
  Some users have reported problems with compiling Qhull under Irix 5.1.  It
  compiles under other versions of Irix. 
  
  If you have troubles with the memory manager, you can turn it off by
  defining qh_NOmem in mem.h.

  You may compile Qhull with a C++ compiler.  


Distributed files

  README.txt           // instructions for installing Qhull 
  REGISTER.txt         // Qhull registration 
  COPYING.txt          // copyright notice 
  Announce.txt         // announcement 
  Changes.txt          // change history for Qhull and rbox 
  Makefile             // Makefile for Unix 'make' 
  qhull-PPC.sit.bin    // projects for Metrowerk's C++/Mac
  MBorland             // Makefile for Borland C++/Win32
  qh-faq.htm           // Frequently asked questions
  
  rbox consists of:
     rbox.exe          // Win32 executable (.zip only) 
     qh-rbox.htm       // html manual 
     rbox.man          // Unix man page 
     rbox.txt
     rbox.c            // source program 
     
  qhull consists of:
     qhull.exe         // Win32 executable (.zip only) 
     qhull-go.bat      //   DOS window     (.zip only) 
     qh-man.htm        // html manual 
     qh-in.htm
     qh-impre.htm
     qh-opt*.htm
     qh-eg.htm
     qh--4d.gif,etc.   // images for manual 
     qhull.man         // Unix man page 
     qhull.txt
     q_eg              // shell script for Geomview examples
     q_egtest          // shell script for Geomview test examples
     q_test            // shell script to test qhull
  
  top-level source files:
     qh-c.htm          // index to source files 
     user.h            // header file of user definable constants 
     qhull.h           // header file for qhull 
     unix.c            // Unix front end to qhull 
     qhull.c           // Quickhull algorithm with partitioning 
     user.c            // user re-definable functions 
     user_eg.c         // example of incorporating qhull into a user program 

  other source files:
     qhull_a.h         // include file for *.c 
     geom.c            // geometric routines 
     geom2.c
     geom.h	
     global.c          // global variables 
     io.c              // input-output routines 
     io.h
     mem.c             // memory routines, this is stand-alone code 
     mem.h
     merge.c           // merging of non-convex facets 
     merge.h
     poly.c            // polyhedron routines 
     poly2.c
     poly.h 
     set.c             // set routines, this only depends on mem.c 
     set.h
     stat.c            // statistics 
     stat.h

Authors:

  C. Bradford Barber                    Hannu Huhdanpaa
  bradb@geom.umn.edu                    hannu@geom.umn.edu
  
                    c/o The Geometry Center
                    University of Minnesota
                    400 Lind Hall
                    207 Church Street S.E.
                    Minneapolis, MN 55455
  
  This software was developed under NSF grants NSF/DMS-8920161 and
  NSF-CCR-91-15793 750-7504 at the Geometry Center and Harvard 
  University.  If you find Qhull useful, please let us know.
