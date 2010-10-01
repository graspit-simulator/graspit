/*<html><pre>  -<a                             href="qh-c.htm#user"
  >-------------------------------</a><a name="TOP">-</a>

   user.c 
   user redefinable functions

   see README.txt  see COPYING.txt for copyright information.

   see qhull.h for data structures, macros, and user-callable functions.

   see user_eg.c and unix.c for examples.

   see user.h for user-definable constants

      use qh_NOmem in mem.h to turn off memory management
      use qh_NOmerge in user.h to turn off facet merging
      set qh_KEEPstatistics in user.h to 0 to turn off statistics

   This is unsupported software.  You're welcome to make changes,
   but you're on your own if something goes wrong.  Use 'Tc' to
   check frequently.  Usually qhull will report an error if 
   a data structure becomes inconsistent.  If so, it also reports
   the last point added to the hull, e.g., 102.  You can then trace
   the execution of qhull with "T4P102".  

   Please report any errors that you fix to qhull@geom.umn.edu

   call_qhull is a template for calling qhull from within your application

   if you recompile and load this module, then user.o will not be loaded
   from qhull.a

   you can add additional quick allocation sizes in qh_user_memsizes

   if the other functions here are redefined to not use qh_print...,
   then io.o will not be loaded from qhull.a.  See user_eg.c for an
   example.  We recommend keeping io.o for the extra debugging 
   information it supplies.
*/

#include "qhull_a.h" 

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

/*-------------------------------------------------
-call_qhull- template for calling qhull from inside your program
   remove #if 0, #endif to compile
   define: char qh_version[]= "...";
returns: 
   exit code (see qh_ERR... in qhull.h)

  This can be called any number of times.  
  
  To allow multiple, concurrent calls to qhull() 
    - use qh_QHpointer, qh_save_qhull, and qh_restore_qhull to swap
      the global data structure between calls.
    - initialize the first call with qh_init_A
    - initialize other calls with qh_initqhull_start
    - use qh_freeqhull(qh_ALL) instead of qh_memfreeshort
    - warning: this ought to work but it has not been fully tested.
      Let us know if you are successful.
*/
#if 0
char qh_version[] = "user_eg 97/3/31";  /* used for error messages */

int call_qhull (void) {
  int curlong, totlong, exitcode, numpoints, dim;
  coordT *points;
  boolT ismalloc;
  char *flags;

  qh_init_A (stdin, stdout, stderr, 0, NULL);
  exitcode= setjmp (qh errexit);
  if (!exitcode) {
    strcat (qh rbox_command, "user");            /* for documentation only */
    qh_initflags ("qhull Tc (your flags go here)"); /* 1st word ignored */

    /* define the input data */
    points= array;    		/* an array of point coordinates */
    ismalloc= False; 		/* True if qhull should 'free(points)' at end*/
    dim= 3;         		/* dimension of points */
    numpoints= 100; 		/* number of points */;
    /* For Delaunay triangulation ('d' or 'v'), set "qh PROJECTdelaunay= True" 
       to project points to a paraboloid.  You may project the points yourself.

       For halfspace intersection ('H'), compute the dual point array
       by "points= qh_sethalfspace_all (dim, numpoints, array, feasible);"
    */
    qh_init_B (points, numpoints, dim, ismalloc);

    qh_qhull();       		/* construct the hull */
    /*  For Voronoi centers ('v'), call qh_setvoronoi_all() */
    qh_check_output();     	/* optional */
    qh_produce_output();   	/* optional */
    if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone)
      qh_check_points();     	/* optional */
    /* exitcode == 0 */
  }
  qh NOerrexit= True;
  qh_freeqhull(!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);
  if (curlong || totlong)  	/* optional */
    fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
       totlong, curlong);
  return exitcode;
} /* call_qhull */
#endif

/*-<a                             href="qh-c.htm#user"
  >-------------------------------</a><a name="errexit">-</a>
  
  qh_errexit( exitcode, facet, ridge )
    report and exit from an error
    report facet and ridge if non-NULL
    reports useful information such as last point processed
    set qh.FORCEoutput to print neighborhood of facet

  see: 
    qh_errexit2() in qhull.c for printing 2 facets

  design:
    check for error within error processing
    compute qh.hulltime
    print facet and ridge (if any)
    report commandString, options, qh.furthest_id
    print summary and statistics (including precision statistics)
    if qh_ERRsingular
      print help text for singular data set
    exit program via long jump (if defined) or exit()      
*/
void qh_errexit(int exitcode, facetT *facet, ridgeT *ridge) {

  if (qh ERREXITcalled) {
    fprintf (qh ferr, "\nqhull error while processing previous error.  Exit program\n");
    exit(1);
  }
  qh ERREXITcalled= True;
  if (!qh QHULLfinished)
    qh hulltime= qh_CPUclock - qh hulltime;
  qh_errprint("ERRONEOUS", facet, NULL, ridge, NULL);
  fprintf (qh ferr, "\nWhile executing: %s | %s\n", qh rbox_command, qh qhull_command);
  fprintf(qh ferr, "Options selected for %s:\n%s\n", qh_version, qh qhull_options);
  if (qh furthest_id >= 0) {
    fprintf(qh ferr, "Last point added to hull was p%d.", qh furthest_id);
    if (zzval_(Ztotmerge))
      fprintf(qh ferr, "  Last merge was #%d.", zzval_(Ztotmerge));
    if (qh QHULLfinished)
      fprintf(qh ferr, "\nQhull has finished constructing the hull.");
    else if (qh POSTmerging)
      fprintf(qh ferr, "\nQhull has started post-merging.");
    fprintf (qh ferr, "\n");
  }
  if (qh FORCEoutput && (qh QHULLfinished || (!facet && !ridge)))
    qh_produce_output();
  else {
    if (exitcode != qh_ERRsingular && zzval_(Zsetplane) > qh hull_dim+1) {
      fprintf (qh ferr, "\nAt error exit:\n");
      qh_printsummary (qh ferr);
      if (qh PRINTstatistics) {
	qh_collectstatistics();
	qh_printstatistics(qh ferr, "at error exit");
	qh_memstatistics (qh ferr);
      }
    }
    if (qh PRINTprecision)
      qh_printstats (qh ferr, qhstat precision, NULL);
  }
  if (!exitcode)
    exitcode= qh_ERRqhull;
  else if (exitcode == qh_ERRsingular)
    qh_printhelp_singular(qh ferr);
  else if (exitcode == qh_ERRprec && !qh PREmerge)
    qh_printhelp_degenerate (qh ferr);
  if (qh NOerrexit) {
    fprintf (qh ferr, "qhull error while ending program.  Exit program\n");
    exit(1);
  }
  qh NOerrexit= True;
  longjmp(qh errexit, exitcode);
} /* errexit */


/*-<a                             href="qh-c.htm#user"
  >-------------------------------</a><a name="errprint">-</a>
  
  qh_errprint( fp, string, atfacet, otherfacet, atridge, atvertex )
    prints out the information of facets and ridges to fp
    also prints neighbors and geomview output
    
  notes:
    except for string, any parameter may be NULL
*/
void qh_errprint(char *string, facetT *atfacet, facetT *otherfacet, ridgeT *atridge, vertexT *atvertex) {
  int i;

  if (atfacet) {
    fprintf(qh ferr, "%s FACET:\n", string);
    qh_printfacet(qh ferr, atfacet);
  }
  if (otherfacet) {
    fprintf(qh ferr, "%s OTHER FACET:\n", string);
    qh_printfacet(qh ferr, otherfacet);
  }
  if (atridge) {
    fprintf(qh ferr, "%s RIDGE:\n", string);
    qh_printridge(qh ferr, atridge);
    if (atridge->top && atridge->top != atfacet && atridge->top != otherfacet)
      qh_printfacet(qh ferr, atridge->top);
    if (atridge->bottom
	&& atridge->bottom != atfacet && atridge->bottom != otherfacet)
      qh_printfacet(qh ferr, atridge->bottom);
    if (!atfacet)
      atfacet= atridge->top;
    if (!otherfacet)
      otherfacet= otherfacet_(atridge, atfacet);
  }
  if (atvertex) {
    fprintf(qh ferr, "%s VERTEX:\n", string);
    qh_printvertex (qh ferr, atvertex);
  }
  if (qh FORCEoutput && atfacet && !qh QHULLfinished && !qh IStracing) {
    fprintf(qh ferr, "ERRONEOUS and NEIGHBORING FACETS to output\n");
    for (i= 0; i < qh_PRINTEND; i++)  /* use fout for geomview output */
      qh_printneighborhood (qh fout, qh PRINTout[i], atfacet, otherfacet,
			    !qh_ALL);
  }
} /* errprint */


/*-<a                             href="qh-c.htm#user"
  >-------------------------------</a><a name="printfacetlist">-</a>
  
  qh_printfacetlist( fp, facetlist, facets, printall )
    print all fields for a facet list and/or set of facets to fp
    if !printall, 
      only prints good facets

  notes:
    also prints all vertices
*/
void qh_printfacetlist(facetT *facetlist, setT *facets, boolT printall) {
  facetT *facet, **facetp;

  qh_printbegin (qh ferr, qh_PRINTfacets, facetlist, facets, printall);
  FORALLfacet_(facetlist)
    qh_printafacet(qh ferr, qh_PRINTfacets, facet, printall);
  FOREACHfacet_(facets)
    qh_printafacet(qh ferr, qh_PRINTfacets, facet, printall);
  qh_printend (qh ferr, qh_PRINTfacets, facetlist, facets, printall);
} /* printfacetlist */


/*-<a                             href="qh-c.htm#global"
  >-------------------------------</a><a name="user_memsizes">-</a>
  
  qh_user_memsizes()
    allocate up to 10 additional, quick allocation sizes

  notes:
    increase maximum number of allocations in qh_initqhull_mem()
*/
void qh_user_memsizes (void) {

  /* qh_memsize (size); */
} /* user_memsizes */

