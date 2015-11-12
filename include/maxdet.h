#ifndef MAXDET_H
/*
 * maxdet, version alpha
 * C source for solving MAXDET problems
 *
 * Shao-Po Wu, Lieven Vandenberghe and Stephen Boyd
 * Feb. 23, 1996, last major update
 *
 * COPYRIGHT 1996 SHAO-PO WU, LIEVEN VANDENBERGHE AND STEPHEN BOYD
 * Permission to use, copy, modify, and distribute this software for
 * any purpose without fee is hereby granted, provided that this entire
 * notice is included in all copies of any software which is or includes
 * a copy or modification of this software and in all copies of the
 * supporting documentation for such software.
 * This software is being provided "as is", without any express or
 * implied warranty.  In particular, the authors do not make any
 * representation or warranty of any kind concerning the merchantability
 * of this software or its fitness for any particular purpose.
 */

/*! \file
  \brief Constants and prototypes for the maxdet package.
*/

/*
 * macros
 */
#ifndef SQR
#define SQR(x)    ((x)*(x))
#endif
#ifndef MAX
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#endif
#ifndef MIN
#define MIN(x,y)  ((x) < (y) ? (x) : (y))
#endif

/*
 * constant parameters
 */
#define NB        32        /* block size for dgels, must be at least 1 */
#define MINABSTOL 1e-10     /* min absolute tolerance allowed */
#define MAXITERS  100       /* default max number of iterations */
#define CENTOL    1e-3      /* tolerance on Newton decrement of centering */
#define LSTOL     1e-3      /* tolerance used in line search */
#define TOLC      1e-5      /* tolerance used for dual infeasibility */
#define SIGTOL    1e-5      /* tolerance used for detecting zero steps 
                             * dF or dZ */ 
#define MINRCOND  1e-10     /* minimum rcond to declare F_i dependent */
#define LSITERUB  30        /* maximum number of line-search iterations */

#define YES       1
#define NO        0

#define VERSION   "alpha (Apr. 1996)"


#ifdef nounderscores
#define daxpy_ daxpy
#define dcopy_ dcopy
#define ddot_ ddot
#define dgemv_ dgemv
#define dnrm2_ dnrm2
#define dscal_ dscal
#endif


void mydlascl(double from,double to,int m,int n,double *A);

void disp_imat(FILE* fp, int* ip, int r, int c, int att);
void disp_mat(FILE* fp, double* dp,int r, int c, int att);
/* double inprd(double *X, double *Z, int L, int* blck_szs); */
double eig_val(double*sig, double* ap, int L, int* blck_szs, int Npd, double* work);

int maxdet(
 int m, int L, double *F, int *F_blkszs, int K, double *G, int *G_blkszs,
 double *c, double *x, double *Z, double *W, double *ul, double *hist,        
 double gamma, double abstol, double reltol, int *NTiters, double *work,
 int lwork, int *iwork, int *info, int negativeFlag
);
#define MAXDET_H
#endif
