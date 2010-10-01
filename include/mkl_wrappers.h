//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Andrew T. Miller 
//
// $Id: mkl_wrappers.h,v 1.4 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

/*! \file
  \brief c wrappers for the math kernel library fortran functions so they may be called from c++ routines
*/

#include "mkl_blas.h"
#include "mkl_lapack.h"

static __inline void daxpy(int n,double da,double *dx,int incx,double *dy,
			   int incy)
{
  daxpy(&n,&da,dx,&incx,dy,&incy);
}

static __inline void dcopy(int n,double *dx,int incx,double *dy,int incy)
{
  dcopy(&n,dx,&incx,dy,&incy);
}

static __inline double ddot(int n,double *dx,int incx,double *dy,int incy)
{
    return ddot(&n,dx,&incx,dy,&incy);
}

static __inline void dgels(char *trans,int m,int n,int nrhs,double *a,
			   int lda,double *b,int ldb,double *work,
			   int lwork,int *info)
{
  dgels(trans,&m,&n,&nrhs,a,&lda,b,&ldb,work,&lwork,info);
}

static __inline void dgelss(int m, int n, int nrhs, 
							double *a, int lda, double *b, int ldb, double *s, 
							double rcond, int *rank,
							double *work, int lwork, int *info)
{
	dgelss(&m, &n, &nrhs, a, &lda, b, &ldb, s, &rcond, rank, work, &lwork, info);
}


static __inline void dgemm(char *transa,char *transb,int m,int n,int k,
			   double alpha,double *a,int lda,double *b,int ldb,
			   double beta,double *c,int ldc)
{
    dgemm(transa,transb,&m,&n,&k,&alpha,a,&lda,b,&ldb,&beta,c,&ldc);
}


static __inline void dgemv(char *trans,int m,int n,double alpha,
			   double *a,int lda,double *dx,int incx,
			   double beta,double *dy,int incy)
{
     dgemv(trans,&m,&n,&alpha,a,&lda,dx,&incx,&beta,dy,&incy);
}


static __inline void dgesv(int n, int nrhs, double *a, int lda, int *ipiv,
			   double *b, int ldb, int *info)
{
  dgesv(&n,&nrhs,a,&lda,ipiv,b,&ldb,info);
}


static __inline void dgesvd(char *jobu,char *jobvt, int m, int n, double *a,
			    int lda, double *s, double *u, int ldu, double *vt,
			    int ldvt,double *work, int lwork, int *info)
{
  dgesvd(jobu,jobvt,&m,&n,a,&lda,s,u,&ldu,vt,&ldvt,work,&lwork,info);
}



static __inline void dgetrf(int m,int n,double *a,int lda,int *ipiv,int *info)
{
  dgetrf(&m,&n,a,&lda,ipiv,info);
}



static __inline void dgetri(int n,double *a,int lda,int *ipiv,double *work,
			    int lwork,int *info)
{
  dgetri(&n,a,&lda,ipiv,work,&lwork,info);
}



static __inline void dlascl(char *type,int kl,int ku,double from,double to,
			    int m,int n,double *a,int lda,int *info)
{
  dlascl(type,&kl,&ku,&from,&to,&m,&n,a,&lda,info);
}



static __inline double dnrm2(int n,double *dx,int incx)
{
  return dnrm2(&n,dx,&incx);
}



static __inline void dpptrf(char *uplo,int n,double *ap,int *info)
{
  dpptrf(uplo,&n,ap,info);
}


static __inline void dscal(int n,double da,double *dx,int incx)
{
  dscal(&n,&da,dx,&incx);
}

static __inline void dspgst(int itype,char *uplo,int n,double *ap,
			    double *bp,int *info)
{
  dspgst(&itype,uplo,&n,ap,bp,info);
}



static __inline void dspev(char *jobz,char *uplo, int n, double *ap,
			   double *w, double *z, int ldz, double *work,
			   int *info)
{
  dspev(jobz,uplo,&n,ap,w,z,&ldz,work,info);
}



static __inline void dspgv(int itype,char *jobz,char *uplo, int n, double *ap,
			   double *bp, double *w,double *z, int ldz,
			   double *work,int *info)
{
  dspgv(&itype,jobz,uplo,&n,ap,bp,w,z,&ldz,work,info);
}



static __inline void dtptri(char *uplo,char *diag,int n,double *ap,int *info)
{
  dtptri(uplo,diag,&n,ap,info);
}


static __inline void dtrcon(char *norm,char *uplo,char *diag,int n,double *a,
			    int lda,double *rcond,double *work,int *iwork,
			    int *info)
{
  dtrcon(norm,uplo,diag,&n,a,&lda,rcond,work,iwork,info);
}

static __inline void dorgqr(int m, int n, int k, double *a, int lda, 
							double *tau, double *work, int lwork, int *info)
{
  dorgqr(&m, &n, &k, a, &lda, tau, work, &lwork, info); 
}

static __inline void dgeqp3(int m, int n, double *a, int lda, int *jpvt, 
							double *tau, double *work, int lwork, int *info)
{
	dgeqp3(&m, &n, a, &lda, jpvt, tau, work, &lwork, info);
}

static __inline void dtrtrs(char *uplo, char *trans, char *diag, int n,
							int nrhs, double *a, int lda, double *b,
							int ldb, int *info)
{
	dtrtrs(uplo, trans, diag, &n, &nrhs, a, &lda, b, &ldb, info);
}