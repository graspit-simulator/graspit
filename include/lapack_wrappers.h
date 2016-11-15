/*! \file
\brief C wrappers for the fortran functions so they may be called from C++ routines
*/

//cblas uses a different naming convention
#ifdef CLAPACK
#include "blaswrap.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

static __inline void daxpy(int n,double da,double *dx,int incx,double *dy,
			   int incy)
{
  void daxpy_(int *,double *,double *,int *,double *,int *);

  daxpy_(&n,&da,dx,&incx,dy,&incy);
}

static __inline void dcopy(int n,double *dx,int incx,double *dy,int incy)
{
  void dcopy_(int *,double *,int *, double *,int *);

  dcopy_(&n,dx,&incx,dy,&incy);
}

static __inline double ddot(int n,double *dx,int incx,double *dy,int incy)
{
    double ddot_(int *,double *,int *,double *,int *);

    return ddot_(&n,dx,&incx,dy,&incy);
}

static __inline void dgels(const char *trans,int m,int n,int nrhs,double *a,
			   int lda,double *b,int ldb,double *work,
			   int lwork,int *info)
{
  void dgels_(const char *,int *,int *,int *,double *,int *,double *,int *,double *,
	      int *,int *);

  dgels_(trans,&m,&n,&nrhs,a,&lda,b,&ldb,work,&lwork,info);
}

static __inline void dgemm(const char *transa,const char *transb,int m,int n,int k,
			   double alpha,double *a,int lda,double *b,int ldb,
			   double beta,double *c,int ldc)
{
    void dgemm_(const char *,const char *,int *,int *, int *,double *,double *,int *,
		double *,int *,double *,double *,int *);

    dgemm_(transa,transb,&m,&n,&k,&alpha,a,&lda,b,&ldb,&beta,c,&ldc);
}


static __inline void dgemv(const char *trans,int m,int n,double alpha,
			   double *a,int lda,double *dx,int incx,
			   double beta,double *dy,int incy)
{
     void dgemv_(const char *,int *,int *,double *,double *,int *,double *, 
		 int *,double *,double *,int *);

     dgemv_(trans,&m,&n,&alpha,a,&lda,dx,&incx,&beta,dy,&incy);
}


static __inline void dgesv(int n, int nrhs, double *a, int lda, int *ipiv,
			   double *b, int ldb, int *info)
{

  void dgesv_(int *,int *,double *,int *,int *,double *,int *,int *);

  dgesv_(&n,&nrhs,a,&lda,ipiv,b,&ldb,info);
}

static __inline void dgelss(int m, int n, int nrhs, double *a, int lda, double *b, int ldb, 
			    double *s, double rcond, int *rank, 
			    double *work, int lwork, int *info)
{
	void dgelss_(int *, int *, int *, double *, int *, double *, int *, 
		     double *, double *, int *, 
		     double *, int *, int *);
	dgelss_(&m, &n, &nrhs, a, &lda, b, &ldb, 
		s, &rcond, rank, 
		work, &lwork, info);
}


static __inline void dgesvd(const char *jobu,const char *jobvt, int m, int n, double *a,
			    int lda, double *s, double *u, int ldu, double *vt,
			    int ldvt,double *work, int lwork, int *info)
{

  void dgesvd_(const char *, const char *,int *,int *,double *,int *,double *,double *,
	       int *,double *,int *,double *,int *,int *);

  dgesvd_(jobu, jobvt,&m,&n,a,&lda,s,u,&ldu,vt,&ldvt,work,&lwork,info);
}



static __inline void dgetrf(int m,int n,double *a,int lda,int *ipiv,int *info)
{
  void dgetrf_(int *,int *,double *,int *,int *,int *);

  dgetrf_(&m,&n,a,&lda,ipiv,info);
}



static __inline void dgetri(int n,double *a,int lda,int *ipiv,double *work,
			    int lwork,int *info)
{
  void dgetri_(int *,double *,int *,int *,double *,int *,int *);

  dgetri_(&n,a,&lda,ipiv,work,&lwork,info);
}



static __inline void dlascl(const char *type,int kl,int ku,double from,double to,
			    int m,int n,double *a,int lda,int *info)
{
  void dlascl_(const char *,int *,int *,double *,double *, int *,
	       int *, double *,int *,int *);

  dlascl_(type,&kl,&ku,&from,&to,&m,&n,a,&lda,info);
}



static __inline double dnrm2(int n,double *dx,int incx)
{
  double dnrm2_(int *,double *,int *); 

  return dnrm2_(&n,dx,&incx);
}



static __inline void dpptrf(const char *uplo,int n,double *ap,int *info)
{
  void dpptrf_(const char *,int *,double *,int *);

  dpptrf_(uplo,&n,ap,info);
}


static __inline void dscal(int n,double da,double *dx,int incx)
{
  void dscal_(int *,double *,double *,int *);

  dscal_(&n,&da,dx,&incx);
}

static __inline void dspgst(int itype,const char *uplo,int n,double *ap,
			    double *bp,int *info)
{
  void dspgst_(int *,const char *,int *,double *,double *,int *);

  dspgst_(&itype, uplo,&n,ap,bp,info);
}



static __inline void dspev(const char *jobz,const char *uplo, int n, double *ap,
			   double *w, double *z, int ldz, double *work,
			   int *info)
{
  void dspev_(const char *,const char *,int *,double *,double *,double *,int *,double *,
	      int *);

  dspev_(jobz,uplo,&n,ap,w,z,&ldz,work,info);
}



static __inline void dspgv(int itype,const char *jobz,const char *uplo, int n, double *ap,
			   double *bp, double *w,double *z, int ldz,
			   double *work,int *info)
{
  void dspgv_(int *,const char *,const char *,int *,double *,double *,double *,double *,
	      int *,double *,int *);

  dspgv_(&itype,jobz,uplo,&n,ap,bp,w,z,&ldz,work,info);
}



static __inline void dtptri(const char *uplo,const char *diag,int n,double *ap,int *info)
{
  void dtptri_(const char *,const char *,int *,double *,int *);

  dtptri_(uplo,diag,&n,ap,info);
}


static __inline void dtrcon(const char *norm,const char *uplo,const char *diag,int n,double *a,
			    int lda,double *rcond,double *work,int *iwork,
			    int *info)
{
  void dtrcon_(const char *,const char *,const char *,int *,double *,int *,double *,double *,
	       int *,int *);

  dtrcon_(norm,uplo,diag,&n,a,&lda,rcond,work,iwork,info);
}

static __inline void dgeqp3(int m, int n, double *a, int lda, int *jpvt, 
							double *tau, double *work, int lwork, int *info)
{
  void dgeqp3_(int*, int*, double*, int*, int*, double*, double*, int*, int*);
  dgeqp3_(&m, &n, a, &lda, jpvt, tau, work, &lwork, info);
}

static __inline void dtrtrs(const char *uplo, const char *trans, const char *diag, int n, 
							int nrhs, double *a, int lda, double *b, int ldb,
							int *info)
{
  void dtrtrs_(const char*, const char*, const char*, int*, int*, double*, int*, double*, int*, int*);
  dtrtrs_(uplo, trans, diag, &n, &nrhs, a, &lda, b, &ldb, info);
}

static __inline void dorgqr(int m, int n, int k, double *a, int lda, double *tau,
							double *work, int lwork, int *info)
{
  void dorgqr_(int*, int*, int*, double*, int*, double*, double*, int*, int*);
  dorgqr_(&m, &n, &k, a, &lda, tau, work, &lwork, info);
}


#ifdef __cplusplus
}
#endif /* __cplusplus */
