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
// Author(s): Matei T. Ciocarlie
//
// $Id: matrix.h,v 1.21 2010/08/03 17:17:09 cmatei Exp $
//
//######################################################################

#ifndef _matrix_h_
#define _matrix_h_

#include <assert.h>
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <memory>
#include <cstdio>

class transf;
class mat3;

//! A class for storing and performing operations on general 2-dimensional matrices
/*! The Matrix is a general 2-dimensional matrix, along with a (growing)
	number of operations that can be performed on it. The data itself is stored as
	a column-major array of doubles. The important design decisions are:

	A Matrix only holds doubles for now. At some point I hope to template the class 
	to hold	anything you might want.

	Many of the computations are done using LAPACK. This should be really fast,
	but also has problems. One of them is that it prevents using const correctly
	for all the operations and arguments that should be const.

	A Matrix's size is determined when it is constructed, and can never be changed
	afterwards. This is somewhat inflexible, but allows us to catch size 
	mismatches at run time. In debug mode, all operations first assert(...) the 
	correct matrix sizes.

	All Matrix copies are deep copies. This avoids lots of problem with consistency,
	but can also eat up memory and slow down operations. Be particularly careful 
	when using the copy constructor, or any functions that return instances of
	the Matrix class: all are done using deep copies.

	No operators have been implemented. Most operations that involve two or more
	matrices are external to this class, and rather than returning the result, they
	take in the result as an argument. This makes it somewhat easier to catch errors,
	but also makes chaining of operations annoying.

	Also provides what I call "sequential access" so that a sparse matrix (which 
	inherits from this class) can give you its non-zero elements in linear time in 
	the number of elements. This is somewhat hackish, for now. The way sequential 
	access works, for both dense and sparse matrix, is using two functions:
	sequentialReset() and nextSequentialElement(...). See documentation of those
	functions for details.
*/
class Matrix{
private:
	//! The actual data, in column-major format
	double *mData;
	//! All the work that is common between all constructors
	void initialize(int m, int n);
	//! Copies the actual data from a column-major array in memory
	void setFromColMajor(const double *M);
	//! Copies the actual data from a row-major array in memory
	void setFromRowMajor(const double *M);

	//! Used to keep track of sequential access
	mutable int mSequentialI;
	//! Used to keep track of sequential access
	mutable int mSequentialJ;
protected:
	//! The size of this matrix, set at construction time
	int mRows, mCols;
public:
	enum Type{DENSE, SPARSE};
	//! A matrix of the given size with unspecified contents
	Matrix(int m, int n);
	//! Copy constructor
	Matrix(const Matrix &M);
	//! A matrix of the given size, with contents initialized from an array in memory
	Matrix(const double *M, int m, int n, bool colMajor);

	virtual ~Matrix();

        //! Resizes the matrix. All current data is lost.
        virtual void resize(int m, int n);

	virtual Type getType() const {return DENSE;}
	//! Not inlined as it is overloaded in SparseMatrix
	virtual double& elem(int m, int n);
	//! Not inlined as it is overloaded in SparseMatrix
	virtual const double& elem(int m, int n) const;
	//! Returns an auto_ptr to a copy of the data 
	virtual std::auto_ptr<double> getDataCopy() const;
	//! Returns a copy of the data as a column major vector of doubles
	virtual void getData(std::vector<double> *data) const;
        //! Returns the actual data pointer for this matrix
        virtual double* getDataPointer();
  
	//! The number of rows of this matrix
	int rows() const {return mRows;}
	//! The number of columns of this matrix
	int cols() const {return mCols;}
	Matrix getColumn(int c) const;
	Matrix getRow(int r) const;
	Matrix getSubMatrix(int startRow, int startCol, int rows, int cols) const;

	virtual void setAllElements(double val);
	//! Copies a block of the matrix \a m into a block of this matrix
	virtual void copySubBlock(int startRow, int startCol, int rows, int cols, 
					          const Matrix &m, int startMRow, int startMCol);
	//! Copies the entire matrix \a m into a sub-block of this matrix
	void copySubMatrix(int startRow, int startCol, const Matrix &m){
		copySubBlock(startRow, startCol, m.rows(), m.cols(), m, 0, 0);}
	//! Copies the entire matrix \a m to this matrix; sizes must be identical
	void copyMatrix(const Matrix &m){copySubMatrix(0, 0, m);}

	friend std::ostream& operator<<(std::ostream &os, const Matrix &m);
	void print(FILE *fp = stderr) const;

	//! Resets the sequential access to the matrix
	virtual void sequentialReset() const;
	//! Returns the next non-zero element, in sequential access
	virtual bool nextSequentialElement(int &i, int &j, double &val) const;
	//! Returns the default value for the elements of this matrix. 
	/*! Really only makes sense for sparse matrices. */
	virtual double getDefault() const {return 0.0;}

	//! Returns the number of elements set
	virtual int numElements() const {return mRows * mCols;}
	//! Computes the rank of the matrix using SVD
	int rank() const;
	//! Computes the Frobenius norm of the matrix: sqrt(sum_of_all_squared_elems)
	double fnorm() const;
	//! The largest absolute value of any element in this matrix
	double absMax() const;
	//! The sum of all the elements in the matrix
	double elementSum() const;
	void swapRows(int r1, int r2);
	void swapCols(int c1, int c2);
	//! Transposes this matrix in place
	virtual void transpose();
	//! Sets this matrix to identity. Not really used, use the static EYE instead
	void eye();
	//! Multiples the matrix by the scalar s
	void multiply(double s);

	//! Returns the transpose of this matrix; the original is unaffected
	Matrix transposed() const;

	//! An identity matrix of the given size
	static Matrix EYE(int m, int n);
	//! A negated identity matrix of the given size
	static Matrix NEGEYE(int m, int n);

	//! A permutation matrix initialized from a permutation vector in memory
	static Matrix PERMUTATION(int n, int *jpvt);
	//! A 4x4 transform matrix that can be used to left-multiply a 4x1 homogeneous vector
	static Matrix TRANSFORM(const transf &t);
	//! A 3x3 rotation matrix that can be used to left-multiply a 3x1 vector
	static Matrix ROTATION(const mat3 &rot);
	//! A 2D 2x2 rotation matrix that can be used to left-multiply a 2x1 vector
	static Matrix ROTATION2D(double theta);
	//! Returns a vector filled with the max value that can be expressed as a double
	static Matrix MAX_VECTOR(int rows);
	//! Returns a vector filled with the min value that can be expressed as a double
	static Matrix MIN_VECTOR(int rows);

	//! A mtrix of the given size filled with zeroes
	template <class MatrixType>
	static MatrixType ZEROES(int m, int n);

	//! Builds a block diagonal matrix from the two given matrices
	template <class MatrixType>
	static MatrixType BLOCKDIAG(const Matrix &M1, const Matrix &M2);
	//! Builds a block column matrix from the two given matrices
	template <class MatrixType>
	static MatrixType BLOCKCOLUMN(const Matrix &M1, const Matrix &M2);
	//! Builds a block row matrix from the two given matrices
	template <class MatrixType>
	static MatrixType BLOCKROW(const Matrix &M1, const Matrix &M2);

	//! Builds a block diagonal matrix from the matrices in the list
	template <class MatrixType>
	static MatrixType BLOCKDIAG(std::list<Matrix*> *blocks);
	//! Builds a block column matrix from the column matrices in the list
	template <class MatrixType>
	static MatrixType BLOCKCOLUMN(std::list<Matrix*> *blocks);
	//! Builds a block row matrix from the row matrices in the list
	template <class MatrixType>
	static MatrixType BLOCKROW(std::list<Matrix*> *blocks);

	//! Used for zero comparisons in all computations
	static const double EPS;
};

/*! This is just a stump of a Sparse Matrix class. Some of its functionality is 
	still incomplete and untested!

	A Sparse matrix has a key restriction: its elements can be individually
	accessed ONLY of the matrix is const. This means that many operations that
	work on the Matrix class will die as soon as they try to access elements in the
	sparse class. Examples include eye(), etc. At some point, I hope to 
	overload these function in the SparseMatrix class to do the right thing.
	Still, somebody might then extend the Matrix class with some function that
	will not work on the SparseMatrix; that would be a problem.

	A sparse matrix can be created with useful values by copying a dense matrix 
	into it, or as a block of dense matrices.

	This is really just a hack for using sparse matrices; I have found that a 
	nice design where dense and sparse matrices can be used inter-changeably
	and the user is not allowed to mess things up is really really hard. Ideally,
	there should be a Matrix pure abstract class and then DenseMatrix and
	SparseMatrix implementations, but that turned out to be more difficult than
	I thought.

	Probably the way forward is to replace the inner data storage of these
	classes with Boost matrices.

	For sequential access to non-zero elements in linear time, see the 
	documentation of the parent Matrix class.
*/
class SparseMatrix : public Matrix
{
private:
	//! The default value for the non-specified elements of this matrix
	double mDefaultValue;
	//! A ghost value used for hacking
	double mFoo;
	//! This hold the actual values
	/*! The key is computed from the row and column values as col*mRows + row */
	std::map<int, double> mSparseData;
	
	//! Creates a single integer key from a row and column index
	/*! WARNING: there are other function in this class that silently assume that
		this actually mimics column-major indexing, so be sure you know what you
		are doing if you change that.
	*/
	int key(int m, int n) const{
		return n*mRows + m;
	}
	//! Given a key, computes back the row and column that created it
	void reverseKey(int k, int &m, int &n) const {
		m = k % mRows;
		n = k / mRows;
		assert( key(m,n) == k );
	}

	mutable std::map<int, double>::const_iterator mSequentialIt;
public:
	//! Sets up an empty sparse matrix. Copy a dense sub-matrix into it to populate it.
	SparseMatrix(int m, int n, double defaultValue=0.0) : Matrix(0,0){
		//since we call the super constructor with 0 size, mData will be NULL
		mRows = m;
		mCols = n;
		mDefaultValue = defaultValue;
		mFoo = 0.0;
	}
	//! Copy constructor just copies the map
	SparseMatrix(const SparseMatrix &SM);
	~SparseMatrix() {
		//so that the super destructor does not delete mData
		mRows = 0;
	}

        //! Resizes the matrix. All current data is lost.
        virtual void resize(int m, int n);

	virtual Type getType() const {return SPARSE;}
	//! Returns the default value for the elements of this matrix. 
	virtual double getDefault() const {return mDefaultValue;}

	//! Returns the number of elements set explicitly, the size of the sparse matrix
	virtual int numElements() const {
		return mSparseData.size();
	}

	virtual double& elem(int m, int n);
	virtual const double& elem(int m, int n) const;
	virtual std::auto_ptr<double> getDataCopy() const;
	virtual void getData(std::vector<double> *data) const;
        //! There is no data pointer for a sparse matrix, just dies
        virtual double* getDataPointer();
	virtual void transpose();

	//! Copies a block of the matrix \a m (dense or sparse) into a block of this matrix
	void copySubBlock(int startRow, int startCol, int rows, int cols, 
				      const Matrix &m, int startMRow, int startMCol);

	virtual void setAllElements(double val){
		mDefaultValue = val;
		mSparseData.clear();
	}

	//! An identity matrix of the given size
	static SparseMatrix EYE(int m, int n);
	//! A negated identity matrix of the given size
	static SparseMatrix NEGEYE(int m, int n);

	//! Resets the sequential access to the matrix
	virtual void sequentialReset() const;
	//! Returns the next element, in sequential access
	virtual bool nextSequentialElement(int &i, int &j, double &val) const;
};

//! Performs M = L * R
void matrixMultiply(const Matrix &L, const Matrix &R, Matrix &M);
//! Performs M = L + R
void matrixAdd(const Matrix &L, const Matrix &R, Matrix &M);
//! Checks if two matrices are identical
bool matrixEqual(const Matrix &R, const Matrix &L);
//! Solves the system A*X=B with square A. X is overwritten on B. 
int triangularSolve(Matrix &A, Matrix &B);
//! Computes a solution of a non-square system A*X = B using Moore-Penrose pseudo-inverse
int linearSolveMPInv(Matrix &A, Matrix &B, Matrix &X);
//! Computes a solution of a non-square system A*X = B using SVD decomposition
int linearSolveSVD(Matrix &A, Matrix &B, Matrix &X);
//! Computes minimum norm solution of underdetermined system A*X=B even for rank-deficient A
int underDeterminedSolveQR(Matrix &A, Matrix &B, Matrix &X);
//! Computes the inverse of a square matrix
int matrixInverse(const Matrix &A, Matrix &AInv);
//! Solves a Quadratic Program
int QPSolver(const Matrix &Q, 
			 const Matrix &Eq, const Matrix &b,
			 const Matrix &InEq, const Matrix &ib,
			 const Matrix &lowerBounds, const Matrix &upperBounds,
			 Matrix &sol, double *objVal);
//! Solves a factorized Quadratic Program
int factorizedQPSolver(const Matrix &Qf, 
								  const Matrix &Eq, const Matrix &b,
								  const Matrix &InEq, const Matrix &ib,			
								  const Matrix &lowerBounds, const Matrix &upperBounds,
								  Matrix &sol, double *objVal);
//! A simple test to check that the QP solver works
void testQP();
//! Solves a linear program
int LPSolver(const Matrix &Q,
			 const Matrix &Eq, const Matrix &b,
			 const Matrix &InEq, const Matrix &ib,
			 const Matrix &lowerBounds, const Matrix &upperBounds,
			 Matrix &sol, double *objVal);
//! A simple test to check that the LP solver works
void testLP();

template <class MatrixType>
MatrixType Matrix::ZEROES(int m, int n)
{
	assert( m>0 && n>0 );
	MatrixType Z(m,n);
	Z.setAllElements(0.0);
	return Z;
}

template <class MatrixType>
MatrixType Matrix::BLOCKROW(std::list<Matrix*> *blocks)
{
	int numCols = 0;
	std::list<Matrix*>::iterator it;
	int numRows = 0;
	for(it=blocks->begin(); it!=blocks->end(); it++) {
		numCols += (*it)->cols();
		if ( (*it)->cols() ) {
			if (numRows == 0) {
				numRows = (*it)->rows();
			} else {
				assert((*it)->rows() == numRows);
			}
		}
	}
	if (!numCols) return MatrixType(0, 0);
	MatrixType C(numRows, numCols);
	numCols  = 0;
	for(it=blocks->begin(); it!=blocks->end(); it++) {
		if (!(*it)->cols()) continue;
		C.copySubMatrix(0, numCols, *(*it));
		numCols += (*it)->cols();
	}
	return C;
}


template <class MatrixType>
MatrixType Matrix::BLOCKDIAG(std::list<Matrix*> *blocks)
{
	int numRows=0, numCols=0;
	std::list<Matrix*>::iterator it;
	for(it=blocks->begin(); it!=blocks->end(); it++) {
		numRows += (*it)->rows();
		numCols += (*it)->cols();
	}
	if (!numRows || !numCols) return MatrixType(numRows, numCols);
	MatrixType B(numRows, numCols);
	B.setAllElements(0.0);
	numRows = numCols = 0;
	for(it=blocks->begin(); it!=blocks->end(); it++) {
		if (!(*it)->rows() || !(*it)->cols()) continue;
		B.copySubMatrix(numRows, numCols, *(*it));
		numRows += (*it)->rows();
		numCols += (*it)->cols();
	}
	return B;
}

template <class MatrixType>
MatrixType Matrix::BLOCKCOLUMN(std::list<Matrix*> *blocks)
{
	int numRows = 0;
	std::list<Matrix*>::iterator it;
	int numCols = 0;
	for(it=blocks->begin(); it!=blocks->end(); it++) {
		numRows += (*it)->rows();
		if ( (*it)->rows() ) {
			if (numCols == 0) {
				numCols = (*it)->cols();
			} else {
				assert((*it)->cols() == numCols);
			}
		}
	}
	if (!numRows) return MatrixType(0, 0);
	MatrixType C(numRows, numCols);
	numRows  = 0;
	for(it=blocks->begin(); it!=blocks->end(); it++) {
		if (!(*it)->rows()) continue;
		C.copySubMatrix(numRows, 0, *(*it));
		numRows += (*it)->rows();
	}
	return C;
}

template <class MatrixType>
MatrixType Matrix::BLOCKCOLUMN(const Matrix &M1, const Matrix &M2)
{
        if ( M1.rows() && M2.rows() ) {
	         assert(M1.cols() == M2.cols());
        }
	MatrixType M(M1.rows() + M2.rows(), std::max(M1.cols(), M2.cols()));
	M.copySubMatrix(0, 0, M1);
	M.copySubMatrix(M1.rows(), 0, M2);
	return M;
}

template <class MatrixType>
MatrixType Matrix::BLOCKROW(const Matrix &M1, const Matrix &M2)
{
	assert(M1.rows() == M2.rows());
	MatrixType M(M1.rows(), M1.cols() + M2.cols());
	M.copySubMatrix(0, 0, M1);
	M.copySubMatrix(0, M1.cols(), M2);
	return M;
}

template <class MatrixType>
MatrixType Matrix::BLOCKDIAG(const Matrix &M1, const Matrix &M2)
{
	MatrixType M( M1.rows() + M2.rows(), M1.cols() + M2.cols() );
	M.setAllElements(0.0);
	M.copySubMatrix(0, 0, M1);
	M.copySubMatrix(M1.rows(), M1.cols(), M2);
	return M;
}


#endif
