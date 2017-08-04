class Matrix;

int cgalNNQPSolverWrapper(const Matrix &Q, 
						  const Matrix &Eq, const Matrix &b,
						  const Matrix &InEq, const Matrix &ib,
						  Matrix &sol);