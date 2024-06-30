// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

namespace MiniSlamGraph {
/** This is an interface class for doing some linear algebra. Note that for
    general use within the K_OPTIM optimization framework, mostly the methods
    solve() and clone() are required.
*/
class Matrix {
 public:
  /** destructor */
  virtual ~Matrix(void) {}

  /** Virtual copy constructor */
  virtual Matrix *clone(void) const = 0;

  /** 将A*b存入x中。b和x必须有正确的维度。稀疏矩阵可能提供非常高效的实现！
   * Multiply A*b and store result in x. Both b and x are assumed to
      have correct dimensions. Sparse matrices may provide very
      efficient implementations!
  */
  virtual void multiply(const double *b, double *x) const = 0;

  /** 求解A*x=b中的x。需保证b和x有正确的维度。如果A是奇异矩阵，返回false，否则返回true。
   * Solves A*x=b for x. Both b and x are assumed to have correct
      dimensions. If A is singular, false is returned, otherwise true.
  */
  virtual bool solve(const double *b, double *x) const = 0;

  /** 返回矩阵(i,i)位置的元素。i从0开始。Return a reference to the i-th diagonal element. Start counting at 0! */
  virtual const double &diag(int i) const = 0;
  /** 返回矩阵(i,i)位置的元素。i从0开始。Return a reference to the i-th diagonal element. Start counting at 0! */
  virtual double &diag(int i) = 0;

  /** 返回矩阵行数。Return the number of rows in the matrix. */
  virtual int numRows(void) const = 0;
  /** 返回矩阵列数。Return the number of columns in the matrix. */
  virtual int numCols(void) const = 0;

  /** 将对角元素增加lambda，即A += lambda * diag(A) */
  virtual void addDiagonal(double lambda);
  /** 将对角元素扩大lambda倍，即A += lambda * diag(A) */
  virtual void multDiagonal(double lambda);
};

/** This is a reimplementation of Matrix for symmetric, positive definite
    matrices. The method solve() then uses Cholesky decomposition.
*/
class MatrixSymPosDef : public Matrix {
 public:
  /** will allocate new memory */
  MatrixSymPosDef(int dim);
  /** copy constructor */
  MatrixSymPosDef(const MatrixSymPosDef &src);

  ~MatrixSymPosDef(void);

  /** */
  MatrixSymPosDef *clone(void) const {
    return new MatrixSymPosDef(*this);
  }

  /** */
  bool solve(const double *b, double *x) const;
  /** */
  bool multisolve(const double *B, double *X, int num, int ldb = -1) const;

  /** */
  int numRows(void) const { return size; }
  /** */
  int numCols(void) const { return size; }

  /** */
  const double &ele(int row, int col) const {
    return memory[row + col * size];
  }
  /** */
  double &ele(int row, int col) {
    return memory[row + col * size];
  }

  /** */
  virtual const double &diag(int i) const {
    return ele(i, i);
  }
  /** */
  virtual double &diag(int i) {
    return ele(i, i);
  }

  const double *getMemory(void) const { return memory; }
  double *getMemory(void) { return memory; }

  /** Compute the matrix-vector product A * b. */
  virtual void multiply(const double *b, double *result) const;

 private:
  double *memory;
  int size;
};

}

