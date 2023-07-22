// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

namespace ORUtils {
template<class F>
class GenericCholesky {
 private:
  std::vector<F> cholesky;
  int size, rank;

 public:
  /**
   * @brief Cholesky分解？？？
   * 参考视频教程https://www.bilibili.com/video/BV1Ag411M76G
   * @param[in] mat 
   * @param[in] size 
   */
  GenericCholesky(const F *mat, int size) {
    this->size = size;
    this->cholesky.resize(size * size);

    for (int i = 0; i < size * size; i++) cholesky[i] = mat[i];

    for (int c = 0; c < size; c++) {
      F inv_diag = 1;
      for (int r = c; r < size; r++) {
        F val = cholesky[c + r * size];
        for (int c2 = 0; c2 < c; c2++)
          val -= cholesky[c + c2 * size] * cholesky[c2 + r * size];

        if (r == c) {
          cholesky[c + r * size] = val;
          if (val == 0) { rank = r; }   // 遇到零值，即为矩阵的秩
          inv_diag = 1.0f / val;        // FIXME: val可能为0，这里的除法会报错吧？
        } else {
          cholesky[r + c * size] = val;
          cholesky[c + r * size] = val * inv_diag;
        }
      }
    }

    rank = size;
  }

  /**
   * @brief 求解Cholesky分解得到的矩阵的行列式
   * 
   * @return F 
   */
  F Determinant(void) const {
    F ret = 1.0f;
    for (int i = 0; i < size; ++i) {
      ret *= cholesky[i + i * size];
    }
    return ret * ret;
  }
  /**
   * @brief 返回通过反向替代（back-substitution）步骤求解方程Ax=b
   * 这个链接中的注释http://mrpt.ual.es/reference/1.5.5/classmrpt_1_1math_1_1_c_sparse_matrix_1_1_cholesky_decomp.html
   * @param[out] result 方程中的x
   * @param[in] v       方程中的b      
   */
  void Backsub(F *result, const F *v) const {
    std::vector<F> y(size);
    for (int i = 0; i < size; i++) {
      F val = v[i];
      for (int j = 0; j < i; j++) 
        val -= cholesky[j + i * size] * y[j];
      y[i] = val;
    }

    for (int i = 0; i < size; i++) 
      y[i] /= cholesky[i + i * size];

    for (int i = size - 1; i >= 0; i--) {
      F val = y[i];
      for (int j = i + 1; j < size; j++) 
        val -= cholesky[i + j * size] * result[j];
      result[i] = val;
    }
  }

  ~GenericCholesky(void) {
  }
};

typedef GenericCholesky<float> Cholesky;
}
