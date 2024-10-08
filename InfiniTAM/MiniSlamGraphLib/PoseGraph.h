// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "SlamGraph.h"

namespace MiniSlamGraph {
class PoseGraph : public SlamGraph {
 protected:
  /**
   * @brief 给梯度向量和hessian矩阵分配内存
   * @param[out] g 梯度向量，维度未定
   * @param[out] H hessian矩阵，维度6*6（因为位姿是6维的）
   */
  void allocateGradientAndHessian(VariableLengthVector *&g, SparseBlockMatrix *&H) const;
};
}

