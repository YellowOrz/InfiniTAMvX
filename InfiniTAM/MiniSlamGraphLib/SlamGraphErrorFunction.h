// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MatrixWrapper.h"
#include "SlamGraph.h"

namespace MiniSlamGraph {
/** 误差优化函数 */ // ? 为啥要搞成这么复杂，SlamGraphErrorFunction，里面包了Parameters和EvaluationPoint两个类，但是又没有相关的成员变量
class SlamGraphErrorFunction /*: public K_OPTIM::ErrorFunctionLeastSquares*/ {
 public:
  /** 优化参数，其实就只有位姿图的节点 */
  class Parameters/* : public K_OPTIM::OptimizationParameter */ {
   public:
    Parameters(const SlamGraph &graph);
    Parameters(const Parameters &src);
    ~Parameters(void);
    Parameters *clone(void) const { return new Parameters(*this); }
    void copyFrom(const /*Optimization*/Parameters &_src);
    //void copyValuesFrom(const OptimizationParameter & src);
    void clear(void);

    const SlamGraph::NodeIndex &getNodes(void) const {
      return mNodes;
    }
    SlamGraph::NodeIndex &getNodes(void) {
      return mNodes;
    }

   private:
    SlamGraph::NodeIndex mNodes;  // 位姿图的所有节点
  };
  /** 用于计算优化问题中的目标函数、误差、导数 */
  class EvaluationPoint /*: public K_OPTIM::ErrorFunctionLeastSquares::EvaluationPoint*/ {
   public:
    EvaluationPoint(const SlamGraphErrorFunction *parent, Parameters *para);
    ~EvaluationPoint(void);
    /** 得到误差。误差在构造函数里就算好了 */
    double f(void);
    const double *nabla_f(void);
    const Matrix *hessian_GN(void);
    const Parameters &getParameter(void) const { return *mPara; }

   private:
    void cacheGH(void);

    const SlamGraphErrorFunction *mParent;  // 误差优化函数。包含了
    const Parameters *mPara;                // 优化参数，其实就只有位姿图的节点
    double cacheF;                          // 误差的平方
    VariableLengthVector *cacheG;
    Matrix *cacheH;
  };

  SlamGraphErrorFunction(const SlamGraph &graph);

  ~SlamGraphErrorFunction(void);

  int numParameters(void) const;
  /**
   * @brief 初始化 评估点，其内部会计算误差
   * @param[in] para  优化参数
   * @return          评估点，记录了误差
   */
  EvaluationPoint *evaluateAt(/*K_OPTIM::Optimization*/Parameters *para) const;

  void applyDelta(const /*K_OPTIM::Optimization*/Parameters &para_old,
                  const double *delta, /*K_OPTIM::Optimization*/
                  Parameters &para_new) const;

  //Matrix_CSparse::Pattern* & getHessianSparsityPattern(void)
  void *&getHessianSparsityPattern(void) {
    return mSparsityPattern;
  }

  const SlamGraph *getGraph(void) const {
    return mGraph;
  }

 private:
  const SlamGraph *mGraph;  // 位姿图
  void *mSparsityPattern;   // ?稀疏矩阵的存储结构
};
}

